/*
 * ========================================
 *  program_current.c — 控制算法层
 *  负责: PI 控制器、速度/电流/位置环、
 *        模式切换、电流反馈、电压限制
 *  被调: program.c (HAL回调链)
 *  调用: program_svpwm.c (斜坡/编码器)
 * ========================================
 */
#include "program_current.h"
#include "program_svpwm.h"

#include <math.h>

#include "foc_core.h"
#include "filter.h"
#include "motor_params.h"

/* ── 跨文件业务状态（extern 声明见 program.h） ── */
/** 电机状态机对象（定义于 program.c） */
extern motor_state_t   g_motor;
/** FOC 数学核心对象 */
extern foc_core_t      g_foc;
/** 程序层遥测对象 */
extern volatile program_telemetry_t      g_program_telemetry;
/** debug PWM 测试参数 */
extern volatile program_debug_pwm_test_t g_program_debug_pwm_test;

/* ── 电流换算 ── */

/* 函数作用：ADC 原始码值换算为相电流 (A)。
 * 输入：raw 为 ADC 码值，offset_raw 为零偏码值。输出：相电流值。
 * 调用频率：快环每拍调用。
 * 运行内容：(码值-零偏)×伏/码 → 电压 → ÷(增益×采样电阻) → 电流。 */
float program_convert_current_from_raw(uint16_t raw, uint16_t offset_raw)
{
    /** 每个 ADC 码值对应的电压 (V/count) */
    float volts_per_count = PROGRAM_ADC_REF_V / PROGRAM_ADC_FULL_SCALE_COUNTS;
    /** 采样电阻两端电压（已减零偏） */
    float sense_voltage = ((float)raw - (float)offset_raw) * volts_per_count;
    return sense_voltage / (PROGRAM_CURRENT_SENSE_GAIN * PROGRAM_SHUNT_RESISTOR_OHM);
}

/* 函数作用：ADC 原始码值换算为母线电压 (V)。
 * 输入：raw 为 ADC2 母线电压码值。输出：母线电压值。
 * 调用频率：慢任务 1kHz 调用。
 * 运行内容：码值×伏/码 → ADC 引脚电压 → ×分压比(240k+10k)/10k → 母线电压。 */
float program_convert_vbus_from_raw(uint16_t raw)
{
    /** 每个 ADC 码值对应的电压 (V/count) */
    float volts_per_count = PROGRAM_ADC_REF_V / PROGRAM_ADC_FULL_SCALE_COUNTS;
    /** ADC 引脚实际电压（分压后） */
    float adc_input_voltage = (float)raw * volts_per_count;
    return adc_input_voltage * ((PROGRAM_VBUS_R_UP_OHM + PROGRAM_VBUS_R_DOWN_OHM) / PROGRAM_VBUS_R_DOWN_OHM);
}

/* ── PI 控制器 ── */

/* 函数作用：带抗饱和的浮点 PI 控制器。
 * 输入：ref 目标值，feedback 反馈值，kp/ki 增益，dt_s 周期，integral 积分项指针，out_min/max 限幅。
 * 输出：限幅后的 PI 输出。
 * 调用频率：速度环 200Hz、电流环 10kHz、位置环 200Hz。
 * 运行内容：误差→比例项→积分候选(限幅)→总输出→抗饱和积分更新。 */
float program_run_pi_f32(float ref, float feedback, float kp, float ki,
                         float dt_s, float *integral, float out_min, float out_max)
{
    /** 误差 */
    float error;
    /** 比例项输出 */
    float p_out;
    /** 积分候选值 */
    float i_candidate;
    /** 总输出 */
    float out;
    /** 输入合法性检查：任一参数无效 → 清零积分并安全返回 */
    if ((integral == 0) || (!isfinite(ref)) || (!isfinite(feedback)) ||
        (!isfinite(kp)) || (!isfinite(ki)) || (!isfinite(dt_s)) ||
        (!isfinite(*integral)) || (!isfinite(out_min)) || (!isfinite(out_max)) ||
        (out_max < out_min)) {
        if (integral != 0) *integral = 0.0f;
        return 0.0f;
    }
    /** 误差 = 目标 - 反馈 */
    error = ref - feedback;
    /** 比例项 = kp × error */
    p_out = kp * error;
    /** 积分候选 = 上次积分 + ki × dt × error，并限幅 */
    i_candidate = *integral + (ki * dt_s * error);
    i_candidate = program_clamp_f32(i_candidate, out_min, out_max);
    /** 总输出 = P + I */
    out = p_out + i_candidate;
    /** 抗饱和：输出越限时只在不加深饱和的方向更新积分 */
    if (out > out_max) { out = out_max; if (error < 0.0f) *integral = i_candidate; }
    else if (out < out_min) { out = out_min; if (error > 0.0f) *integral = i_candidate; }
    else { *integral = i_candidate; }
    return out;
}

/* ── 电流环参数 ── */

/* 函数作用：由带宽推导电流环 PI 比例增益。
 * 输入：bandwidth_hz 期望电流环带宽。输出：kp 增益值。
 * 调用频率：系统初始化时调用一次。
 * 运行内容：kp = 2π × 带宽 × 电机等效电感(L)。 */
float program_current_loop_kp_from_bandwidth_hz(float bandwidth_hz)
{
    if ((!isfinite(bandwidth_hz)) || (bandwidth_hz <= 0.0f)) return 0.0f;
    return MOTOR_TWO_PI * bandwidth_hz * 0.0004100f;
}

/* 函数作用：由带宽推导电流环 PI 积分增益。
 * 输入：bandwidth_hz 期望电流环带宽。输出：ki 增益值。
 * 调用频率：系统初始化时调用一次。
 * 运行内容：ki = 2π × 带宽 × 电机等效电阻(R)。 */
float program_current_loop_ki_from_bandwidth_hz(float bandwidth_hz)
{
    if ((!isfinite(bandwidth_hz)) || (bandwidth_hz <= 0.0f)) return 0.0f;
    return MOTOR_TWO_PI * bandwidth_hz * 0.7250f;
}

/* ── 电流清洗与斜坡 ── */

/* 函数作用：对电流指令做合法性清洗和限幅。
 * 输入：ref_cmd 原始电流指令，limit_abs_a 绝对值限幅。输出：清洗后指令。
 * 调用频率：快环每拍调用。
 * 运行内容：非有限值或限幅无效 → 0；否则钳位到 ±limit_abs_a。 */
float program_sanitize_current_ref_cmd(float ref_cmd, float limit_abs_a)
{
    if ((!isfinite(ref_cmd)) || (!isfinite(limit_abs_a)) || (limit_abs_a <= 0.0f)) return 0.0f;
    return program_clamp_f32(ref_cmd, -limit_abs_a, limit_abs_a);
}

/* 函数作用：对单步变化量做斜率限制（斜坡核心）。
 * 输入：target 目标值，state 当前值，max_step 单步最大变化量。输出：限速后新值。
 * 调用频率：快环每拍调用。
 * 运行内容：delta=目标-当前 → 钳位 ±max_step → 当前值+delta。 */
float program_apply_slew_limit_f32(float target, float state, float max_step)
{
    /** 本步变化量 */
    float delta;
    if ((!isfinite(target)) || (!isfinite(state)) || (!isfinite(max_step)) || (max_step <= 0.0f))
        return 0.0f;
    delta = target - state;
    delta = program_clamp_f32(delta, -max_step, max_step);
    return state + delta;
}

/* 函数作用：更新实际生效的 id/iq 给定（电流参考斜坡）。
 * 输入：无。输出：无返回值。
 * 调用频率：快环每拍调用。
 * 运行内容：目标值清洗限幅 → 以 150A/s 斜率逐步逼近 → 写入 g_id/iq_ref_applied_a。 */
void program_update_applied_current_references(void)
{
    /** 电流限幅 (A) */
    float current_limit_a = g_motor.iq_limit;
    /** 单步允许的最大电流变化量 */
    float current_step_a = PROGRAM_CURRENT_REF_RAMP_A_PER_S * PROGRAM_FAST_LOOP_DT_S;
    /** 清洗后的 id 目标 */
    float id_target_a = program_sanitize_current_ref_cmd(g_motor.id_ref, current_limit_a);
    /** 清洗后的 iq 目标 */
    float iq_target_a = program_sanitize_current_ref_cmd(g_motor.iq_ref, current_limit_a);
    g_control.id_ref_applied_a = program_apply_slew_limit_f32(id_target_a, g_control.id_ref_applied_a, current_step_a);
    g_control.iq_ref_applied_a = program_apply_slew_limit_f32(iq_target_a, g_control.iq_ref_applied_a, current_step_a);
}

/* ── 电压限制 ── */

/* 函数作用：获取 SVPWM 电压矢量限幅值。
 * 输入：无。输出：电压限幅值 (V)。
 * 调用频率：快环每拍调用。
 * 运行内容：vbus × 1/√3(0.577)，母线电压低于 1V 时按 1V 处理。 */
float program_get_voltage_limit_v(void)
{
    /** 当前母线电压 (V) */
    float vbus = g_program_telemetry.vbus;
    return 0.57735026919f * ((vbus > 1.0f) ? vbus : 1.0f);
}

/* 函数作用：限制 dq 电压矢量半径不超过 SVPWM 调制范围。
 * 输入：ud_ref/uq_ref 电压矢量指针，v_limit 限幅值。输出：无返回值。
 * 调用频率：电流环每拍调用。
 * 运行内容：矢量模长 > 限幅 → 按比例缩小；参数无效 → 清零输出。 */
void program_limit_voltage_vector(float *ud_ref, float *uq_ref, float v_limit)
{
    /** 电压矢量模长 */
    float v_mag;
    /** 缩放比例 */
    float v_scale;
    if ((ud_ref == 0) || (uq_ref == 0)) return;
    if ((!isfinite(*ud_ref)) || (!isfinite(*uq_ref)) || (!isfinite(v_limit)) || (v_limit <= 0.0f)) {
        *ud_ref = 0.0f; *uq_ref = 0.0f; return;
    }
    v_mag = sqrtf((*ud_ref * *ud_ref) + (*uq_ref * *uq_ref));
    if ((v_mag > v_limit) && (v_mag > 0.0f)) {
        v_scale = v_limit / v_mag;
        *ud_ref *= v_scale; *uq_ref *= v_scale;
    }
}

/* ── 复位函数 ── */

/* 函数作用：复位电流参考斜坡，实际生效电流给定立即归零。 */
void program_reset_current_reference_ramp(void)
    { g_control.id_ref_applied_a = 0.0f; g_control.iq_ref_applied_a = 0.0f; }

/* 函数作用：复位速度环状态。
 * 输入：无。输出：无返回值。
 * 调用频率：停机/故障/模式切换时调用。
 * 运行内容：清积分、清 iq_ref、清挂起标志、恢复默认控制周期。 */
void program_reset_speed_loop(void)
{
    g_motor.speed_integral_iq = 0.0f;
    g_motor.speed_integral_uq = 0.0f;
    g_motor.iq_ref = 0.0f;
    if (g_motor.speed_loop_enable != 0U) g_motor.uq_ref = 0.0f;
    g_encoder.speed_loop_update_pending = 0U;
    g_control.speed_loop_dt_s = PROGRAM_FAST_LOOP_DT_S * 20.0f;
}

/* 函数作用：复位速度参考斜坡。
 * 输入：无。输出：无返回值。
 * 调用频率：停机/模式切换时调用。
 * 运行内容：生效速度、电角速度参考、开环速度全部归零。 */
void program_reset_speed_reference_ramp(void)
{
    g_motor.speed_ref_mech_applied_rad_s = 0.0f;
    g_motor.speed_ref_elec_rad_s = 0.0f;
    g_motor.open_loop_speed_elec = 0.0f;
}

/* 函数作用：复位位置环状态。
 * 输入：无。输出：无返回值。
 * 调用频率：停机/模式切换/位置环禁用时调用。
 * 运行内容：清积分/误差/hold 状态，重置位置测量 LPF。 */
void program_reset_position_loop(void)
{
    g_motor.position_integral_speed = 0.0f;
    g_motor.position_error_mech_deg = 0.0f;
    g_motor.position_error_mech_rad = 0.0f;
    g_control.position_loop_elapsed_s = 0.0f;
    g_control.position_hold_active = 0U;
    g_control.position_hold_release_counter = 0U;
    g_control.position_meas_output_continuous_rad = 0.0f;
    filter_lpf_f32_init(&g_control.position_meas_lpf,
        program_lpf_alpha_from_cutoff_hz(12.0f, 0.005f), 0.0f);
    g_control.position_meas_lpf.initialized = 0U;
}

/* 函数作用：复位电流环状态。
 * 输入：无。输出：无返回值。
 * 调用频率：停机/故障/模式切换时调用。
 * 运行内容：清 id/iq/ud/uq 参考、积分项、电压限幅和电流斜坡。 */
void program_reset_current_loop(void)
{
    g_motor.id_ref = 0.0f;  g_motor.iq_ref = 0.0f;
    g_motor.ud_ref = 0.0f;  g_motor.uq_ref = 0.0f;
    g_motor.id_integral_v = 0.0f;  g_motor.iq_integral_v = 0.0f;
    g_motor.voltage_limit = 0.0f;
    program_reset_current_reference_ramp();
}

/* ── 模式切换 ── */

/* 函数作用：处理位置环使能状态切换。
 * 输入：无。输出：无返回值。
 * 调用频率：快环每拍调用（内部检测边沿）。
 * 运行内容：检测使能边沿 → 复位相关环 → 启用时用当前编码器位置初始化目标，防止跳变。 */
void program_handle_position_loop_mode_switch(void)
{
    /** 位置环当前使能状态（归一化 0/1） */
    uint8_t position_loop_enable_now = (g_motor.position_loop_enable != 0U) ? 1U : 0U;
    if (position_loop_enable_now == g_control.position_loop_enable_prev) return;
    g_control.position_loop_enable_prev = position_loop_enable_now;
    /** 边沿触发：复位位置环/速度环/斜坡 */
    program_reset_position_loop();
    program_reset_speed_loop();
    program_reset_speed_reference_ramp();
    if (position_loop_enable_now != 0U) {
        if (g_encoder.speed_primed != 0U) {
            /** 编码器已就绪：用当前位置初始化目标，避免启动跳变 */
            g_control.position_meas_output_continuous_rad =
                program_get_encoder_output_continuous_mech_angle_rad();
            filter_lpf_f32_init(&g_control.position_meas_lpf,
                program_lpf_alpha_from_cutoff_hz(12.0f, 0.005f),
                g_control.position_meas_output_continuous_rad);
            g_motor.position_meas_mech_rad =
                program_wrap_angle_0_2pi(g_control.position_meas_output_continuous_rad);
            g_motor.position_meas_mech_deg =
                program_wrap_angle_0_360_deg(program_rad_to_deg(g_motor.position_meas_mech_rad));
            g_motor.position_ref_mech_deg = g_motor.position_meas_mech_deg;
            g_motor.position_ref_mech_rad = g_motor.position_meas_mech_rad;
        } else {
            /** 编码器未就绪：全部清零 */
            g_motor.position_ref_mech_deg = 0.0f; g_motor.position_ref_mech_rad = 0.0f;
            g_motor.position_meas_mech_deg = 0.0f; g_motor.position_meas_mech_rad = 0.0f;
        }
    }
}

/* 函数作用：处理电流环使能状态切换。
 * 输入：无。输出：无返回值。
 * 调用频率：快环每拍调用（内部检测边沿）。
 * 运行内容：检测使能边沿 → 复位速度环/电流环 → 清全部电压电流参考。 */
void program_handle_current_loop_mode_switch(void)
{
    /** 电流环当前使能状态（归一化 0/1） */
    uint8_t current_loop_enable_now = (g_motor.current_loop_enable != 0U) ? 1U : 0U;
    if (current_loop_enable_now == g_control.current_loop_enable_prev) return;
    g_control.current_loop_enable_prev = current_loop_enable_now;
    program_reset_speed_loop();
    program_reset_current_loop();
    g_motor.id_ref = 0.0f; g_motor.iq_ref = 0.0f;
    g_motor.ud_ref = 0.0f; g_motor.uq_ref = 0.0f;
}

/* ── 电流反馈 ── */

/* 函数作用：由三相原始 ADC 值更新电流反馈 id/iq。
 * 输入：ia/ib/ic_raw 三相码值，theta_elec 控制电角度。输出：无返回值。
 * 调用频率：快环每拍调用。
 * 运行内容：码值→电流→Clarke→Park→写入遥测 id/iq。 */
void program_update_current_feedback_from_raw(uint16_t ia_raw, uint16_t ib_raw,
                                              uint16_t ic_raw, float theta_elec)
{
    /** Clarke 变换中间量 αβ */
    foc_alpha_beta_t i_ab;
    /** Park 变换结果 dq */
    foc_dq_t i_dq;
    /** A/B/C 相电流测量值 (A) */
    float ia_meas, ib_meas, ic_meas;
    /** 电角度 sin/cos 缓存 */
    float sin_theta, cos_theta;

    /** 零偏校准未完成 → 全部清零返回 */
    if (g_program_telemetry.current_offset_ready == 0U) {
        g_program_telemetry.ia = 0.0f; g_program_telemetry.ib = 0.0f;
        g_program_telemetry.ic = 0.0f; g_program_telemetry.ic_meas = 0.0f;
        g_program_telemetry.i_abc_sum = 0.0f;
        g_program_telemetry.id = 0.0f; g_program_telemetry.iq = 0.0f;
        return;
    }

    /** 码值 → 电流，并施加接线方向符号校正 */
    ia_meas = PROGRAM_CURRENT_SIGN_IA * program_convert_current_from_raw(ia_raw, g_program_telemetry.ia_offset_raw);
    ib_meas = PROGRAM_CURRENT_SIGN_IB * program_convert_current_from_raw(ib_raw, g_program_telemetry.ib_offset_raw);
    ic_meas = PROGRAM_CURRENT_SIGN_IC * program_convert_current_from_raw(ic_raw, g_program_telemetry.ic_offset_raw);

    g_program_telemetry.ia = ia_meas;
    g_program_telemetry.ib = ib_meas;
    g_program_telemetry.ic_meas = ic_meas;
    g_program_telemetry.i_abc_sum = ia_meas + ib_meas + ic_meas;
    /** ic 由基尔霍夫定律推算（三相电流和为零） */
    g_program_telemetry.ic = -(ia_meas + ib_meas);

    /** 功率级未使能 → 不更新 id/iq */
    if (g_control.power_stage_enabled == 0U) {
        g_program_telemetry.id = 0.0f; g_program_telemetry.iq = 0.0f;
        g_foc.i_ab.alpha = 0.0f; g_foc.i_ab.beta = 0.0f;
        g_foc.i_dq.d = 0.0f; g_foc.i_dq.q = 0.0f;
        return;
    }

    /** Clarke: ia/ib → αβ */
    foc_core_clarke(g_program_telemetry.ia, g_program_telemetry.ib, &i_ab);
    /** Park: αβ → dq（用电角度旋转坐标） */
    sin_theta = sinf(theta_elec);
    cos_theta = cosf(theta_elec);
    foc_core_park(&i_ab, sin_theta, cos_theta, &i_dq);

    g_foc.i_ab = i_ab;
    g_foc.i_dq = i_dq;
    g_program_telemetry.id = i_dq.d;
    g_program_telemetry.iq = i_dq.q;
}

/* ── 位置环 ── */

/* 函数作用：位置环主逻辑（200Hz，输出轴坐标系）。
 * 输入：position_loop_dt_s 位置环实际执行周期。输出：无返回值。
 * 调用频率：速度环中分频到 200Hz 调用。
 * 运行内容：测量 LPF → 位置误差 → hold/release 滞回 → PI → creep 补偿 → 输出速度指令。 */
void program_update_position_loop(float position_loop_dt_s)
{
    /** 目标位置（归一化 rad） */
    float position_ref_wrapped_rad;
    /** 测量位置原始连续角 (rad) */
    float position_meas_raw_output_continuous_rad;
    /** 测量位置（归一化 rad） */
    float position_meas_wrapped_rad;
    /** 位置误差（±π 归一化） */
    float position_error_wrapped_rad;
    /** 位置环输出速度限幅 */
    float position_speed_limit_mech_rad_s;
    /** 位置环输出速度指令 */
    float position_speed_cmd_output_mech_rad_s;
    /** 测量输出轴速度 */
    float position_speed_meas_output_mech_rad_s;
    /** 误差绝对值 / LPF 系数 / 速度绝对值 */
    float position_error_abs_rad, position_meas_filter_alpha, speed_meas_abs_rad_s;

    /** 位置环或速度环未使能 → 清误差返回 */
    if ((g_motor.position_loop_enable == 0U) || (g_motor.speed_loop_enable == 0U)) {
        g_motor.position_error_mech_deg = 0.0f;
        g_motor.position_error_mech_rad = 0.0f;
        return;
    }
    /** 编码器观测器未就绪 → 清积分和速度指令 */
    if (g_encoder.speed_primed == 0U) {
        g_motor.position_integral_speed = 0.0f;
        g_motor.position_error_mech_deg = 0.0f;
        g_motor.position_error_mech_rad = 0.0f;
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }
    /** 周期参数异常 → 用默认 2ms */
    if ((!isfinite(position_loop_dt_s)) || (position_loop_dt_s <= 0.0f))
        position_loop_dt_s = 0.0001f * 20.0f;

    /** 目标位置归一化并转弧度 */
    g_motor.position_ref_mech_deg = program_wrap_angle_0_360_deg(g_motor.position_ref_mech_deg);
    position_ref_wrapped_rad = program_deg_to_rad(g_motor.position_ref_mech_deg);
    /** 读取输出轴连续角 */
    position_meas_raw_output_continuous_rad = program_get_encoder_output_continuous_mech_angle_rad();
    if ((!isfinite(position_ref_wrapped_rad)) ||
        (!isfinite(position_meas_raw_output_continuous_rad))) {
        program_reset_position_loop();
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    /** 更新测量 LPF 系数并滤波连续角 */
    position_meas_filter_alpha = program_lpf_alpha_from_cutoff_hz(12.0f, position_loop_dt_s);
    g_control.position_meas_lpf.alpha = position_meas_filter_alpha;
    if (g_control.position_meas_lpf.initialized == 0U)
        filter_lpf_f32_init(&g_control.position_meas_lpf, position_meas_filter_alpha,
                            position_meas_raw_output_continuous_rad);
    g_control.position_meas_output_continuous_rad =
        filter_lpf_f32_update(&g_control.position_meas_lpf, position_meas_raw_output_continuous_rad);
    if (!isfinite(g_control.position_meas_output_continuous_rad)) {
        program_reset_position_loop();
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    /** 归一化测量角并计算 ±π 误差 */
    position_meas_wrapped_rad = program_wrap_angle_0_2pi(g_control.position_meas_output_continuous_rad);
    position_error_wrapped_rad = program_wrap_delta_pm_pi(
        position_ref_wrapped_rad - position_meas_wrapped_rad);
    if (!isfinite(position_error_wrapped_rad)) {
        program_reset_position_loop();
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    /** 回写遥测：目标/测量/误差 */
    g_motor.position_ref_mech_rad = position_ref_wrapped_rad;
    g_motor.position_meas_mech_deg =
        program_wrap_angle_0_360_deg(program_rad_to_deg(position_meas_wrapped_rad));
    g_motor.position_meas_mech_rad = position_meas_wrapped_rad;
    g_motor.position_error_mech_deg = program_rad_to_deg(position_error_wrapped_rad);
    g_motor.position_error_mech_rad = position_error_wrapped_rad;
    position_error_abs_rad = fabsf(position_error_wrapped_rad);
    position_speed_meas_output_mech_rad_s = g_motor.speed_meas_mech_rad_s / MOTOR_GEAR_RATIO;
    speed_meas_abs_rad_s = fabsf(position_speed_meas_output_mech_rad_s);

    /* Hold: 位置误差和速度都在阈值内 → 刹车保持 */
    if ((position_error_abs_rad <= PROGRAM_POSITION_HOLD_ERR_RAD) &&
        (speed_meas_abs_rad_s <= PROGRAM_POSITION_HOLD_SPEED_MECH_RAD_S)) {
        g_control.position_hold_active = 1U;
        g_control.position_hold_release_counter = 0U;
        g_motor.position_integral_speed = 0.0f;
        g_motor.position_error_mech_deg = 0.0f;
        g_motor.position_error_mech_rad = 0.0f;
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }
    /** 已处于 hold：误差未超释放阈值 → 继续保持；超阈值需连续 N 周期才释放 */
    if (g_control.position_hold_active != 0U) {
        if (position_error_abs_rad <= PROGRAM_POSITION_HOLD_RELEASE_ERR_RAD) {
            g_control.position_hold_release_counter = 0U;
            g_motor.position_integral_speed = 0.0f;
            g_motor.position_error_mech_deg = 0.0f;
            g_motor.position_error_mech_rad = 0.0f;
            g_motor.speed_ref_mech_rad_s = 0.0f;
            return;
        }
        g_control.position_hold_release_counter++;
        if (g_control.position_hold_release_counter < PROGRAM_POSITION_HOLD_RELEASE_CONFIRM_CYCLES) {
            g_motor.position_integral_speed = 0.0f;
            g_motor.position_error_mech_deg = 0.0f;
            g_motor.position_error_mech_rad = 0.0f;
            g_motor.speed_ref_mech_rad_s = 0.0f;
            return;
        }
        g_control.position_hold_active = 0U;
        g_control.position_hold_release_counter = 0U;
    }

    /** 输出速度限幅有效性检查 */
    position_speed_limit_mech_rad_s = g_motor.position_speed_limit_mech_rad_s;
    if ((!isfinite(position_speed_limit_mech_rad_s)) || (position_speed_limit_mech_rad_s <= 0.0f))
        position_speed_limit_mech_rad_s = 20.0f;

    /** 位置 PI：误差 → 速度指令，并减去阻尼项 */
    position_speed_cmd_output_mech_rad_s = program_run_pi_f32(
        position_error_wrapped_rad, 0.0f, g_motor.position_kp, g_motor.position_ki,
        position_loop_dt_s, &g_motor.position_integral_speed,
        -position_speed_limit_mech_rad_s, position_speed_limit_mech_rad_s);
    position_speed_cmd_output_mech_rad_s -=
        g_motor.position_kd * position_speed_meas_output_mech_rad_s;
    position_speed_cmd_output_mech_rad_s = program_clamp_f32(
        position_speed_cmd_output_mech_rad_s,
        -position_speed_limit_mech_rad_s, position_speed_limit_mech_rad_s);

    /* Creep: 缓慢蠕动防止卡在摩擦死区 */
    if ((position_error_abs_rad > PROGRAM_POSITION_CREEP_ENABLE_ERR_RAD) &&
        (fabsf(position_speed_cmd_output_mech_rad_s) < PROGRAM_POSITION_CREEP_SPEED_MECH_RAD_S) &&
        (speed_meas_abs_rad_s <= PROGRAM_POSITION_HOLD_SPEED_MECH_RAD_S)) {
        position_speed_cmd_output_mech_rad_s =
            copysignf(PROGRAM_POSITION_CREEP_SPEED_MECH_RAD_S, position_error_wrapped_rad);
    }
    /** 输出轴速度 → 转子速度（× 减速比） */
    g_motor.speed_ref_mech_rad_s = position_speed_cmd_output_mech_rad_s * MOTOR_GEAR_RATIO;
}

/* ── 速度环 ── */

/* 函数作用：速度环主逻辑（200Hz，输出 iq_ref 或 uq_ref）。
 * 输入：无。输出：无返回值。
 * 调用频率：快环每拍调用，内部按窗口挂起标志实际 200Hz 执行。
 * 运行内容：位置环分频 → 速度参考斜坡 → 速度 PI → 输出电流或电压指令。 */
void program_update_speed_loop(void)
{
    /** PI 输出 / 位置环周期 / 速度环周期 / 输出限幅 */
    float output_cmd, position_loop_dt_s, speed_loop_dt_s, speed_output_limit;
    /** 速度环积分项指针（电流模式用 iq 积分，电压模式用 uq 积分） */
    float *speed_integral;

    /** 速度环周期有效性检查 */
    speed_loop_dt_s = g_control.speed_loop_dt_s;
    if ((!isfinite(speed_loop_dt_s)) || (speed_loop_dt_s <= 0.0f))
        speed_loop_dt_s = 0.0001f * 20.0f;

    /** 位置环禁用 → 复位；启用且窗口触发 → 累计 5ms 后执行位置环 */
    if (g_motor.position_loop_enable == 0U) {
        program_reset_position_loop();
    } else if (g_encoder.speed_loop_update_pending != 0U) {
        g_control.position_loop_elapsed_s += speed_loop_dt_s;
        if (g_control.position_loop_elapsed_s >= 0.005f) {
            position_loop_dt_s = g_control.position_loop_elapsed_s;
            g_control.position_loop_elapsed_s = 0.0f;
            program_update_position_loop(position_loop_dt_s);
        }
    }

    /** 更新速度参考斜坡 */
    program_update_speed_reference_ramp();

    /** 编码器速度未就绪 → 清输出返回 */
    if (g_encoder.speed_ready == 0U) {
        g_motor.iq_ref = 0.0f; g_motor.uq_ref = 0.0f;
        g_encoder.speed_loop_update_pending = 0U;
        if (g_motor.position_loop_enable != 0U) {
            g_motor.speed_ref_mech_rad_s = 0.0f;
            program_reset_position_loop();
        }
        return;
    }
    /** 无窗口更新 → 本拍跳过（实际 200Hz 执行） */
    if (g_encoder.speed_loop_update_pending == 0U) return;
    g_encoder.speed_loop_update_pending = 0U;

    /** 电流模式：输出限幅 = iq_limit；电压模式：输出限幅 = 电压矢量限幅 */
    if (g_motor.current_loop_enable != 0U) {
        speed_integral = &g_motor.speed_integral_iq;
        speed_output_limit = g_motor.iq_limit;
    } else {
        speed_integral = &g_motor.speed_integral_uq;
        speed_output_limit = program_get_voltage_limit_v();
        g_motor.voltage_limit = speed_output_limit;
    }

    /** 速度 PI：电角速度误差 → 输出指令 */
    output_cmd = program_run_pi_f32(g_motor.speed_ref_elec_rad_s,
                                    g_motor.speed_meas_elec_rad_s,
                                    g_motor.speed_kp, g_motor.speed_ki,
                                    speed_loop_dt_s, speed_integral,
                                    -speed_output_limit, speed_output_limit);

    /** 电流模式：输出 → iq_ref；电压模式：输出 → uq_ref */
    if (g_motor.current_loop_enable != 0U) {
        g_motor.iq_ref = output_cmd;
    } else {
        g_motor.ud_ref = 0.0f; g_motor.uq_ref = output_cmd; g_motor.iq_ref = 0.0f;
    }
}

/* ── 电流环 / 电压模式 ── */

/* 函数作用：电流环主逻辑（10kHz，输出 ud/uq 并执行 SVPWM）。
 * 输入：theta_cmd 控制电角度 (rad)。输出：无返回值。
 * 调用频率：快环每拍调用。
 * 运行内容：电流参考斜坡 → d/q 轴 PI → 电压矢量限幅 → 反 Park + SVPWM。 */
void program_run_current_loop(float theta_cmd)
{
    /** 母线电压 / 电压限幅值 */
    float vbus, v_limit;
    /** 更新实际生效的 id/iq 给定（斜坡） */
    program_update_applied_current_references();
    vbus = g_program_telemetry.vbus;
    v_limit = program_get_voltage_limit_v();
    g_motor.voltage_limit = v_limit;

    /** d 轴 PI：id_ref → 0 */
    g_motor.ud_ref = program_run_pi_f32(g_control.id_ref_applied_a, g_program_telemetry.id,
        g_motor.current_kp, g_motor.current_ki, PROGRAM_FAST_LOOP_DT_S,
        &g_motor.id_integral_v, -v_limit, v_limit);
    /** q 轴 PI：iq_ref → 跟踪 */
    g_motor.uq_ref = program_run_pi_f32(g_control.iq_ref_applied_a, g_program_telemetry.iq,
        g_motor.current_kp, g_motor.current_ki, PROGRAM_FAST_LOOP_DT_S,
        &g_motor.iq_integral_v, -v_limit, v_limit);
    /** 电压矢量限幅 */
    program_limit_voltage_vector(&g_motor.ud_ref, &g_motor.uq_ref, v_limit);
    /** 反 Park + SVPWM 输出 */
    foc_core_run_voltage_open_loop(&g_foc, g_motor.ud_ref, g_motor.uq_ref, theta_cmd, vbus);
}

/* 函数作用：电压开环模式（绕过电流环，直接输出 ud/uq）。
 * 输入：theta_cmd 控制电角度 (rad)。输出：无返回值。
 * 调用频率：快环每拍调用（电流环禁用时）。
 * 运行内容：电压矢量限幅 → 反 Park + SVPWM。 */
void program_run_voltage_mode(float theta_cmd)
{
    /** 母线电压 / 电压限幅值 */
    float vbus, v_limit;
    vbus = g_program_telemetry.vbus;
    v_limit = program_get_voltage_limit_v();
    g_motor.voltage_limit = v_limit;
    /** 电压矢量限幅 */
    program_limit_voltage_vector(&g_motor.ud_ref, &g_motor.uq_ref, v_limit);
    /** 反 Park + SVPWM 输出 */
    foc_core_run_voltage_open_loop(&g_foc, g_motor.ud_ref, g_motor.uq_ref, theta_cmd, vbus);
}
