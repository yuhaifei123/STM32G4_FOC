/*
 * ========================================
 *  program_svpwm.c — 编码器 / 斜坡 / 工具层
 *  负责: 速度测量、编码器角度获取与对齐、
 *        角度换算、斜坡生成、量化保护
 *  被调: program.c (HAL回调链)
 *        program_current.c (控制环)
 * ========================================
 */
#include "program_svpwm.h"
#include "program_current.h"

#include <math.h>

#include "filter.h"
#include "foc_core.h"
#include "ma600a.h"
#include "motor_params.h"

/* ── 来自 program.c 的外部变量 ── */
extern ma600a_t        g_ma600a;
extern motor_state_t   g_motor;
extern foc_core_t      g_foc;
extern volatile program_telemetry_t      g_program_telemetry;
extern volatile program_debug_pwm_test_t g_program_debug_pwm_test;

extern volatile uint16_t g_adc2_dma_buf[];
extern volatile uint32_t g_tim6_tick_ms;
extern volatile uint32_t g_ia_offset_sum;
extern volatile uint32_t g_ib_offset_sum;
extern volatile uint32_t g_ic_offset_sum;

extern uint32_t g_encoder_last_sample_counter;
extern uint8_t  g_encoder_speed_primed;
extern uint8_t  g_encoder_speed_ready;
extern uint8_t  g_speed_loop_update_pending;
extern float    g_encoder_prev_mech_angle_rad;
extern float    g_encoder_continuous_mech_rad;
extern float    g_encoder_speed_window_start_mech_rad;
extern uint32_t g_encoder_speed_window_sample_count;
extern float    g_encoder_speed_raw_mech_rad_s;
extern uint8_t  g_encoder_align_done;
extern uint32_t g_encoder_align_counter;
extern float    g_encoder_elec_offset_rad;
extern float    g_encoder_align_sum_sin;
extern float    g_encoder_align_sum_cos;
extern uint32_t g_encoder_align_sample_count;

extern float g_id_ref_applied_a;
extern float g_iq_ref_applied_a;
extern filter_lpf_f32_t g_speed_meas_lpf;
extern filter_lpf_f32_t g_position_meas_lpf;
extern float g_speed_loop_dt_s;
extern float g_position_meas_output_continuous_rad;

extern uint8_t g_power_stage_enabled;
extern uint8_t g_position_hold_active;
extern uint8_t g_position_hold_release_counter;

/* 函数作用：限制浮点量上下界。 */
float program_clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/* 函数作用：由截止频率和采样周期换算一阶低通滤波系数。 */
float program_lpf_alpha_from_cutoff_hz(float cutoff_hz, float dt_s)
{
    float alpha;
    if ((!isfinite(cutoff_hz)) || (!isfinite(dt_s)) || (cutoff_hz <= 0.0f) || (dt_s <= 0.0f))
        return 1.0f;
    alpha = 1.0f - expf(-MOTOR_TWO_PI * cutoff_hz * dt_s);
    return program_clamp_f32(alpha, 0.0f, 1.0f);
}

/* 函数作用：把角度归一化到 0~2pi。 */
float program_wrap_angle_0_2pi(float angle_rad)
{
    float turns;
    if (!isfinite(angle_rad)) return 0.0f;
    turns = floorf(angle_rad / MOTOR_TWO_PI);
    angle_rad -= turns * MOTOR_TWO_PI;
    if (angle_rad < 0.0f) angle_rad += MOTOR_TWO_PI;
    else if (angle_rad >= MOTOR_TWO_PI) angle_rad -= MOTOR_TWO_PI;
    return angle_rad;
}

/* 函数作用：把角度差归一化到 -pi~pi。 */
float program_wrap_delta_pm_pi(float angle_rad)
{
    if (!isfinite(angle_rad)) return 0.0f;
    return program_wrap_angle_0_2pi(angle_rad + PROGRAM_PI) - PROGRAM_PI;
}

/* 函数作用：把角度归一化到 0~360°。 */
float program_wrap_angle_0_360_deg(float angle_deg)
{
    float turns;
    if (!isfinite(angle_deg)) return 0.0f;
    turns = floorf(angle_deg / 360.0f);
    angle_deg -= turns * 360.0f;
    if (angle_deg < 0.0f) angle_deg += 360.0f;
    else if (angle_deg >= 360.0f) angle_deg -= 360.0f;
    return angle_deg;
}

/* 函数作用：rad/s → rpm */
float program_rad_s_to_rpm(float speed_rad_s)
    { return speed_rad_s * (60.0f / MOTOR_TWO_PI); }

/* 函数作用：rad → deg */
float program_rad_to_deg(float angle_rad)
    { if (!isfinite(angle_rad)) return 0.0f; return angle_rad * (360.0f / MOTOR_TWO_PI); }

/* 函数作用：deg → rad */
float program_deg_to_rad(float angle_deg)
    { if (!isfinite(angle_deg)) return 0.0f; return angle_deg * (MOTOR_TWO_PI / 360.0f); }

/* 函数作用：rpm → rad/s */
float program_rpm_to_rad_s(float speed_rpm)
    { if (!isfinite(speed_rpm)) return 0.0f; return speed_rpm * (MOTOR_TWO_PI / 60.0f); }

/* 函数作用：估算编码器测速量化分辨率。 */
float program_get_speed_quantization_rad_s(uint32_t observer_window_samples)
{
    if (observer_window_samples == 0U) return 0.0f;
    return (MOTOR_TWO_PI / 65536.0f) / (PROGRAM_FAST_LOOP_DT_S * (float)observer_window_samples);
}

/* 函数作用：在低速附近抑制编码器量化抖动。 */
float program_apply_speed_quantization_guard(float speed_mech_rad_s,
                                             uint32_t observer_window_samples)
{
    float zero_hold_threshold_rad_s;
    if (!isfinite(speed_mech_rad_s)) return 0.0f;
    zero_hold_threshold_rad_s = 8.0f * program_get_speed_quantization_rad_s(observer_window_samples);
    if ((!isfinite(zero_hold_threshold_rad_s)) || (zero_hold_threshold_rad_s <= 0.0f))
        return speed_mech_rad_s;
    if (zero_hold_threshold_rad_s < 0.35f) zero_hold_threshold_rad_s = 0.35f;
    if ((fabsf(g_motor.speed_ref_mech_applied_rad_s) <= zero_hold_threshold_rad_s) &&
        (fabsf(speed_mech_rad_s) <= zero_hold_threshold_rad_s))
        return 0.0f;
    return speed_mech_rad_s;
}

/* ── 编码器角度获取 ── */

/* 函数作用：读取编码器对应的转子机械角。 */
float program_get_encoder_rotor_mech_angle_rad(void)
{
    return g_ma600a.angle_rad * MOTOR_ENCODER_DIRECTION_SIGN;
}

/* 函数作用：获取输出轴连续机械角。 */
float program_get_encoder_output_continuous_mech_angle_rad(void)
{
    float rotor_mech_angle_rad;
    if (g_encoder_speed_primed != 0U)
        rotor_mech_angle_rad = g_encoder_continuous_mech_rad;
    else
        rotor_mech_angle_rad = program_get_encoder_rotor_mech_angle_rad();
    if ((!isfinite(rotor_mech_angle_rad)) || (MOTOR_GEAR_RATIO <= 0.0f)) return 0.0f;
    return rotor_mech_angle_rad / MOTOR_GEAR_RATIO;
}

/* 函数作用：获取输出轴单圈机械角。 */
float program_get_encoder_output_mech_angle_rad(void)
{
    return program_wrap_angle_0_2pi(program_get_encoder_output_continuous_mech_angle_rad());
}

/* 函数作用：获取原始电角度。 */
float program_get_encoder_raw_elec_angle_rad(void)
{
    return program_wrap_angle_0_2pi(program_get_encoder_rotor_mech_angle_rad() * MOTOR_POLE_PAIRS);
}

/* 函数作用：获取零位补偿后电角度。 */
float program_get_encoder_aligned_elec_angle_rad(void)
{
    return program_wrap_angle_0_2pi(program_get_encoder_raw_elec_angle_rad() - g_encoder_elec_offset_rad);
}

/* 函数作用：提供当前控制电角度。 */
float program_get_control_elec_angle_rad(void)
{
    if (g_motor.control_angle_open_loop_enable != 0U)
        return program_wrap_angle_0_2pi(g_motor.theta_open_loop);
    return program_get_encoder_aligned_elec_angle_rad();
}

/* 函数作用：更新控制角度开环状态。 */
void program_update_control_angle_open_loop_state(void)
{
    if ((g_motor.control_angle_open_loop_enable == 0U) || (g_encoder_align_done == 0U)) return;
    g_motor.theta_open_loop = program_wrap_angle_0_2pi(
        g_motor.theta_open_loop + g_motor.control_angle_open_loop_speed_elec * PROGRAM_FAST_LOOP_DT_S);
}

/* ── 编码器观测器 ── */

/* 函数作用：复位编码器测速观测器。 */
void program_reset_encoder_observer(void)
{
    g_encoder_last_sample_counter = 0U;
    g_encoder_speed_primed = 0U;
    g_encoder_speed_ready = 0U;
    g_encoder_prev_mech_angle_rad = 0.0f;
    g_encoder_continuous_mech_rad = 0.0f;
    g_encoder_speed_window_start_mech_rad = 0.0f;
    g_encoder_speed_window_sample_count = 0U;
    g_encoder_speed_raw_mech_rad_s = 0.0f;
    g_motor.speed_meas_mech_rad_s = 0.0f;
    g_motor.speed_meas_elec_rad_s = 0.0f;
    g_motor.position_meas_mech_deg = 0.0f;
    g_motor.position_meas_mech_rad = 0.0f;
    g_speed_loop_update_pending = 0U;
    g_speed_loop_dt_s = PROGRAM_FAST_LOOP_DT_S * 20.0f;
    g_position_meas_output_continuous_rad = 0.0f;
    filter_lpf_f32_init(&g_position_meas_lpf,
        program_lpf_alpha_from_cutoff_hz(12.0f, 0.005f), 0.0f);
    g_position_meas_lpf.initialized = 0U;
    filter_lpf_f32_init(&g_speed_meas_lpf,
        program_lpf_alpha_from_cutoff_hz(g_motor.speed_meas_lpf_cutoff_hz, 0.002f), 0.0f);
}

/* 函数作用：对连续机械角做周期性重归一化。 */
static void program_renormalize_encoder_observer(void)
{
    float anchor_turns, anchor_rad;
    if ((!isfinite(g_encoder_continuous_mech_rad)) ||
        (!isfinite(g_encoder_speed_window_start_mech_rad)) ||
        (!isfinite(g_encoder_speed_raw_mech_rad_s))) {
        program_reset_encoder_observer();
        return;
    }
    if ((fabsf(g_encoder_continuous_mech_rad) < (32.0f * MOTOR_TWO_PI)) &&
        (fabsf(g_encoder_speed_window_start_mech_rad) < (32.0f * MOTOR_TWO_PI)))
        return;
    anchor_turns = floorf(g_encoder_continuous_mech_rad / MOTOR_TWO_PI);
    anchor_rad = anchor_turns * MOTOR_TWO_PI;
    g_encoder_continuous_mech_rad -= anchor_rad;
    g_encoder_speed_window_start_mech_rad -= anchor_rad;
}

/* 函数作用：基于编码器角度更新机械速度测量。 */
void program_update_speed_measurement(void)
{
    /** 当前拍机械角 (rad)，归一化到 [0, 2π) */
    float mech_angle_rad;
    /** 本拍角差 (rad)，处理 0°/360° 跳变 */
    float mech_delta_rad;
    /** 观测窗口时间跨度 (s)，窗口满时用于计算速度 */
    float observer_dt_s;
    /** 观测窗口内的拍数（可能因丢样本而 >20） */
    uint32_t observer_window_samples;
    /** 两次有效样本间的计数器增量（>1 表示有丢样本） */
    uint32_t sample_delta_count;

    /** 编码器数据无效 → 跳过本拍 */
    if (g_ma600a.data_valid == 0U) return;
    /** 无新样本到达（counter 未变）→ 跳过 */
    if (g_ma600a.sample_counter == g_encoder_last_sample_counter) return;

    /** 获取当前拍转子机械角 (rad)，归一化到 [0, 2π) */
    mech_angle_rad = program_wrap_angle_0_2pi(program_get_encoder_rotor_mech_angle_rad());
    /** 两次有效样本间的计数器增量（>1 表示有丢样本） */
    sample_delta_count = g_ma600a.sample_counter - g_encoder_last_sample_counter;
    if (sample_delta_count == 0U) return;
    /** 更新最新样本计数器 */
    g_encoder_last_sample_counter = g_ma600a.sample_counter;

    /* ══ 首次采样：初始化观测器基准值 ══ */
    if (g_encoder_speed_primed == 0U) {
        g_encoder_prev_mech_angle_rad = mech_angle_rad;          /** 记录初始角度 */
        g_encoder_continuous_mech_rad = mech_angle_rad;          /** 连续角起点 */
        g_motor.position_meas_mech_rad = program_get_encoder_output_mech_angle_rad();
        g_encoder_speed_window_start_mech_rad = mech_angle_rad;  /** 窗口起点 */
        g_encoder_speed_window_sample_count = 0U;                /** 窗口计数清零 */
        g_encoder_speed_raw_mech_rad_s = 0.0f;
        g_encoder_speed_primed = 1U;                             /** 首次角度已记录 */
        g_encoder_speed_ready = 0U;                              /** 速度尚未就绪 */
        g_motor.speed_meas_mech_rad_s = 0.0f;
        g_motor.speed_meas_elec_rad_s = 0.0f;
        return;
    }

    /* ══ 正常累加：计算本拍角差 → 累加到连续角 → 窗口计数递增 ══ */
    /** 本拍角差（处理 0°/360° 跳变） */
    mech_delta_rad = program_wrap_delta_pm_pi(mech_angle_rad - g_encoder_prev_mech_angle_rad);
    if (!isfinite(mech_delta_rad)) { program_reset_encoder_observer(); return; }
    g_encoder_prev_mech_angle_rad = mech_angle_rad;
    g_encoder_continuous_mech_rad += mech_delta_rad;             /** 连续角累加 */
    g_encoder_speed_window_sample_count += sample_delta_count;   /** 窗口计数累加 */
    /** 防止连续角无限增长导致浮点精度丢失 */
    program_renormalize_encoder_observer();
    /** 同步更新位置测量值 */
    g_motor.position_meas_mech_rad = program_get_encoder_output_mech_angle_rad();
    g_motor.position_meas_mech_deg = program_wrap_angle_0_360_deg(
        program_rad_to_deg(g_motor.position_meas_mech_rad));

    /** 窗口未满 20 拍 → 继续积累，不计算速度 */
    if (g_encoder_speed_window_sample_count < 20U) return;

    /* ══ 窗口满：差分计算原始速度 ══ */
    observer_window_samples = g_encoder_speed_window_sample_count;
    /** 窗口时间跨度 = 快环周期 × 拍数 */
    observer_dt_s = PROGRAM_FAST_LOOP_DT_S * (float)observer_window_samples;
    if ((!isfinite(observer_dt_s)) || (observer_dt_s <= 0.0f))
        { program_reset_encoder_observer(); return; }

    /** 速度 = (终点角度 - 起点角度) / 时间 */
    g_encoder_speed_raw_mech_rad_s =
        (g_encoder_continuous_mech_rad - g_encoder_speed_window_start_mech_rad) / observer_dt_s;
    if (!isfinite(g_encoder_speed_raw_mech_rad_s)) { program_reset_encoder_observer(); return; }

    /** 重置窗口：起点拉到当前连续角，计数清零，开启下一窗口 */
    g_encoder_speed_window_start_mech_rad = g_encoder_continuous_mech_rad;
    g_encoder_speed_window_sample_count = 0U;
    g_speed_loop_dt_s = observer_dt_s;                           /** 速度环实际调用周期 */
    /** 更新 LPF 系数，匹配当前的观测周期 */
    g_speed_meas_lpf.alpha = program_lpf_alpha_from_cutoff_hz(
        g_motor.speed_meas_lpf_cutoff_hz, observer_dt_s);

    /* ══ 首个窗口：初始化 LPF → 速度测量就绪 ══ */
    if (g_encoder_speed_ready == 0U) {
        filter_lpf_f32_init(&g_speed_meas_lpf, g_speed_meas_lpf.alpha,
                            g_encoder_speed_raw_mech_rad_s);
        g_encoder_speed_ready = 1U;                              /** 速度测量就绪，闭环可启动 */
    }

    /** LPF 滤波原始速度 */
    g_motor.speed_meas_mech_rad_s = filter_lpf_f32_update(&g_speed_meas_lpf,
        g_encoder_speed_raw_mech_rad_s);
    if (!isfinite(g_motor.speed_meas_mech_rad_s)) { program_reset_encoder_observer(); return; }

    /** 低速量化保护：接近零速时强制归零，消除编码器量化噪声 */
    g_motor.speed_meas_mech_rad_s = program_apply_speed_quantization_guard(
        g_motor.speed_meas_mech_rad_s, observer_window_samples);
    /** 机械角速度 → 电角速度（× 极对数） */
    g_motor.speed_meas_elec_rad_s = g_motor.speed_meas_mech_rad_s * MOTOR_POLE_PAIRS;
    /** 通知速度环：本次窗口已产出新速度，可执行一次速度 PI */
    g_speed_loop_update_pending = 1U;
}

/* ── 编码器零位对齐 ── */

void program_reset_encoder_alignment(void)
{
    g_encoder_align_done = 0U;
    g_encoder_align_counter = 0U;
    g_encoder_elec_offset_rad = 0.0f;
    g_motor.align_done = 0U;
}

void program_reset_encoder_align_runtime(void)
{
    g_encoder_align_counter = 0U;
    g_encoder_align_sum_sin = 0.0f;
    g_encoder_align_sum_cos = 0.0f;
    g_encoder_align_sample_count = 0U;
}

void program_capture_encoder_alignment_sample(void)
{
    uint32_t sample_window_start_tick;
    float raw_theta_elec;
    sample_window_start_tick = 0U;
    if (8000U > 512U) sample_window_start_tick = 8000U - 512U;
    if (g_encoder_align_counter < sample_window_start_tick) return;
    raw_theta_elec = program_get_encoder_raw_elec_angle_rad();
    g_encoder_align_sum_sin += sinf(raw_theta_elec);
    g_encoder_align_sum_cos += cosf(raw_theta_elec);
    g_encoder_align_sample_count++;
}

float program_get_encoder_alignment_angle_rad(void)
{
    float raw_theta_elec;
    if (g_encoder_align_sample_count == 0U)
        return program_get_encoder_raw_elec_angle_rad();
    raw_theta_elec = atan2f(g_encoder_align_sum_sin, g_encoder_align_sum_cos);
    return program_wrap_angle_0_2pi(raw_theta_elec);
}

/* ── 速度斜坡 ── */

void program_update_speed_reference_ramp(void)
{
    float speed_step_rad_s, speed_delta_rad_s;
    if (g_motor.position_loop_enable == 0U)
        g_motor.speed_ref_mech_rad_s = program_rpm_to_rad_s(g_motor.speed_ref_mech_rpm);
    speed_step_rad_s = PROGRAM_SPEED_REF_RAMP_RAD_S2 * PROGRAM_FAST_LOOP_DT_S;
    speed_delta_rad_s = g_motor.speed_ref_mech_rad_s - g_motor.speed_ref_mech_applied_rad_s;
    speed_delta_rad_s = program_clamp_f32(speed_delta_rad_s, -speed_step_rad_s, speed_step_rad_s);
    g_motor.speed_ref_mech_applied_rad_s += speed_delta_rad_s;
    g_motor.speed_ref_elec_rad_s = g_motor.speed_ref_mech_applied_rad_s * MOTOR_POLE_PAIRS;
    g_motor.open_loop_speed_elec = g_motor.speed_ref_elec_rad_s;
}
