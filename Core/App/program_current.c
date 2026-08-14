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

/* ── 来自 program.c 的外部变量 ── */
extern motor_state_t   g_motor;
extern foc_core_t      g_foc;
extern volatile program_telemetry_t      g_program_telemetry;
extern volatile program_debug_pwm_test_t g_program_debug_pwm_test;

extern volatile uint16_t g_adc2_dma_buf[];
extern volatile uint32_t g_ia_offset_sum;
extern volatile uint32_t g_ib_offset_sum;
extern volatile uint32_t g_ic_offset_sum;

extern uint8_t  g_encoder_speed_primed;
extern uint8_t  g_encoder_speed_ready;
extern uint8_t  g_speed_loop_update_pending;
extern float    g_encoder_speed_raw_mech_rad_s;

extern float g_id_ref_applied_a;
extern float g_iq_ref_applied_a;
extern filter_lpf_f32_t g_speed_meas_lpf;
extern filter_lpf_f32_t g_position_meas_lpf;
extern float g_speed_loop_dt_s;
extern float g_position_loop_elapsed_s;
extern float g_position_meas_output_continuous_rad;

extern uint8_t g_power_stage_enabled;
extern uint8_t g_position_hold_active;
extern uint8_t g_position_hold_release_counter;
extern uint8_t g_current_loop_enable_prev;
extern uint8_t g_position_loop_enable_prev;
extern uint8_t g_encoder_align_done;
extern uint32_t g_encoder_align_counter;
extern float    g_encoder_elec_offset_rad;

/* ── 电流换算 ── */

float program_convert_current_from_raw(uint16_t raw, uint16_t offset_raw)
{
    float volts_per_count = PROGRAM_ADC_REF_V / PROGRAM_ADC_FULL_SCALE_COUNTS;
    float sense_voltage = ((float)raw - (float)offset_raw) * volts_per_count;
    return sense_voltage / (PROGRAM_CURRENT_SENSE_GAIN * PROGRAM_SHUNT_RESISTOR_OHM);
}

float program_convert_vbus_from_raw(uint16_t raw)
{
    float volts_per_count = PROGRAM_ADC_REF_V / PROGRAM_ADC_FULL_SCALE_COUNTS;
    float adc_input_voltage = (float)raw * volts_per_count;
    return adc_input_voltage * ((PROGRAM_VBUS_R_UP_OHM + PROGRAM_VBUS_R_DOWN_OHM) / PROGRAM_VBUS_R_DOWN_OHM);
}

/* ── PI 控制器 ── */

float program_run_pi_f32(float ref, float feedback, float kp, float ki,
                         float dt_s, float *integral, float out_min, float out_max)
{
    float error, p_out, i_candidate, out;
    if ((integral == 0) || (!isfinite(ref)) || (!isfinite(feedback)) ||
        (!isfinite(kp)) || (!isfinite(ki)) || (!isfinite(dt_s)) ||
        (!isfinite(*integral)) || (!isfinite(out_min)) || (!isfinite(out_max)) ||
        (out_max < out_min)) {
        if (integral != 0) *integral = 0.0f;
        return 0.0f;
    }
    error = ref - feedback;
    p_out = kp * error;
    i_candidate = *integral + (ki * dt_s * error);
    i_candidate = program_clamp_f32(i_candidate, out_min, out_max);
    out = p_out + i_candidate;
    if (out > out_max) { out = out_max; if (error < 0.0f) *integral = i_candidate; }
    else if (out < out_min) { out = out_min; if (error > 0.0f) *integral = i_candidate; }
    else { *integral = i_candidate; }
    return out;
}

/* ── 电流环参数 ── */

float program_current_loop_kp_from_bandwidth_hz(float bandwidth_hz)
{
    if ((!isfinite(bandwidth_hz)) || (bandwidth_hz <= 0.0f)) return 0.0f;
    return MOTOR_TWO_PI * bandwidth_hz * 0.0004100f;
}

float program_current_loop_ki_from_bandwidth_hz(float bandwidth_hz)
{
    if ((!isfinite(bandwidth_hz)) || (bandwidth_hz <= 0.0f)) return 0.0f;
    return MOTOR_TWO_PI * bandwidth_hz * 0.7250f;
}

/* ── 电流清洗与斜坡 ── */

float program_sanitize_current_ref_cmd(float ref_cmd, float limit_abs_a)
{
    if ((!isfinite(ref_cmd)) || (!isfinite(limit_abs_a)) || (limit_abs_a <= 0.0f)) return 0.0f;
    return program_clamp_f32(ref_cmd, -limit_abs_a, limit_abs_a);
}

float program_apply_slew_limit_f32(float target, float state, float max_step)
{
    float delta;
    if ((!isfinite(target)) || (!isfinite(state)) || (!isfinite(max_step)) || (max_step <= 0.0f))
        return 0.0f;
    delta = target - state;
    delta = program_clamp_f32(delta, -max_step, max_step);
    return state + delta;
}

void program_update_applied_current_references(void)
{
    float current_limit_a = g_motor.iq_limit;
    float current_step_a = PROGRAM_CURRENT_REF_RAMP_A_PER_S * PROGRAM_FAST_LOOP_DT_S;
    float id_target_a = program_sanitize_current_ref_cmd(g_motor.id_ref, current_limit_a);
    float iq_target_a = program_sanitize_current_ref_cmd(g_motor.iq_ref, current_limit_a);
    g_id_ref_applied_a = program_apply_slew_limit_f32(id_target_a, g_id_ref_applied_a, current_step_a);
    g_iq_ref_applied_a = program_apply_slew_limit_f32(iq_target_a, g_iq_ref_applied_a, current_step_a);
}

/* ── 电压限制 ── */

float program_get_voltage_limit_v(void)
{
    float vbus = g_program_telemetry.vbus;
    return 0.57735026919f * ((vbus > 1.0f) ? vbus : 1.0f);
}

void program_limit_voltage_vector(float *ud_ref, float *uq_ref, float v_limit)
{
    float v_mag, v_scale;
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

void program_reset_current_reference_ramp(void)
    { g_id_ref_applied_a = 0.0f; g_iq_ref_applied_a = 0.0f; }

void program_reset_speed_loop(void)
{
    g_motor.speed_integral_iq = 0.0f;
    g_motor.speed_integral_uq = 0.0f;
    g_motor.iq_ref = 0.0f;
    if (g_motor.speed_loop_enable != 0U) g_motor.uq_ref = 0.0f;
    g_speed_loop_update_pending = 0U;
    g_speed_loop_dt_s = PROGRAM_FAST_LOOP_DT_S * 20.0f;
}

void program_reset_speed_reference_ramp(void)
{
    g_motor.speed_ref_mech_applied_rad_s = 0.0f;
    g_motor.speed_ref_elec_rad_s = 0.0f;
    g_motor.open_loop_speed_elec = 0.0f;
}

void program_reset_position_loop(void)
{
    g_motor.position_integral_speed = 0.0f;
    g_motor.position_error_mech_deg = 0.0f;
    g_motor.position_error_mech_rad = 0.0f;
    g_position_loop_elapsed_s = 0.0f;
    g_position_hold_active = 0U;
    g_position_hold_release_counter = 0U;
    g_position_meas_output_continuous_rad = 0.0f;
    filter_lpf_f32_init(&g_position_meas_lpf,
        program_lpf_alpha_from_cutoff_hz(12.0f, 0.005f), 0.0f);
    g_position_meas_lpf.initialized = 0U;
}

void program_reset_current_loop(void)
{
    g_motor.id_ref = 0.0f;  g_motor.iq_ref = 0.0f;
    g_motor.ud_ref = 0.0f;  g_motor.uq_ref = 0.0f;
    g_motor.id_integral_v = 0.0f;  g_motor.iq_integral_v = 0.0f;
    g_motor.voltage_limit = 0.0f;
    program_reset_current_reference_ramp();
}

/* ── 模式切换 ── */

void program_handle_position_loop_mode_switch(void)
{
    uint8_t position_loop_enable_now = (g_motor.position_loop_enable != 0U) ? 1U : 0U;
    if (position_loop_enable_now == g_position_loop_enable_prev) return;
    g_position_loop_enable_prev = position_loop_enable_now;
    program_reset_position_loop();
    program_reset_speed_loop();
    program_reset_speed_reference_ramp();
    if (position_loop_enable_now != 0U) {
        if (g_encoder_speed_primed != 0U) {
            g_position_meas_output_continuous_rad =
                program_get_encoder_output_continuous_mech_angle_rad();
            filter_lpf_f32_init(&g_position_meas_lpf,
                program_lpf_alpha_from_cutoff_hz(12.0f, 0.005f),
                g_position_meas_output_continuous_rad);
            g_motor.position_meas_mech_rad =
                program_wrap_angle_0_2pi(g_position_meas_output_continuous_rad);
            g_motor.position_meas_mech_deg =
                program_wrap_angle_0_360_deg(program_rad_to_deg(g_motor.position_meas_mech_rad));
            g_motor.position_ref_mech_deg = g_motor.position_meas_mech_deg;
            g_motor.position_ref_mech_rad = g_motor.position_meas_mech_rad;
        } else {
            g_motor.position_ref_mech_deg = 0.0f; g_motor.position_ref_mech_rad = 0.0f;
            g_motor.position_meas_mech_deg = 0.0f; g_motor.position_meas_mech_rad = 0.0f;
        }
    }
}

void program_handle_current_loop_mode_switch(void)
{
    uint8_t current_loop_enable_now = (g_motor.current_loop_enable != 0U) ? 1U : 0U;
    if (current_loop_enable_now == g_current_loop_enable_prev) return;
    g_current_loop_enable_prev = current_loop_enable_now;
    program_reset_speed_loop();
    program_reset_current_loop();
    g_motor.id_ref = 0.0f; g_motor.iq_ref = 0.0f;
    g_motor.ud_ref = 0.0f; g_motor.uq_ref = 0.0f;
}

/* ── 电流反馈 ── */

void program_update_current_feedback_from_raw(uint16_t ia_raw, uint16_t ib_raw,
                                              uint16_t ic_raw, float theta_elec)
{
    foc_alpha_beta_t i_ab;
    foc_dq_t i_dq;
    float ia_meas, ib_meas, ic_meas, sin_theta, cos_theta;

    if (g_program_telemetry.current_offset_ready == 0U) {
        g_program_telemetry.ia = 0.0f; g_program_telemetry.ib = 0.0f;
        g_program_telemetry.ic = 0.0f; g_program_telemetry.ic_meas = 0.0f;
        g_program_telemetry.i_abc_sum = 0.0f;
        g_program_telemetry.id = 0.0f; g_program_telemetry.iq = 0.0f;
        return;
    }

    ia_meas = PROGRAM_CURRENT_SIGN_IA * program_convert_current_from_raw(ia_raw, g_program_telemetry.ia_offset_raw);
    ib_meas = PROGRAM_CURRENT_SIGN_IB * program_convert_current_from_raw(ib_raw, g_program_telemetry.ib_offset_raw);
    ic_meas = PROGRAM_CURRENT_SIGN_IC * program_convert_current_from_raw(ic_raw, g_program_telemetry.ic_offset_raw);

    g_program_telemetry.ia = ia_meas;
    g_program_telemetry.ib = ib_meas;
    g_program_telemetry.ic_meas = ic_meas;
    g_program_telemetry.i_abc_sum = ia_meas + ib_meas + ic_meas;
    g_program_telemetry.ic = -(ia_meas + ib_meas);

    if (g_power_stage_enabled == 0U) {
        g_program_telemetry.id = 0.0f; g_program_telemetry.iq = 0.0f;
        g_foc.i_ab.alpha = 0.0f; g_foc.i_ab.beta = 0.0f;
        g_foc.i_dq.d = 0.0f; g_foc.i_dq.q = 0.0f;
        return;
    }

    foc_core_clarke(g_program_telemetry.ia, g_program_telemetry.ib, &i_ab);
    sin_theta = sinf(theta_elec);
    cos_theta = cosf(theta_elec);
    foc_core_park(&i_ab, sin_theta, cos_theta, &i_dq);

    g_foc.i_ab = i_ab;
    g_foc.i_dq = i_dq;
    g_program_telemetry.id = i_dq.d;
    g_program_telemetry.iq = i_dq.q;
}

/* ── 位置环 ── */

void program_update_position_loop(float position_loop_dt_s)
{
    float position_ref_wrapped_rad, position_meas_raw_output_continuous_rad;
    float position_meas_wrapped_rad, position_error_wrapped_rad;
    float position_speed_limit_mech_rad_s, position_speed_cmd_output_mech_rad_s;
    float position_speed_meas_output_mech_rad_s;
    float position_error_abs_rad, position_meas_filter_alpha, speed_meas_abs_rad_s;

    if ((g_motor.position_loop_enable == 0U) || (g_motor.speed_loop_enable == 0U)) {
        g_motor.position_error_mech_deg = 0.0f;
        g_motor.position_error_mech_rad = 0.0f;
        return;
    }
    if (g_encoder_speed_primed == 0U) {
        g_motor.position_integral_speed = 0.0f;
        g_motor.position_error_mech_deg = 0.0f;
        g_motor.position_error_mech_rad = 0.0f;
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }
    if ((!isfinite(position_loop_dt_s)) || (position_loop_dt_s <= 0.0f))
        position_loop_dt_s = 0.0001f * 20.0f;

    g_motor.position_ref_mech_deg = program_wrap_angle_0_360_deg(g_motor.position_ref_mech_deg);
    position_ref_wrapped_rad = program_deg_to_rad(g_motor.position_ref_mech_deg);
    position_meas_raw_output_continuous_rad = program_get_encoder_output_continuous_mech_angle_rad();
    if ((!isfinite(position_ref_wrapped_rad)) ||
        (!isfinite(position_meas_raw_output_continuous_rad))) {
        program_reset_position_loop();
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    position_meas_filter_alpha = program_lpf_alpha_from_cutoff_hz(12.0f, position_loop_dt_s);
    g_position_meas_lpf.alpha = position_meas_filter_alpha;
    if (g_position_meas_lpf.initialized == 0U)
        filter_lpf_f32_init(&g_position_meas_lpf, position_meas_filter_alpha,
                            position_meas_raw_output_continuous_rad);
    g_position_meas_output_continuous_rad =
        filter_lpf_f32_update(&g_position_meas_lpf, position_meas_raw_output_continuous_rad);
    if (!isfinite(g_position_meas_output_continuous_rad)) {
        program_reset_position_loop();
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    position_meas_wrapped_rad = program_wrap_angle_0_2pi(g_position_meas_output_continuous_rad);
    position_error_wrapped_rad = program_wrap_delta_pm_pi(
        position_ref_wrapped_rad - position_meas_wrapped_rad);
    if (!isfinite(position_error_wrapped_rad)) {
        program_reset_position_loop();
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

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
        g_position_hold_active = 1U;
        g_position_hold_release_counter = 0U;
        g_motor.position_integral_speed = 0.0f;
        g_motor.position_error_mech_deg = 0.0f;
        g_motor.position_error_mech_rad = 0.0f;
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }
    if (g_position_hold_active != 0U) {
        if (position_error_abs_rad <= PROGRAM_POSITION_HOLD_RELEASE_ERR_RAD) {
            g_position_hold_release_counter = 0U;
            g_motor.position_integral_speed = 0.0f;
            g_motor.position_error_mech_deg = 0.0f;
            g_motor.position_error_mech_rad = 0.0f;
            g_motor.speed_ref_mech_rad_s = 0.0f;
            return;
        }
        g_position_hold_release_counter++;
        if (g_position_hold_release_counter < PROGRAM_POSITION_HOLD_RELEASE_CONFIRM_CYCLES) {
            g_motor.position_integral_speed = 0.0f;
            g_motor.position_error_mech_deg = 0.0f;
            g_motor.position_error_mech_rad = 0.0f;
            g_motor.speed_ref_mech_rad_s = 0.0f;
            return;
        }
        g_position_hold_active = 0U;
        g_position_hold_release_counter = 0U;
    }

    position_speed_limit_mech_rad_s = g_motor.position_speed_limit_mech_rad_s;
    if ((!isfinite(position_speed_limit_mech_rad_s)) || (position_speed_limit_mech_rad_s <= 0.0f))
        position_speed_limit_mech_rad_s = 20.0f;

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
    g_motor.speed_ref_mech_rad_s = position_speed_cmd_output_mech_rad_s * MOTOR_GEAR_RATIO;
}

/* ── 速度环 ── */

void program_update_speed_loop(void)
{
    float output_cmd, position_loop_dt_s, speed_loop_dt_s, speed_output_limit;
    float *speed_integral;

    speed_loop_dt_s = g_speed_loop_dt_s;
    if ((!isfinite(speed_loop_dt_s)) || (speed_loop_dt_s <= 0.0f))
        speed_loop_dt_s = 0.0001f * 20.0f;

    if (g_motor.position_loop_enable == 0U) {
        program_reset_position_loop();
    } else if (g_speed_loop_update_pending != 0U) {
        g_position_loop_elapsed_s += speed_loop_dt_s;
        if (g_position_loop_elapsed_s >= 0.005f) {
            position_loop_dt_s = g_position_loop_elapsed_s;
            g_position_loop_elapsed_s = 0.0f;
            program_update_position_loop(position_loop_dt_s);
        }
    }

    program_update_speed_reference_ramp();

    if (g_encoder_speed_ready == 0U) {
        g_motor.iq_ref = 0.0f; g_motor.uq_ref = 0.0f;
        g_speed_loop_update_pending = 0U;
        if (g_motor.position_loop_enable != 0U) {
            g_motor.speed_ref_mech_rad_s = 0.0f;
            program_reset_position_loop();
        }
        return;
    }
    if (g_speed_loop_update_pending == 0U) return;
    g_speed_loop_update_pending = 0U;

    if (g_motor.current_loop_enable != 0U) {
        speed_integral = &g_motor.speed_integral_iq;
        speed_output_limit = g_motor.iq_limit;
    } else {
        speed_integral = &g_motor.speed_integral_uq;
        speed_output_limit = program_get_voltage_limit_v();
        g_motor.voltage_limit = speed_output_limit;
    }

    output_cmd = program_run_pi_f32(g_motor.speed_ref_elec_rad_s,
                                    g_motor.speed_meas_elec_rad_s,
                                    g_motor.speed_kp, g_motor.speed_ki,
                                    speed_loop_dt_s, speed_integral,
                                    -speed_output_limit, speed_output_limit);

    if (g_motor.current_loop_enable != 0U) {
        g_motor.iq_ref = output_cmd;
    } else {
        g_motor.ud_ref = 0.0f; g_motor.uq_ref = output_cmd; g_motor.iq_ref = 0.0f;
    }
}

/* ── 电流环 / 电压模式 ── */

void program_run_current_loop(float theta_cmd)
{
    float vbus, v_limit;
    program_update_applied_current_references();
    vbus = g_program_telemetry.vbus;
    v_limit = program_get_voltage_limit_v();
    g_motor.voltage_limit = v_limit;

    g_motor.ud_ref = program_run_pi_f32(g_id_ref_applied_a, g_program_telemetry.id,
        g_motor.current_kp, g_motor.current_ki, PROGRAM_FAST_LOOP_DT_S,
        &g_motor.id_integral_v, -v_limit, v_limit);
    g_motor.uq_ref = program_run_pi_f32(g_iq_ref_applied_a, g_program_telemetry.iq,
        g_motor.current_kp, g_motor.current_ki, PROGRAM_FAST_LOOP_DT_S,
        &g_motor.iq_integral_v, -v_limit, v_limit);
    program_limit_voltage_vector(&g_motor.ud_ref, &g_motor.uq_ref, v_limit);
    foc_core_run_voltage_open_loop(&g_foc, g_motor.ud_ref, g_motor.uq_ref, theta_cmd, vbus);
}

void program_run_voltage_mode(float theta_cmd)
{
    float vbus, v_limit;
    vbus = g_program_telemetry.vbus;
    v_limit = program_get_voltage_limit_v();
    g_motor.voltage_limit = v_limit;
    program_limit_voltage_vector(&g_motor.ud_ref, &g_motor.uq_ref, v_limit);
    foc_core_run_voltage_open_loop(&g_foc, g_motor.ud_ref, g_motor.uq_ref, theta_cmd, vbus);
}
