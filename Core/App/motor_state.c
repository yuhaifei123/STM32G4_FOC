/*
 * ========================================
 *  motor_state.c — 电机运行核心状态机
 *  调用: program.c 的 10kHz ADC ISR
 *  依赖: program_current.c / program_svpwm.c 的控制函数
 * ========================================
 */
#include "motor_state.h"
#include "motor_params.h"
#include "program_current.h"
#include "program_svpwm.h"

/* 对齐/开环参数宏统一在 program_config.h 中定义 */
/* 跨文件业务状态（g_encoder/g_control）extern 声明见 program.h */

/* 函数作用：执行一次状态切换并记录进入时间。
 * 输入：motor 为状态机对象，next_state 为目标状态，now_ms 为当前毫秒时间。
 * 输出：无返回值。
 * 调用频率：仅在发生状态切换时调用。
 * 运行内容：写入新的状态值，并记录状态进入时刻。 */
static void motor_state_enter(motor_state_t *motor, motor_state_id_t next_state, uint32_t now_ms)
{
    motor->state = next_state;
    motor->state_enter_ms = now_ms;
}

/* 函数作用：初始化电机状态机对象。
 * 输入：motor 为待初始化的状态机对象。
 * 输出：无返回值。
 * 调用频率：系统启动时调用一次。
 * 运行内容：设置通用初始状态、清零运行和故障变量，并初始化速度滤波器和 PI 占位对象。
 * 备注：当前项目的有效调参默认值会在 program_init() 中按板级实现统一覆写，这里保留的是结构体级安全初值。 */
void motor_state_init(motor_state_t *motor)
{
    if (motor == 0) {
        return;
    }

    motor->state = MOTOR_STATE_READY;
    motor->run_request = 0U;
    motor->speed_loop_enable = 1U;
    motor->current_loop_enable = 1U;
    motor->position_loop_enable = 0U;
    motor->control_angle_open_loop_enable = 0U;
    motor->align_done = 0U;
    motor->fault_code = MOTOR_FAULT_NONE;
    motor->state_enter_ms = 0U;
    motor->theta_open_loop = 0.0f;
    motor->open_loop_speed_elec = PROGRAM_OPEN_LOOP_DEFAULT_SPEED_ELEC;
    motor->control_angle_open_loop_speed_elec = PROGRAM_OPEN_LOOP_DEFAULT_SPEED_ELEC;
    motor->id_ref = 0.0f;
    motor->iq_ref = 0.0f;
    motor->ud_ref = 0.0f;
    motor->uq_ref = 0.0f;
    motor->speed_ref_mech_rpm = 0.0f;
    motor->speed_ref_mech_rad_s = 0.0f;
    motor->speed_ref_mech_applied_rad_s = 0.0f;
    motor->speed_meas_mech_rad_s = 0.0f;
    motor->speed_ref_elec_rad_s = 0.0f;
    motor->speed_meas_elec_rad_s = 0.0f;
    motor->speed_kp = 0.001f;
    motor->speed_ki = 0.004f;
    motor->position_ref_mech_deg = 0.0f;
    motor->position_ref_mech_rad = 0.0f;
    motor->position_meas_mech_deg = 0.0f;
    motor->position_meas_mech_rad = 0.0f;
    motor->position_error_mech_deg = 0.0f;
    motor->position_error_mech_rad = 0.0f;
    motor->position_kp = 8.0f;
    motor->position_ki = 0.0f;
    motor->position_kd = 0.0f;
    motor->position_integral_speed = 0.0f;
    motor->position_speed_limit_mech_rad_s = 20.0f;
    motor->speed_meas_lpf_cutoff_hz = 150.0f;
    motor->speed_integral_iq = 0.0f;
    motor->speed_integral_uq = 0.0f;
    motor->iq_limit = 2.0f;
    motor->current_kp = 0.5f;
    motor->current_ki = 200.0f;
    motor->id_integral_v = 0.0f;
    motor->iq_integral_v = 0.0f;
    motor->voltage_limit = 0.0f;

    filter_lpf_f32_init(&motor->speed_lpf, 0.1f, 0.0f);
    drv_pid_pi_init(&motor->speed_pi, 0, 0, -1000, 1000, 0);
}

/**
 * 电机运行核心状态机（10kHz 调用）
 *
 * 统一处理全部运行模式：
 *   READY → ALIGN(编码器对齐800ms+sin/cos平均) → 速度就绪后 CLOSED_LOOP(速度+电流级联)
 *   FAULT → 停机 → 清除后回 READY
 *
 * @param motor   电机状态机对象
 * @param foc     FOC 数学核心对象
 * @param now_ms  当前系统时间 (ms)，仅用于遥测参考，对齐用 tick 计数
 */
void motor_state_task(motor_state_t *motor, foc_core_t *foc, uint32_t now_ms)
{
    /** 当前控制电角度 (rad) */
    float theta_cmd;
    /** 对齐结束时 atan2 算出的原始电角度 */
    float raw_theta_elec;
    /** 驱动芯片故障标志（低电平有效） */
    uint8_t driver_fault_active;

    if ((motor == 0) || (foc == 0)) return;

    /* ── 故障检测 ── */
    driver_fault_active = program_is_driver_fault_active();
    g_program_telemetry.driver_fault_active = driver_fault_active;

    /* ── 调试 PWM 测试模式：直出固定占空比，绕过全部控制逻辑 ── */
    if (program_debug_pwm_test_is_enabled() != 0U) {
        if (driver_fault_active != 0U) {
            program_set_power_stage_enable(0U);
            foc_core_reset_output(foc);
            program_apply_svpwm_to_tim1(&foc->duty);
            motor_state_enter(motor, MOTOR_STATE_FAULT, now_ms);
        } else {
            program_apply_debug_pwm_test_output();
            motor->state = MOTOR_STATE_READY;
        }
        // 快照数据
        program_update_debug_telemetry();
        return;
    }

    /* 故障 → FAULT */
    if (driver_fault_active != 0U) {
        motor->fault_code = MOTOR_FAULT_DRIVER;
        motor_state_enter(motor, MOTOR_STATE_FAULT, now_ms);
    }

    /* 未就绪（零偏/编码器无效）→ 安全停机 */
    if ((g_program_telemetry.current_offset_ready == 0U) ||
        (g_program_telemetry.ma600a_angle_valid == 0U)) {
        program_set_power_stage_enable(0U);
        foc_core_reset_output(foc);
        foc_core_set_electrical_angle(foc, 0.0f);
        program_apply_svpwm_to_tim1(&foc->duty);
        program_reset_speed_loop();
        program_reset_position_loop();
        program_reset_current_loop();
        program_reset_speed_reference_ramp();
        program_reset_encoder_observer();
        program_reset_encoder_alignment();
        motor->theta_open_loop = 0.0f;
        motor->state = MOTOR_STATE_READY;
        program_update_debug_telemetry();
        return;
    }

    /* 模式切换处理 */
    program_handle_position_loop_mode_switch();
    program_handle_current_loop_mode_switch();

    /* ── 状态机主逻辑 ── */
    switch (motor->state) {

    /** INIT / READY：休眠等待启动命令 */
    case MOTOR_STATE_INIT:
    case MOTOR_STATE_READY:
        foc_core_reset_output(foc);
        program_apply_svpwm_to_tim1(&foc->duty);
        program_set_power_stage_enable(0U);
        if (motor->run_request != 0U) {
            /** 收到启动命令 → 复位对齐状态 → 进入 ALIGN */
            motor->align_done = 0U;
            g_encoder.align_counter = 0U;
            g_encoder.align_done  = 0U;
            g_encoder.elec_offset_rad = 0.0f;
            program_reset_speed_loop();
            program_reset_position_loop();
            program_reset_current_loop();
            program_reset_speed_reference_ramp();
            program_reset_encoder_observer();
            motor_state_enter(motor, MOTOR_STATE_ALIGN, now_ms);
        }
        break;

    case MOTOR_STATE_ALIGN:
        /** Ud=1.8V 直流锁定转子 800ms，最后 512 拍 sin/cos 平均求电角偏置 */
        motor->id_ref    = 0.0f;
        motor->iq_ref    = 0.0f;
        motor->theta_open_loop = 0.0f;        /** 电角度 = 0（d 轴固定） */
        motor->ud_ref    = PROGRAM_ALIGN_UD_V;   /** 1.8V 直流电压 */
        motor->uq_ref    = 0.0f;
        /** 计算 SVPWM 占空比 → 下发 TIM1 → 使能功率级 */
        foc_core_run_voltage_open_loop(foc, motor->ud_ref, motor->uq_ref,
                                       motor->theta_open_loop, g_program_telemetry.vbus);
        program_apply_svpwm_to_tim1(&foc->duty);
        program_set_power_stage_enable(1U);
        g_encoder.align_counter++;
        /** 在最后 512 拍的窗口内累加 sin/cos */
        program_capture_encoder_alignment_sample();

        if (g_encoder.align_counter >= PROGRAM_ALIGN_HOLD_TICKS) {
            /** 800ms 到 → atan2(sin_sum, cos_sum) 求平均电角 → 写入偏置 */
            raw_theta_elec = program_get_encoder_alignment_angle_rad();
            g_encoder.elec_offset_rad = motor_params_wrap_angle_rad(
                raw_theta_elec - motor->theta_open_loop);
            g_encoder.align_done = 1U;
            motor->align_done   = 1U;
            /** 对齐完成 → 复位观测器 → 切闭环 */
            program_reset_encoder_align_runtime();
            program_reset_speed_loop();
            program_reset_position_loop();
            program_reset_current_loop();
            program_reset_encoder_observer();
            motor_state_enter(motor, MOTOR_STATE_CLOSED_LOOP, now_ms);
        }
        break;

    /** CLOSED_LOOP：速度环 + 电流环级联控制 */
    case MOTOR_STATE_CLOSED_LOOP:
        /** 停机请求 → 关功率级 → 回 READY */
        if (motor->run_request == 0U) {
            program_set_power_stage_enable(0U);
            foc_core_reset_output(foc);
            foc_core_set_electrical_angle(foc, 0.0f);
            program_apply_svpwm_to_tim1(&foc->duty);
            program_reset_speed_loop();
            program_reset_position_loop();
            program_reset_current_loop();
            program_reset_speed_reference_ramp();
            program_reset_encoder_alignment();
            motor_state_enter(motor, MOTOR_STATE_READY, now_ms);
            break;
        }

        /** 速度观测未就绪 → 电压模式等待（仅电流环或直接电压输出） */
        if ((motor->speed_loop_enable != 0U) && (g_encoder.speed_ready == 0U)) {
            theta_cmd = program_get_control_elec_angle_rad();
            motor->theta_open_loop = theta_cmd;
            program_reset_speed_loop();
            program_reset_position_loop();
            program_reset_speed_reference_ramp();
            motor->id_ref = 0.0f;
            motor->iq_ref = 0.0f;
            motor->ud_ref = 0.0f;
            motor->uq_ref = 0.0f;
            if (motor->current_loop_enable != 0U)
                program_run_current_loop(theta_cmd);  /** 电流环（id=0 控制） */
            else
                program_run_voltage_mode(theta_cmd);   /** 纯电压模式 */
            program_apply_svpwm_to_tim1(&foc->duty);
            program_set_power_stage_enable(1U);
            break;
        }

        /** 正常闭环：速度环 PI → iq_ref → 电流环 PI → ud/uq → SVPWM */
        motor->id_ref = 0.0f;
        if (motor->speed_loop_enable != 0U) {
            /** 速度环：目标转速 vs 实际转速 → iq_ref */
            program_update_speed_loop();
        } else {
            /** 速度环禁用：清零积分、复位位置环 */
            motor->speed_integral_iq = 0.0f;
            motor->speed_integral_uq = 0.0f;
            program_reset_position_loop();
            g_encoder.speed_loop_update_pending = 0U;
            g_control.speed_loop_dt_s = PROGRAM_FAST_LOOP_DT_S * 20.0f;
            program_reset_speed_reference_ramp();
        }
        /** 获取控制电角度（编码器 + 偏置，或开环积分器） */
        theta_cmd = program_get_control_elec_angle_rad();
        motor->theta_open_loop = theta_cmd;
        /** 电流环：id_ref=0, iq_ref=速度环输出 → ud/uq → SVPWM */
        if (motor->current_loop_enable != 0U)
            program_run_current_loop(theta_cmd);
        else
            program_run_voltage_mode(theta_cmd);
        program_apply_svpwm_to_tim1(&foc->duty);
        program_set_power_stage_enable(1U);
        break;

    /** FAULT：故障停机，等待清除 */
    case MOTOR_STATE_FAULT:
        foc_core_reset_output(foc);
        program_apply_svpwm_to_tim1(&foc->duty);
        program_set_power_stage_enable(0U);
        motor->theta_open_loop = 0.0f;
        /** 停机且故障码已清除 → 恢复 READY */
        if ((motor->run_request == 0U) && (motor->fault_code == MOTOR_FAULT_NONE))
            motor_state_enter(motor, MOTOR_STATE_READY, now_ms);
        break;

    default:
        motor_state_enter(motor, MOTOR_STATE_READY, now_ms);
        break;
    }

    /** 每拍快照遥测（供 VOFA / Watch 窗口观察） */
    program_update_debug_telemetry();
}

/* 函数作用：设置运行请求标志。
 * 输入：motor 为状态机对象，enable 为 0 或 1。
 * 输出：无返回值。
 * 调用频率：外部控制逻辑按需调用。
 * 运行内容：只更新运行请求，真正的状态切换在 motor_state_task() 里完成。 */
void motor_state_set_run_request(motor_state_t *motor, uint8_t enable)
{
    if (motor == 0) {
        return;
    }

    motor->run_request = enable;
}
