#include "program.h"

#include <math.h>
#include <string.h>

#include "cli_uart.h"
#include "filter.h"
#include "gpio.h"
#include "ma600a.h"
#include "motor_params.h"
#include "program_current.h"
#include "program_svpwm.h"

/* 配置宏统一在 program_config.h 中定义（通过 program.h 引入） */

/* ── 全局对象 ── */
/** 电机状态机对象，管理运行状态和所有控制参数 */
motor_state_t g_motor;
/** FOC 数学核心对象，缓存 sin/cos、dq/αβ 变量和 SVPWM 占空比 */
foc_core_t g_foc;
/** MA600A 磁编码器对象，存储角度和通信状态 */
ma600a_t g_ma600a;
/** 程序层遥测对象，汇总全部调试变量供 Watch 窗口观察 */
volatile program_telemetry_t g_program_telemetry;
/** debug PWM 测试参数，调试器中设 enable=1 直接输出固定占空比 */
volatile program_debug_pwm_test_t g_program_debug_pwm_test;

/* ── 业务状态实例（类型定义见 program.h） ── */
/** 编码器跨文件接口状态实例 */
program_encoder_t g_encoder;
/** 控制环运行时状态实例 */
program_control_t g_control = { .speed_loop_dt_s = PROGRAM_SPEED_LOOP_DT_S, .current_loop_enable_prev = 1U };

/* ── 后台调度与性能统计状态（仅本文件，static 封装） ── */
typedef struct {
    filter_lpf_f32_t vbus_lpf;                              /* VBUS 电压一阶低通滤波器 */
    volatile uint16_t adc2_dma_buf[PROGRAM_ADC2_DMA_LENGTH]; /* ADC2 DMA 循环缓冲 [0]=VBUS, [1]=NTC */
    volatile uint32_t tim6_tick_ms;                         /* TIM6 1ms 时基计数器，后台任务调度节拍 */
    volatile uint32_t ia_offset_sum;                        /* A 相零偏累加和（中断中累加，校准完成后取均值） */
    volatile uint32_t ib_offset_sum;                        /* B 相零偏累加和 */
    volatile uint32_t ic_offset_sum;                        /* C 相零偏累加和 */
    uint32_t last_slow_task_tick_ms;                        /* 上一次慢任务执行时刻 (ms) */
    uint32_t last_wave_tick_ms;                             /* 上一次 VOFA 波形发送时刻 (ms) */
    uint8_t  tim1_pwm_started;                              /* TIM1 PWM 已启动标志（防止重复启动） */
    uint8_t  dwt_cycle_counter_ready;                       /* DWT 周期计数器就绪标志 */
    uint32_t fast_loop_period_cycles;                       /* 快环对应的 CPU 周期数 = SystemCoreClock / 10000 */
} program_system_state_t;

/** 后台调度与性能统计状态静态实例 */
static program_system_state_t g_sys = { .fast_loop_period_cycles = 1U };

/* ── UART 接收状态（仅本文件，static 封装） ── */
typedef struct {
    char    rx_buf[PROGRAM_UART_RX_BUF_SIZE]; /* RX 接收缓冲区 */
    uint8_t rx_idx;                           /* RX 当前写入位置 */
    uint8_t rx_done;                          /* RX 完整行就绪标志（收到 \n 后置位，解析后清除） */
    char    rx_char;                          /* RX 单字符接收缓存（中断中写入） */
} program_uart_rx_t;

/** UART 接收状态静态实例 */
static program_uart_rx_t g_uart_rx;

/* ── 调试 PWM 测试 ── */
/* 函数作用：查询调试 PWM 测试模式是否使能。
 * 输入：无。输出：返回 1=使能。调用频率：快环入口调用。 */
uint8_t program_debug_pwm_test_is_enabled(void)
    { return (g_program_debug_pwm_test.enable != 0U) ? 1U : 0U; }

/* 函数作用：直接用固定占空比驱动三相 PWM，用于硬件调试。
 * 输入：无。输出：无返回值。调用频率：仅调试 PWM 测试模式下调用。 */
void program_apply_debug_pwm_test_output(void)
{
    /** 限幅后的三相调试占空比 */
    foc_svpwm_duty_t debug_duty;
    debug_duty.duty_a = program_clamp_f32(g_program_debug_pwm_test.duty_a, 0.0f, 1.0f);
    debug_duty.duty_b = program_clamp_f32(g_program_debug_pwm_test.duty_b, 0.0f, 1.0f);
    debug_duty.duty_c = program_clamp_f32(g_program_debug_pwm_test.duty_c, 0.0f, 1.0f);
    g_foc.duty = debug_duty;
    program_apply_svpwm_to_tim1(&debug_duty);
    program_set_power_stage_enable(1U);
}

/* ── SVPWM 输出 ── */

/* 函数作用：将 SVPWM 占空比写入 TIM1 比较寄存器。
 * 输入：duty 为三相占空比结构体指针。输出：无返回值。
 * 调用频率：每次 FOC 控制完成后调用。
 * 运行内容：占空比→计数值→钳位保护→__HAL_TIM_SET_COMPARE 写入三通道 CCR。 */
void program_apply_svpwm_to_tim1(const foc_svpwm_duty_t *duty)
{
    /** PWM 计数周期 = ARR + 1 */
    uint32_t period_counts;
    /** 三相比较寄存器值 */
    uint32_t ccr_a, ccr_b, ccr_c;
    if (duty == 0) return;
    period_counts = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    ccr_a = (uint32_t)(program_clamp_f32(duty->duty_a, 0.0f, 1.0f) * (float)period_counts);
    ccr_b = (uint32_t)(program_clamp_f32(duty->duty_b, 0.0f, 1.0f) * (float)period_counts);
    ccr_c = (uint32_t)(program_clamp_f32(duty->duty_c, 0.0f, 1.0f) * (float)period_counts);
    if (ccr_a >= period_counts) ccr_a = period_counts - 1U;
    if (ccr_b >= period_counts) ccr_b = period_counts - 1U;
    if (ccr_c >= period_counts) ccr_c = period_counts - 1U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

/* 函数作用：擦除 FOC 输出，三相占空比拉回 50% 零矢量。
 * 输入：无。输出：无返回值。
 * 调用频率：安全停机或复位时调用。
 * 运行内容：赋 50% 占空比并通过 TIM1 下发，不改变 PWM 输出状态。 */
static void program_apply_center_duty(void)
{
    foc_svpwm_duty_t center_duty = { .duty_a = 0.5f, .duty_b = 0.5f, .duty_c = 0.5f };
    program_apply_svpwm_to_tim1(&center_duty);
}

/* 函数作用：首次启动 TIM1 六路互补 PWM 输出并拉零矢量。
 * 输入：无。输出：无返回值；任一路启动失败则进入 Error_Handler()。
 * 调用频率：系统初始化时调用一次。
 * 运行内容：先下 50% 占空比，再依次启动 CH1~CH3 及互补输出。 */
static void program_start_tim1_pwm_outputs(void)
{
    if (g_sys.tim1_pwm_started != 0U) return;
    program_apply_center_duty();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    g_sys.tim1_pwm_started = 1U;
}

/* ── 功率级控制 ── */

/* 函数作用：控制电机驱动芯片的休眠/唤醒。
 * 输入：enable 为 0=休眠，1=唤醒。输出：无返回值。
 * 调用频率：启停和故障处理时调用。
 * 运行内容：通过 N_SLEEP 引脚控制 MP6539B 功率级使能状态。 */
void program_set_power_stage_enable(uint8_t enable)
{
    /** 归一化后的目标状态：1=唤醒, 0=休眠 */
    uint8_t next_state = (enable != 0U) ? 1U : 0U;
    HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin,
                      (next_state != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_control.power_stage_enabled = next_state;
    g_program_telemetry.power_stage_enabled = next_state;
}

/* ── 故障检测 ── */

/* 函数作用：读取驱动芯片 nFAULT 引脚状态。
 * 输入：无。输出：返回 1=故障有效（低电平）。调用频率：快环入口和后台任务中调用。 */
uint8_t program_is_driver_fault_active(void)
    { return (HAL_GPIO_ReadPin(N_FAULT_GPIO_Port, N_FAULT_Pin) == GPIO_PIN_RESET) ? 1U : 0U; }

/* ── DWT 计时 ── */

/* 函数作用：使能 DWT 周期计数器，用于快环耗时统计。
 * 输入：无。输出：无返回值；通过 g_sys.dwt_cycle_counter_ready 反映成功状态。
 * 调用频率：系统初始化时调用一次。
 * 运行内容：使能 TRCENA → 清零 CYCCNT → 使能 CYCCNTENA，并计算快环对应的 CPU 周期数。 */
static void program_init_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_sys.dwt_cycle_counter_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
    if (SystemCoreClock == 0U) g_sys.fast_loop_period_cycles = 1U;
    else {
        g_sys.fast_loop_period_cycles = (uint32_t)((float)SystemCoreClock / PROGRAM_FAST_LOOP_HZ);
        if (g_sys.fast_loop_period_cycles == 0U) g_sys.fast_loop_period_cycles = 1U;
    }
}

/* 函数作用：统计快环执行耗时并检测超时。
 * 输入：elapsed_cycles 为本周期耗用 CPU 周期数。输出：无返回值，更新遥测字段。
 * 调用频率：每次快环结束时调用。
 * 运行内容：换算为 μs，更新当前/最大耗时，超时则置位 overrun 标志并累加计数。 */
static void program_update_fast_loop_timing(uint32_t elapsed_cycles)
{
    /** 每 CPU 周期对应时间 (μs) */
    float cycles_to_us;
    /** 本拍快环耗时 (μs) */
    float elapsed_us;
    g_program_telemetry.fast_loop_cycles = elapsed_cycles;
    if (elapsed_cycles > g_program_telemetry.fast_loop_cycles_max)
        g_program_telemetry.fast_loop_cycles_max = elapsed_cycles;
    if (SystemCoreClock == 0U) elapsed_us = 0.0f;
    else { cycles_to_us = 1000000.0f / (float)SystemCoreClock; elapsed_us = (float)elapsed_cycles * cycles_to_us; }
    g_program_telemetry.fast_loop_time_us = elapsed_us;
    if (elapsed_us > g_program_telemetry.fast_loop_time_max_us)
        g_program_telemetry.fast_loop_time_max_us = elapsed_us;
    g_program_telemetry.fast_loop_period_us = PROGRAM_FAST_LOOP_PERIOD_US;
    if (elapsed_cycles > g_sys.fast_loop_period_cycles) {
        g_program_telemetry.fast_loop_overrun = 1U;
        g_program_telemetry.fast_loop_overrun_count++;
    } else { g_program_telemetry.fast_loop_overrun = 0U; }
}

/* ── 遥测与波形 ── */

/* 函数作用：更新程序层遥测对象到最新状态。
 * 输入：无。输出：无返回值。
 * 调用频率：快环中每次完成控制后调用，后台任务中按需调用。
 * 运行内容：将电机状态、ADC、编码器、速度/位置/电流等关键变量同步写入 g_program_telemetry。 */
void program_update_debug_telemetry(void)
{
    float speed_ref_mech_rad_s_for_telemetry = g_motor.speed_ref_mech_rad_s;
    if (g_motor.position_loop_enable == 0U)
        speed_ref_mech_rad_s_for_telemetry = program_rpm_to_rad_s(g_motor.speed_ref_mech_rpm);

    g_program_telemetry.pwm_enable_cmd = g_motor.run_request;
    g_program_telemetry.power_stage_enabled = g_control.power_stage_enabled;
    g_program_telemetry.control_state = (uint8_t)g_motor.state;
    g_program_telemetry.encoder_align_done = g_encoder.align_done;
    g_program_telemetry.ma600a_sample_counter = g_ma600a.sample_counter;
    g_program_telemetry.ma600a_reject_count = g_ma600a.reject_count;
    g_program_telemetry.ma600a_comm_error_count = g_ma600a.comm_error_count;
    g_program_telemetry.speed_observer_window_samples = PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES;
    g_program_telemetry.current_loop_enable = g_motor.current_loop_enable;
    g_program_telemetry.position_loop_enable = g_motor.position_loop_enable;
    g_program_telemetry.control_angle_open_loop_enable = g_motor.control_angle_open_loop_enable;
    g_program_telemetry.theta_open_loop = g_motor.theta_open_loop;
    g_program_telemetry.control_angle_open_loop_speed_elec = g_motor.control_angle_open_loop_speed_elec;
    g_program_telemetry.id_ref_cmd = g_motor.id_ref;
    g_program_telemetry.iq_ref_cmd = g_motor.iq_ref;
    g_program_telemetry.id_ref_applied_cmd = g_control.id_ref_applied_a;
    g_program_telemetry.iq_ref_applied_cmd = g_control.iq_ref_applied_a;
    g_program_telemetry.ud_ref_cmd = g_motor.ud_ref;
    g_program_telemetry.uq_ref_cmd = g_motor.uq_ref;
    g_program_telemetry.open_loop_speed_elec = g_motor.open_loop_speed_elec;
    g_program_telemetry.speed_loop_ready = g_encoder.speed_ready;
    g_program_telemetry.speed_ref_mech_rad_s = speed_ref_mech_rad_s_for_telemetry;
    g_program_telemetry.speed_ref_mech_applied_rad_s = g_motor.speed_ref_mech_applied_rad_s;
    g_program_telemetry.speed_meas_raw_mech_rad_s = g_encoder.speed_raw_mech_rad_s;
    g_program_telemetry.speed_meas_mech_rad_s = g_motor.speed_meas_mech_rad_s;
    g_program_telemetry.speed_ref_mech_rpm = program_rad_s_to_rpm(speed_ref_mech_rad_s_for_telemetry);
    g_program_telemetry.speed_ref_mech_applied_rpm = program_rad_s_to_rpm(g_motor.speed_ref_mech_applied_rad_s);
    g_program_telemetry.speed_meas_raw_mech_rpm = program_rad_s_to_rpm(g_encoder.speed_raw_mech_rad_s);
    g_program_telemetry.speed_meas_mech_rpm = program_rad_s_to_rpm(g_motor.speed_meas_mech_rad_s);
    g_program_telemetry.speed_error_mech_rad_s = g_motor.speed_ref_mech_applied_rad_s - g_motor.speed_meas_mech_rad_s;
    g_program_telemetry.position_ref_mech_rad = g_motor.position_ref_mech_rad;
    g_program_telemetry.position_meas_mech_rad = g_motor.position_meas_mech_rad;
    g_program_telemetry.position_error_mech_rad = g_motor.position_error_mech_rad;
    g_program_telemetry.position_ref_mech_deg = g_motor.position_ref_mech_deg;
    g_program_telemetry.position_meas_mech_deg = g_motor.position_meas_mech_deg;
    g_program_telemetry.position_error_mech_deg = g_motor.position_error_mech_deg;
    g_program_telemetry.position_kd = g_motor.position_kd;
    g_program_telemetry.speed_loop_dt_s = g_control.speed_loop_dt_s;
    g_program_telemetry.speed_meas_lpf_cutoff_hz = g_motor.speed_meas_lpf_cutoff_hz;
    g_program_telemetry.speed_ref_elec_rad_s = g_motor.speed_ref_elec_rad_s;
    g_program_telemetry.speed_meas_elec_rad_s = g_motor.speed_meas_elec_rad_s;
    g_program_telemetry.uq_limit_v = g_motor.voltage_limit;
    g_program_telemetry.iq_limit_a = g_motor.iq_limit;
    g_program_telemetry.voltage_limit_v = g_motor.voltage_limit;
    g_program_telemetry.encoder_elec_offset_rad = g_encoder.elec_offset_rad;
    g_program_telemetry.duty_a = g_foc.duty.duty_a;
    g_program_telemetry.duty_b = g_foc.duty.duty_b;
    g_program_telemetry.duty_c = g_foc.duty.duty_c;
    g_program_telemetry.fast_loop_period_us = PROGRAM_FAST_LOOP_PERIOD_US;
}

/* 函数作用：按固定节拍发送 VOFA 波形数据。
 * 输入：now_ms 为当前 1 ms 调度计数。输出：无返回值。
 * 调用频率：后台慢任务中约 1 kHz 调用，内部限制为 2 ms 一次。
 * 运行内容：将速度/位置打包成 JustFloat 二进制帧，非阻塞送入 USART1 TX DMA。 */
static void program_send_wave_if_needed(uint32_t now_ms)
{
    /** VOFA 波形数据缓冲：速度、位置等遥测值 */
    float wave_buf[4];
    if ((now_ms - g_sys.last_wave_tick_ms) < PROGRAM_WAVE_PERIOD_MS) return;
    g_sys.last_wave_tick_ms = now_ms;
    wave_buf[0] = g_program_telemetry.speed_meas_mech_rpm;
    wave_buf[1] = g_program_telemetry.position_meas_mech_deg;
    wave_buf[2] = g_program_telemetry.position_meas_mech_rad;
    (void)cli_uart_send_vofa(wave_buf, 4U);
}

/* 函数作用：更新程序层故障标志位。
 * 输入：无。输出：无返回值。
 * 调用频率：后台慢任务和快环入口中调用。
 * 运行内容：读取 nFAULT 引脚状态并写入遥测。 */
static void program_update_fault_flags(void)
{
    g_program_telemetry.driver_fault_active = program_is_driver_fault_active();
}

/* ── UART 命令解析 ── */

/* 函数作用：解析 UART 接收到的文本命令并执行。
 * 输入：无。输出：无返回值。
 * 调用频率：后台慢任务中约 1 kHz 调用。
 * 运行内容：解析 HM/PV/PT/PP 命令，控制启停、转速、电流限幅、目标位置。 */
static void program_parse_uart_command(void)
{
    /** 等号位置指针 */
    char *eq;
    /** 命令字符串指针（指向缓冲区头部） */
    char *cmd;
    /** 解析出的整数值 */
    int value;
    /** 正负号 */
    int sign;
    /** 数字遍历指针 */
    char *p;

    if (g_uart_rx.rx_done == 0U) return;
    g_uart_rx.rx_done = 0U;

    cmd = g_uart_rx.rx_buf;
    eq = strchr(cmd, '=');
    if (eq == 0) {
        g_uart_rx.rx_idx = 0U;
        memset(g_uart_rx.rx_buf, 0, sizeof(g_uart_rx.rx_buf));
        return;
    }

    /* 简易 atoi：解析 = 后面的整数值 */
    p = eq + 1;
    sign = 1;
    if (*p == '-') { sign = -1; p++; }
    value = 0;
    while (*p >= '0' && *p <= '9') { value = value * 10 + (*p - '0'); p++; }
    value *= sign;

    if (strncmp(cmd, "HM", 2) == 0) {
        /* HM=0 停机, HM=1 启动回零（位置模式，目标 0°） */
        if (value == 0) {
            motor_state_set_run_request(&g_motor, 0U);
        } else {
            g_motor.position_ref_mech_deg = 0.0f;
            g_motor.position_loop_enable = 1U;
            motor_state_set_run_request(&g_motor, 1U);
        }
        cli_uart_send_text("HM OK\r\n");
    }
    else if (strncmp(cmd, "PV", 2) == 0) {
        /* PV=1000 → 转速 1000 rpm */
        g_motor.speed_ref_mech_rpm = (float)value;
        g_motor.speed_ref_mech_rad_s = program_rpm_to_rad_s((float)value);
        g_motor.position_loop_enable = 0U;
        if (g_motor.run_request == 0U) motor_state_set_run_request(&g_motor, 1U);
        cli_uart_send_text("PV OK\r\n");
    }
    else if (strncmp(cmd, "PT", 2) == 0) {
        /* PT=3000 → 电流限幅 3000mA = 3.0A */
        g_motor.iq_limit = (float)value / 1000.0f;
        cli_uart_send_text("PT OK\r\n");
    }
    else if (strncmp(cmd, "PP", 2) == 0) {
        /* PP=360 → 目标位置 360°（一圈） */
        g_motor.position_ref_mech_deg = (float)value;
        g_motor.position_loop_enable = 1U;
        if (g_motor.run_request == 0U) motor_state_set_run_request(&g_motor, 1U);
        cli_uart_send_text("PP OK\r\n");
    }

    g_uart_rx.rx_idx = 0U;
    memset(g_uart_rx.rx_buf, 0, sizeof(g_uart_rx.rx_buf));
}

/* ── 初始化与任务 ── */

/* 函数作用：初始化程序层遥测对象，给调试观察提供稳定初值。
 * 输入：无。输出：无返回值。
 * 调用频率：系统启动时只调用一次。
 * 运行内容：清零全部 ADC 与控制量缓存，并写入默认偏置和默认母线电压。 */
static void program_init_telemetry(void)
{
    (void)memset((void *)&g_program_telemetry, 0, sizeof(g_program_telemetry));
    g_program_telemetry.ia_offset_raw = PROGRAM_DEFAULT_CURRENT_OFFSET_RAW;
    g_program_telemetry.ib_offset_raw = PROGRAM_DEFAULT_CURRENT_OFFSET_RAW;
    g_program_telemetry.ic_offset_raw = PROGRAM_DEFAULT_CURRENT_OFFSET_RAW;
    g_program_telemetry.vbus = PROGRAM_DEFAULT_VBUS_V;
    g_program_telemetry.duty_a = 0.5f;
    g_program_telemetry.duty_b = 0.5f;
    g_program_telemetry.duty_c = 0.5f;
    g_program_telemetry.id_ref_applied_cmd = 0.0f;
    g_program_telemetry.iq_ref_applied_cmd = 0.0f;
}

/* 函数作用：启动 ADC2 DMA + TIM6 触发链，用于 VBUS/NTC 慢变量采样。
 * 输入：无。输出：无返回值；任一步失败则进入 Error_Handler()。
 * 调用频率：系统启动时只调用一次。
 * 运行内容：先启动 ADC2 DMA，再启动 TIM6 更新中断，让 ADC2 按 1 kHz 节拍采样。 */
static void program_start_adc2_dma_chain(void)
{
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_sys.adc2_dma_buf, PROGRAM_ADC2_DMA_LENGTH) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
        Error_Handler();
}

/* 函数作用：启动 ADC1 注入组采样链，用于三相电流同步采样。
 * 输入：无。输出：无返回值；任一步失败则进入 Error_Handler()。
 * 调用频率：系统启动时只调用一次。
 * 运行内容：启动 TIM1 PWM → 使能 ADC1 注入组中断，让 TIM1_TRGO2 周期性触发 IA/IB/IC 三通道采样。 */
static void program_start_adc1_injected_chain(void)
{
    program_start_tim1_pwm_outputs();
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
        Error_Handler();
}

/* 函数作用：把最新 ADC 原始数据换算到程序层遥测对象。
 * 输入：无。输出：无返回值。
 * 调用频率：后台慢任务中约 1 kHz 调用。
 * 运行内容：读取 ADC2 DMA 缓冲区 VBUS/NTC 原始值并滤波换算。 */
static void program_update_measurements(void)
{
    /** ADC2 母线电压原始码值 */
    uint16_t vbus_raw = g_sys.adc2_dma_buf[0];
    /** ADC2 NTC 温度原始码值 */
    uint16_t ntc_raw  = g_sys.adc2_dma_buf[1];
    g_program_telemetry.vbus_raw = vbus_raw;
    g_program_telemetry.ntc_raw = ntc_raw;
    g_program_telemetry.vbus = filter_lpf_f32_update(&g_sys.vbus_lpf,
        program_convert_vbus_from_raw(vbus_raw));
    program_update_debug_telemetry();
}

/* 函数作用：读取 MA600A 机械角度并更新速度测量和遥测。
 * 输入：无。输出：无返回值。
 * 调用频率：快环中每次获得有效角度样本后调用。
 * 运行内容：有效时更新角度码/度/弧度并触发速度观测；无效时复位编码器观测器和零位对齐。 */
static void program_update_encoder_measurements(void)
{
    // ma600a SPI 数据读取成功
    if (g_ma600a.data_valid != 0U) {
        g_program_telemetry.ma600a_angle_raw = g_ma600a.angle_raw;
        g_program_telemetry.ma600a_angle_deg = g_ma600a.angle_deg;
        g_program_telemetry.ma600a_angle_rad = g_ma600a.angle_rad;
        g_program_telemetry.ma600a_angle_valid = g_ma600a.data_valid;
        g_program_telemetry.ma600a_consecutive_bad_count = g_ma600a.consecutive_bad_count;
        program_update_speed_measurement();
        if ((g_encoder.align_done != 0U) && (g_motor.control_angle_open_loop_enable != 0U))
            g_program_telemetry.theta_elec = program_wrap_angle_0_2pi(g_motor.theta_open_loop);
        else if (g_encoder.align_done != 0U)
            g_program_telemetry.theta_elec = program_get_encoder_aligned_elec_angle_rad();
        else
            g_program_telemetry.theta_elec = program_get_encoder_raw_elec_angle_rad();
    } else {
        g_program_telemetry.ma600a_angle_valid = 0U;
        g_program_telemetry.ma600a_consecutive_bad_count = g_ma600a.consecutive_bad_count;
        program_reset_encoder_observer();
        program_reset_encoder_alignment();
    }
}

/* ── 公共接口 ── */

/* 函数作用：程序层初始化入口。
 * 输入：无。输出：无返回值。
 * 调用频率：系统启动后只调用一次。
 * 运行内容：保持驱动休眠→ADC校准→初始化滤波/对象→启动ADC1/ADC2采样链→使能UART接收。 */
void program_init(void)
{
    program_set_power_stage_enable(0U);
    program_init_telemetry();
    g_program_debug_pwm_test.enable = 0U;
    g_program_debug_pwm_test.duty_a = PROGRAM_DEBUG_PWM_TEST_DEFAULT_DUTY_A;
    g_program_debug_pwm_test.duty_b = PROGRAM_DEBUG_PWM_TEST_DEFAULT_DUTY_B;
    g_program_debug_pwm_test.duty_c = PROGRAM_DEBUG_PWM_TEST_DEFAULT_DUTY_C;

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

    foc_core_init(&g_foc);
    foc_core_set_bus_voltage(&g_foc, PROGRAM_DEFAULT_VBUS_V);
    motor_state_init(&g_motor);
    g_motor.state = MOTOR_STATE_READY;
    g_motor.run_request = 0U;
    g_motor.speed_loop_enable = 1U;
    g_motor.current_loop_enable = 1U;
    g_motor.position_loop_enable = 0U;
    g_motor.control_angle_open_loop_enable = 0U;
    g_motor.theta_open_loop = 0.0f;
    g_motor.ud_ref = PROGRAM_OPEN_LOOP_DEFAULT_UD_V;
    g_motor.uq_ref = 0.0f;
    g_motor.speed_ref_mech_rpm = program_rad_s_to_rpm(PROGRAM_DEFAULT_SPEED_REF_MECH_RAD_S);
    g_motor.speed_ref_mech_rad_s = program_rpm_to_rad_s(g_motor.speed_ref_mech_rpm);
    g_motor.speed_ref_mech_applied_rad_s = 0.0f;
    g_motor.speed_meas_mech_rad_s = 0.0f;
    g_motor.speed_ref_elec_rad_s = 0.0f;
    g_motor.speed_meas_elec_rad_s = 0.0f;
    g_motor.control_angle_open_loop_speed_elec = PROGRAM_OPEN_LOOP_DEFAULT_SPEED_ELEC;
    g_motor.position_ref_mech_deg = 0.0f;
    g_motor.position_ref_mech_rad = 0.0f;
    g_motor.position_meas_mech_deg = 0.0f;
    g_motor.position_meas_mech_rad = 0.0f;
    g_motor.position_error_mech_deg = 0.0f;
    g_motor.position_error_mech_rad = 0.0f;
    g_motor.position_kp = PROGRAM_DEFAULT_POSITION_KP;
    g_motor.position_ki = PROGRAM_DEFAULT_POSITION_KI;
    g_motor.position_kd = PROGRAM_DEFAULT_POSITION_KD;
    g_motor.position_integral_speed = 0.0f;
    g_motor.position_speed_limit_mech_rad_s = PROGRAM_DEFAULT_POSITION_SPEED_LIMIT_MECH_RAD_S;
    g_motor.open_loop_speed_elec = 0.0f;
    g_motor.speed_kp = PROGRAM_DEFAULT_SPEED_KP;
    g_motor.speed_ki = PROGRAM_DEFAULT_SPEED_KI;
    g_motor.speed_meas_lpf_cutoff_hz = PROGRAM_DEFAULT_SPEED_MEAS_LPF_CUTOFF_HZ;
    g_motor.speed_integral_iq = 0.0f;
    g_motor.speed_integral_uq = 0.0f;
    g_motor.iq_limit = PROGRAM_DEFAULT_IQ_LIMIT_A;
    g_motor.current_kp = program_current_loop_kp_from_bandwidth_hz(PROGRAM_DEFAULT_CURRENT_LOOP_BANDWIDTH_HZ);
    g_motor.current_ki = program_current_loop_ki_from_bandwidth_hz(PROGRAM_DEFAULT_CURRENT_LOOP_BANDWIDTH_HZ);
    g_motor.id_integral_v = 0.0f;
    g_motor.iq_integral_v = 0.0f;
    g_motor.voltage_limit = 0.0f;
    g_control.position_loop_enable_prev = 0U;
    g_control.current_loop_enable_prev = 1U;
    program_reset_position_loop();
    program_reset_encoder_alignment();
    program_reset_encoder_observer();
    program_reset_encoder_align_runtime();
    foc_core_set_electrical_angle(&g_foc, 0.0f);
    filter_lpf_f32_init(&g_sys.vbus_lpf, 0.1f, PROGRAM_DEFAULT_VBUS_V);
    cli_uart_init(&huart1);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart_rx.rx_char, 1);
    ma600a_init(&g_ma600a, &hspi1, ENC_CS_GPIO_Port, ENC_CS_Pin);
    program_init_cycle_counter();
    (void)ma600a_read_angle(&g_ma600a);
    program_update_fault_flags();
    program_update_debug_telemetry();
    program_start_adc2_dma_chain();
    program_start_adc1_injected_chain();
}

/* 函数作用：程序层后台任务入口。
 * 输入：无。输出：无返回值。
 * 调用频率：在 while(1) 中持续调用，内部按 TIM6 1ms 节拍执行。
 * 运行内容：UART 命令解析→ADC 工程量换算→故障检测→VOFA 波形发送。 */
void program_task(void)
{
    /** 当前 1ms 时基计数值 */
    uint32_t now_ms = g_sys.tim6_tick_ms;
    if (now_ms == g_sys.last_slow_task_tick_ms) return;
    g_sys.last_slow_task_tick_ms = now_ms;
    program_parse_uart_command();
    program_update_measurements();
    program_update_fault_flags();
    program_send_wave_if_needed(now_ms);
}

/* ── HAL 回调适配层 ── */

/* 函数作用：程序层定时器周期回调转发入口。
 * 输入：htim 为触发中断的定时器句柄。输出：无返回值。
 * 调用频率：由 HAL 在定时器更新中断中调用（当前主要是 TIM6 1kHz）。
 * 运行内容：识别 TIM6 后递增慢任务时基，供后台任务调度共用。 */
void program_tim_period_elapsed_callback(TIM_HandleTypeDef *htim)
{
    if ((htim != 0) && (htim->Instance == TIM6))
        g_sys.tim6_tick_ms++;
}

/* 函数作用：程序层 regular ADC 完成回调转发入口。
 * 输入：hadc 为完成转换的 ADC 句柄。输出：无返回值。
 * 调用频率：regular ADC 完成转换后由 HAL 调用。
 * 运行内容：当前阶段 ADC2 通过 DMA 环形缓冲读取，此处保留空接口。 */
void program_adc_conv_cplt_callback(ADC_HandleTypeDef *hadc) { (void)hadc; }

/* 函数作用：程序层 injected ADC 完成回调转发入口（10kHz 快环入口）。
 * 输入：hadc 为完成注入组转换的 ADC 句柄。输出：无返回值。
 * 调用频率：每次 TIM1 触发 IA/IB/IC 三通道采样完成后调用。
 * 运行内容：读取三相原始ADC→零偏校准→编码器读取→电流反馈→快环主控制→耗时统计。 
 */
void program_adc_injected_conv_cplt_callback(ADC_HandleTypeDef *hadc)
{
    // Ia, Ib, Ic 是通过ADC 中断获取数据，此处仅读取并保存 不使用DMA
    /** A 相电流原始码值（注入组 RANK_1） */
    uint16_t ia_raw;
    /** B 相电流原始码值（注入组 RANK_2） */
    uint16_t ib_raw;
    /** C 相电流原始码值（注入组 RANK_3） */
    uint16_t ic_raw;
    /** 快环起始 DWT 计数值，用于耗时统计 */
    uint32_t fast_loop_start_cycles;
    /** 电流反馈用控制电角度 (rad)，对齐完成前为 0 */
    float theta_feedback;

    if ((hadc == 0) || (hadc->Instance != ADC1)) return;
    if (g_sys.dwt_cycle_counter_ready != 0U) fast_loop_start_cycles = DWT->CYCCNT;
    else fast_loop_start_cycles = 0U;

    ia_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    ib_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    ic_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    g_program_telemetry.ia_raw = ia_raw;
    g_program_telemetry.ib_raw = ib_raw;
    g_program_telemetry.ic_raw = ic_raw;

    /* 零偏校准  零电流基准 */
    if (g_program_telemetry.current_offset_ready == 0U) {
        g_sys.ia_offset_sum += ia_raw;
        g_sys.ib_offset_sum += ib_raw;
        g_sys.ic_offset_sum += ic_raw;
        g_program_telemetry.current_offset_sample_count++;
        if (g_program_telemetry.current_offset_sample_count >= PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES) {
            g_program_telemetry.ia_offset_raw = (uint16_t)(g_sys.ia_offset_sum / PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES);
            g_program_telemetry.ib_offset_raw = (uint16_t)(g_sys.ib_offset_sum / PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES);
            g_program_telemetry.ic_offset_raw = (uint16_t)(g_sys.ic_offset_sum / PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES);
            g_program_telemetry.current_offset_ready = 1U;
        }
    }

    (void)ma600a_read_angle(&g_ma600a);
    program_update_encoder_measurements();
    program_update_control_angle_open_loop_state();

    /** 对齐完成 → 用编码器真实电角度做 Park 变换 */
    if (g_encoder.align_done != 0U){
        theta_feedback = program_get_control_elec_angle_rad();
    }
    /** 对齐未完成 → 电角度为 0，电流反馈不参与控制 */
    else{
        theta_feedback = 0.0f;
    }
        
    program_update_current_feedback_from_raw(ia_raw, ib_raw, ic_raw, theta_feedback);
    motor_state_task(&g_motor, &g_foc, g_sys.tim6_tick_ms);

    if (g_sys.dwt_cycle_counter_ready != 0U)
        program_update_fast_loop_timing(DWT->CYCCNT - fast_loop_start_cycles);
}

/* HAL 定时器周期回调 → 转发到 program 层 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    { program_tim_period_elapsed_callback(htim); }

/* HAL regular ADC 回调 → 转发到 program 层 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
    { program_adc_conv_cplt_callback(hadc); }

/* HAL injected ADC 回调 → 转发到 program 层 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
    { program_adc_injected_conv_cplt_callback(hadc); }

/* HAL SPI 收发完成回调 → 转发到 ma600a 驱动层 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
    { ma600a_spi_txrx_cplt_callback(&g_ma600a, hspi); }

/* HAL SPI 错误回调 → 转发到 ma600a 驱动层 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
    { ma600a_spi_error_callback(&g_ma600a, hspi); }

/* 函数作用：UART RX 中断回调——逐字符接收，遇 \n 置位完成标志。
 * 输入：huart 为 UART 句柄。输出：无返回值。
 * 调用频率：USART1 每收到一个字符由 HAL 中断调用。
 * 运行内容：\n/\r 结束一行→置位 g_uart_rx.rx_done；否则缓存字符→重新使能 IT 接收。 
 * */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (g_uart_rx.rx_char == '\n' || g_uart_rx.rx_char == '\r') {
            if (g_uart_rx.rx_idx > 0U) {
                g_uart_rx.rx_buf[g_uart_rx.rx_idx] = '\0';
                g_uart_rx.rx_done = 1U;
            }
        } else if (g_uart_rx.rx_idx < PROGRAM_UART_RX_BUF_SIZE - 1U) {
            g_uart_rx.rx_buf[g_uart_rx.rx_idx++] = g_uart_rx.rx_char;
        }
        HAL_UART_Receive_IT(huart, (uint8_t *)&g_uart_rx.rx_char, 1);
    }
}

/* ── 获取器 ── */

/* 获取当前程序层遥测对象（只读） */
const volatile program_telemetry_t *program_get_telemetry(void) { return &g_program_telemetry; }
/* 获取当前电机状态机对象 */
motor_state_t *program_get_motor(void) { return &g_motor; }
/* 获取当前 FOC 核心对象 */
foc_core_t     *program_get_foc(void)   { return &g_foc; }
