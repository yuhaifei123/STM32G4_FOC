#ifndef _PROGRAM_CONFIG_H
#define _PROGRAM_CONFIG_H

#include "main.h"
#include "motor_params.h"

/**
 * @brief 程序层配置宏集中定义头文件
 *
 * 汇总 program.c / program_current.c / program_svpwm.c / motor_state.c
 * 共用的全部 PROGRAM_ 前缀配置参数，按业务主题分节管理。
 * 电机本体参数（MOTOR_ 前缀）由 motor_params.h 独立管理。
 */

/* ── 系统级时序 ── */
/** 圆周率 π */
#define PROGRAM_PI                            3.14159265359f
/** 快环频率 (Hz) = 10kHz */
#define PROGRAM_FAST_LOOP_HZ                  10000.0f
/** 快环周期 (s) = 100μs */
#define PROGRAM_FAST_LOOP_DT_S                (1.0f / PROGRAM_FAST_LOOP_HZ)
/** 快环周期 (μs) = 100μs，DWT 超时判定用 */
#define PROGRAM_FAST_LOOP_PERIOD_US           (1000000.0f / PROGRAM_FAST_LOOP_HZ)

/* ── 斜坡速率 ── */
/** 速度参考斜坡速率 (rad/s?)，防止速度指令突变 */
#define PROGRAM_SPEED_REF_RAMP_RAD_S2         100.0f
/** 电流参考斜坡速率 (A/s)，防止电流指令突变造成转矩冲击 */
#define PROGRAM_CURRENT_REF_RAMP_A_PER_S      150.0f

/* ── ADC 硬件参数 ── */
/** ADC 参考电压 (V) */
#define PROGRAM_ADC_REF_V                     3.3f
/** ADC 满量程码值 (12bit) */
#define PROGRAM_ADC_FULL_SCALE_COUNTS         4095.0f
/** 采样电阻阻值 (Ω)，串在电机相线上 */
#define PROGRAM_SHUNT_RESISTOR_OHM            0.01f
/** INA240 电流检测放大器固定增益 (V/V) */
#define PROGRAM_CURRENT_SENSE_GAIN            20.0f
/** 母线分压电阻上臂 (Ω) */
#define PROGRAM_VBUS_R_UP_OHM                 240000.0f
/** 母线分压电阻下臂 (Ω) */
#define PROGRAM_VBUS_R_DOWN_OHM               10000.0f
/** ADC2 DMA 缓冲长度（VBUS + NTC 共 2 通道） */
#define PROGRAM_ADC2_DMA_LENGTH               2U

/* ── 零偏校准 ── */
/** 默认零偏码值（12bit ADC 中点，对应 INA240 Vref/2 = 1.65V） */
#define PROGRAM_DEFAULT_CURRENT_OFFSET_RAW    2048U
/** 零偏采集样本数（1024 × 100μs ≈ 102ms） */
#define PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES 1024U

/* ── 系统默认值 ── */
/** 默认母线电压 (V)，校准前使用 */
#define PROGRAM_DEFAULT_VBUS_V                48.0f

/* ── 位置环时序（200Hz） ── */
/** 位置环频率 (Hz) */
#define PROGRAM_POSITION_LOOP_HZ              200.0f
/** 位置环周期 (s) */
#define PROGRAM_POSITION_LOOP_DT_S            (1.0f / PROGRAM_POSITION_LOOP_HZ)

/* ── 速度观测 ── */
/** 速度观测窗口拍数（20 × 100μs = 2ms） */
#define PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES 20U
/** 速度测量 LPF 默认截止频率 (Hz) */
#define PROGRAM_DEFAULT_SPEED_MEAS_LPF_CUTOFF_HZ 150.0f
/** 速度环控制周期 (s) = 快环周期 × 窗口拍数 */
#define PROGRAM_SPEED_LOOP_DT_S               (PROGRAM_FAST_LOOP_DT_S * (float)PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES)

/* ── 开环拖动参数 ── */
/** 开环拖动电角速度 (rad/s) */
#define PROGRAM_OPEN_LOOP_DEFAULT_SPEED_ELEC  2000.0f
/** 开环拖动 d 轴电压 (V) */
#define PROGRAM_OPEN_LOOP_DEFAULT_UD_V        0.0f
/** 开环拖动 q 轴电压 (V) */
#define PROGRAM_OPEN_LOOP_DEFAULT_UQ_V        1.0f
/** 默认机械转速参考 = 电角速度 / 极对数 */
#define PROGRAM_DEFAULT_SPEED_REF_MECH_RAD_S  (PROGRAM_OPEN_LOOP_DEFAULT_SPEED_ELEC / MOTOR_POLE_PAIRS)

/* ── 速度环 PI 默认增益 ── */
/** 速度环比例增益 */
#define PROGRAM_DEFAULT_SPEED_KP              0.0015f
/** 速度环积分增益 */
#define PROGRAM_DEFAULT_SPEED_KI              0.015f

/* ── 位置环默认参数 ── */
/** 位置环比例增益 */
#define PROGRAM_DEFAULT_POSITION_KP           3.0f
/** 位置环积分增益 */
#define PROGRAM_DEFAULT_POSITION_KI           0.0f
/** 位置环微分（阻尼）增益 */
#define PROGRAM_DEFAULT_POSITION_KD           0.0f
/** 位置环输出速度限幅 (rad/s) = 默认速度 / 减速比 */
#define PROGRAM_DEFAULT_POSITION_SPEED_LIMIT_MECH_RAD_S (PROGRAM_DEFAULT_SPEED_REF_MECH_RAD_S / MOTOR_GEAR_RATIO)
/** 位置测量 LPF 截止频率 (Hz) */
#define PROGRAM_POSITION_MEAS_LPF_CUTOFF_HZ   12.0f

/* ── 位置环 hold/creep 阈值 ── */
/** 位置 hold：进入保持的误差阈值 (rad) ≈ 1.2° */
#define PROGRAM_POSITION_HOLD_ERR_RAD         0.021f
/** 位置 hold：退出保持的误差阈值 (rad) ≈ 1.8°（滞回） */
#define PROGRAM_POSITION_HOLD_RELEASE_ERR_RAD 0.031f
/** 位置 hold：保持时允许的最大输出速度 (rad/s) */
#define PROGRAM_POSITION_HOLD_SPEED_MECH_RAD_S 0.50f
/** 位置 hold：连续超阈值周期数才释放（防抖） */
#define PROGRAM_POSITION_HOLD_RELEASE_CONFIRM_CYCLES 12U
/** 位置 creep：启动蠕动的误差阈值 (rad) ≈ 2.6° */
#define PROGRAM_POSITION_CREEP_ENABLE_ERR_RAD 0.045f
/** 位置 creep：蠕动补偿速度 (rad/s)，克服摩擦死区 */
#define PROGRAM_POSITION_CREEP_SPEED_MECH_RAD_S 0.020f

/* ── 电流环参数 ── */
/** 默认 q 轴电流限幅 (A) */
#define PROGRAM_DEFAULT_IQ_LIMIT_A            12.00f
/** 电机等效电阻 (Ω)，用于 PI 参数推导 */
#define PROGRAM_CURRENT_LOOP_EQ_RESISTANCE_OHM 0.7250f
/** 电机等效电感 (H)，用于 PI 参数推导 */
#define PROGRAM_CURRENT_LOOP_EQ_INDUCTANCE_H   0.0004100f
/** 电流环带宽 1kHz */
#define PROGRAM_CURRENT_LOOP_BW_1KHZ_HZ        1000.0f
/** 电流环带宽 2kHz */
#define PROGRAM_CURRENT_LOOP_BW_2KHZ_HZ        2000.0f
/** 电流环带宽 3kHz */
#define PROGRAM_CURRENT_LOOP_BW_3KHZ_HZ        3000.0f
/** 当前使用的电流环带宽（默认 1kHz） */
#define PROGRAM_DEFAULT_CURRENT_LOOP_BANDWIDTH_HZ PROGRAM_CURRENT_LOOP_BW_1KHZ_HZ
/** SVPWM 电压限幅系数 = 1/√3 */
#define PROGRAM_VOLTAGE_LIMIT_RATIO           0.57735026919f

/* ── 电流符号（硬件接线方向校正） ── */
/** A 相电流符号（±1，硬件接线方向校正） */
#define PROGRAM_CURRENT_SIGN_IA               (1.0f)
/** B 相电流符号 */
#define PROGRAM_CURRENT_SIGN_IB               (1.0f)
/** C 相电流符号 */
#define PROGRAM_CURRENT_SIGN_IC               (1.0f)

/* ── 编码器观测与量化保护 ── */
/** 连续角重归一化阈值 (rad) = 32 圈，防止浮点精度丢失 */
#define PROGRAM_ENCODER_OBSERVER_RENORM_RAD   (32.0f * MOTOR_TWO_PI)
/** 编码器 1 LSB 对应的机械角分辨率 (rad) */
#define PROGRAM_ENCODER_LSB_RAD               (MOTOR_TWO_PI / 65536.0f)
/** 速度零位保持：量化噪声倍数阈值 */
#define PROGRAM_SPEED_MEAS_ZERO_HOLD_SCALE    8.0f
/** 速度零位保持：最小机械转速死区 (rad/s)，低于此值强制归零 */
#define PROGRAM_SPEED_MEAS_ZERO_HOLD_MIN_MECH_RAD_S 0.35f

/* ── 编码器零位对齐 ── */
/** 对齐 Ud 电压 (V) */
#define PROGRAM_ALIGN_UD_V                    1.8f
/** 对齐保持拍数（8000 × 100μs = 800ms） */
#define PROGRAM_ALIGN_HOLD_TICKS              8000U
/** 对齐采样窗口拍数（最后 512 拍取 sin/cos 平均） */
#define PROGRAM_ALIGN_SAMPLE_WINDOW_TICKS     512U

/* ── 波形发送 ── */
/** VOFA 波形发送周期 (ms) */
#define PROGRAM_WAVE_PERIOD_MS                2U

/* ── UART 命令接收 ── */
/** UART RX 接收缓冲区大小 */
#define PROGRAM_UART_RX_BUF_SIZE              64U

/* ── debug PWM 测试 ── */
/** debug PWM 默认 A 相占空比 */
#define PROGRAM_DEBUG_PWM_TEST_DEFAULT_DUTY_A 0.30f
/** debug PWM 默认 B 相占空比 */
#define PROGRAM_DEBUG_PWM_TEST_DEFAULT_DUTY_B 0.40f
/** debug PWM 默认 C 相占空比 */
#define PROGRAM_DEBUG_PWM_TEST_DEFAULT_DUTY_C 0.60f

#endif /* _PROGRAM_CONFIG_H */
