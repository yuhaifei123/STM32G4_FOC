#ifndef PROGRAM_SVPWM_H
#define PROGRAM_SVPWM_H

#include "program.h"

/* ── 编码器速度测量 ── */

/**
 * @brief  基于编码器角度更新机械速度测量
 *
 * 维护连续机械角和测速窗口，窗口满后计算原始速度 → LPF 滤波 → 量化保护，
 * 最终输出 g_motor.speed_meas_mech_rad_s 和 g_motor.speed_meas_elec_rad_s。
 * 调用时机：每次获得有效 MA600A 角度样本后调用（快环 10kHz 内）。
 */
void program_update_speed_measurement(void);

/**
 * @brief  复位编码器测速观测器
 *
 * 清空连续角、测速窗口、滤波器和相关遥测量。
 * 调用时机：上电初始化、读角失效或重新对齐时调用。
 */
void program_reset_encoder_observer(void);

/**
 * @brief  复位编码器零位对齐结果
 *
 * 清空零位偏置和对齐完成标志。
 * 调用时机：重新对齐、停机或读角失效时调用。
 */
void program_reset_encoder_alignment(void);

/**
 * @brief  复位编码器对齐过程的运行时累积量
 *
 * 清空对齐计数器以及 sin/cos 平均所需的累积和。
 * 调用时机：开始重新对齐或完成对齐后调用。
 */
void program_reset_encoder_align_runtime(void);

/* ── 编码器对齐 ── */

/**
 * @brief  在对齐保持阶段采集编码器电角度样本
 *
 * 只在采样窗口内累积电角度的 sin/cos，用于后续 atan2 求平均角，降低编码器抖动影响。
 * 调用时机：零位对齐保持期间的每个快环调用。
 */
void  program_capture_encoder_alignment_sample(void);

/**
 * @brief  计算编码器零位对齐得到的平均电角度
 *
 * 基于累积的 sin/cos 用 atan2 求平均方向角，抗噪声能力远优于直接平均角度。
 * 调用时机：对齐结束时调用。
 * @return 对齐采样得到的电角度 (rad)
 */
float program_get_encoder_alignment_angle_rad(void);

/* ── 编码器角度获取 ── */
/**
 * @brief  读取编码器对应的转子机械角
 * @return 转子机械角 (rad)，已考虑编码器方向符号
 */
float program_get_encoder_rotor_mech_angle_rad(void);

/**
 * @brief  获取输出轴连续机械角（无跳变，可超过单圈）
 *
 * 优先使用连续机械角观测值，再按减速比折算回输出轴。
 * @return 输出轴连续机械角 (rad)
 */
float program_get_encoder_output_continuous_mech_angle_rad(void);

/**
 * @brief  获取输出轴单圈机械角（归一化到 0~2π）
 * @return 输出轴单圈机械角 (rad)
 */
float program_get_encoder_output_mech_angle_rad(void);

/**
 * @brief  获取未经零位补偿的原始电角度
 *
 * 由转子机械角 × 极对数直接换算，不含对齐偏置。
 * 调用时机：对齐采样和故障前角度观察时调用。
 * @return 原始电角度 (rad)
 */
float program_get_encoder_raw_elec_angle_rad(void);

/**
 * @brief  获取完成零位补偿后的电角度
 *
 * 在原始电角度基础上扣除对齐得到的电角偏置，
 * 使 θe=0 对应 A 相电流达到正向最大值的位置。
 * @return 对齐后的控制电角度 (rad)
 */
float program_get_encoder_aligned_elec_angle_rad(void);

/**
 * @brief  提供当前控制使用的电角度
 *
 * 集中封装控制角来源：开环模式用积分器角度，否则用编码器对齐角度。
 * 调用时机：快环每次执行控制时调用。
 * @return 控制电角度 (rad)
 */
float program_get_control_elec_angle_rad(void);

/**
 * @brief  更新控制角度开环状态
 *
 * 开环模式下对 theta_open_loop 按固定速度积分。
 * 调用时机：快环内每次控制前调用。
 */
void  program_update_control_angle_open_loop_state(void);

/* ── 角度工具 ── */

/** @brief 角度归一化到 [0, 2π)，NaN/Inf 返回 0 */
float program_wrap_angle_0_2pi(float angle_rad);

/** @brief 角度差归一化到 [-π, π)，避免跨 0/2π 跳变 */
float program_wrap_delta_pm_pi(float angle_rad);

/** @brief 角度归一化到 [0°, 360°)，NaN/Inf 返回 0 */
float program_wrap_angle_0_360_deg(float angle_deg);

/** @brief rad/s → rpm */
float program_rad_s_to_rpm(float speed_rad_s);

/** @brief rad → deg */
float program_rad_to_deg(float angle_rad);

/** @brief deg → rad */
float program_deg_to_rad(float angle_deg);

/** @brief rpm → rad/s */
float program_rpm_to_rad_s(float speed_rpm);

/**
 * @brief  限制浮点量上下界 [min, max]
 *
 * 为电流、电压和速度中间量提供统一限幅。
 * @param value     输入值
 * @param min_value  下限
 * @param max_value  上限
 * @return 限幅后的值
 */
float program_clamp_f32(float value, float min_value, float max_value);

/* ── LPF 系数 ── */

/**
 * @brief  由截止频率和采样周期换算一阶低通滤波系数 α
 *
 * 采用指数离散化：α = 1 - exp(-2π·fc·dt)，结果钳位到 [0, 1]。
 * 参数非法时返回 1.0（无滤波）。
 * @param cutoff_hz  截止频率 (Hz)
 * @param dt_s       采样周期 (s)
 * @return 滤波系数 α (0~1)
 */
float program_lpf_alpha_from_cutoff_hz(float cutoff_hz, float dt_s);

/* ── 速度斜坡 ── */

/**
 * @brief  按设定加速度更新实际速度给定
 *
 * 把上层目标速度平滑过渡到实际生效速度，减少转矩冲击。
 * 非位置环模式下自动将 rpm 转换为 rad/s。
 * 调用时机：快环内每次速度控制前调用。
 */
void program_update_speed_reference_ramp(void);

/* ── 量化保护 ── */

/**
 * @brief  在低速附近抑制编码器量化抖动
 *
 * 当速度指令和测速值都接近量化台阶时强制回零，
 * 避免静止时有微小抖动速度导致电机微振。
 * @param speed_mech_rad_s        待保护的机械角速度 (rad/s)
 * @param observer_window_samples 测速窗口样本数
 * @return 保护后的机械角速度
 */
float program_apply_speed_quantization_guard(float speed_mech_rad_s,
                                             uint32_t observer_window_samples);

/**
 * @brief  估算编码器测速量化分辨率
 * @param observer_window_samples 测速窗口样本数
 * @return 最小速度分辨率 (rad/s)
 */
float program_get_speed_quantization_rad_s(uint32_t observer_window_samples);

#endif /* PROGRAM_SVPWM_H */
