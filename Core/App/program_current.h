#ifndef PROGRAM_CURRENT_H
#define PROGRAM_CURRENT_H

#include "program.h"

/* ── 电流换算 ── */

/**
 * @brief  把原始 ADC 码值换算成相电流
 *
 * 公式：I = (raw - offset) × 3.3V/4095 / (20 × 0.01Ω)
 * @param raw         ADC 原始码值
 * @param offset_raw  零电流偏置码值
 * @return 相电流 (A)
 */
float program_convert_current_from_raw(uint16_t raw, uint16_t offset_raw);

/**
 * @brief  把原始 ADC 码值换算成母线电压
 *
 * 使用 240k/10k 分压比和 3.3V ADC 参考反算。
 * @param raw  VBUS 通道 ADC 原始码值
 * @return 母线电压 (V)
 */
float program_convert_vbus_from_raw(uint16_t raw);

/* ── 控制环 ── */

/**
 * @brief  执行速度环并更新 iq 或 uq 命令
 *
 * 先按分频条件运行位置环，再由速度 PI 生成电流/电压模式输出。
 * 调用时机：速度观测窗口完成后按需调用。
 */
void program_update_speed_loop(void);

/**
 * @brief  执行位置环并生成机械速度指令
 *
 * 计算位置误差 → PI 输出速度指令，包含 hold 滞回保持和 creep 蠕动补偿。
 * @param position_loop_dt_s 本次位置环实际周期 (s)
 */
void program_update_position_loop(float position_loop_dt_s);

/**
 * @brief  执行 d/q 轴电流环并更新 SVPWM 输出
 *
 * 更新实际电流给定、运行 d/q 轴 PI、电压限幅并下发 FOC 电压命令。
 * @param theta_cmd 本次控制使用的电角度 (rad)
 */
void program_run_current_loop(float theta_cmd);

/**
 * @brief  以电压模式直接运行 FOC（不经过电流环）
 *
 * 对 ud/uq 命令做矢量限幅后直接生成三相 PWM。
 * 调用时机：关闭电流环时每个快环调用。
 * @param theta_cmd 本次控制使用的电角度 (rad)
 */
void program_run_voltage_mode(float theta_cmd);

/**
 * @brief  把三相原始 ADC 采样更新为 abc/dq 电流反馈
 *
 * 完成零偏扣除、极性修正、Clarke/Park 变换，结果写入 g_foc 和遥测。
 * @param ia_raw     A 相 ADC 原始码值
 * @param ib_raw     B 相 ADC 原始码值
 * @param ic_raw     C 相 ADC 原始码值
 * @param theta_elec 当前反馈电角度 (rad)
 */
void program_update_current_feedback_from_raw(uint16_t ia_raw, uint16_t ib_raw,
                                              uint16_t ic_raw, float theta_elec);

/* ── 模式切换 ── */

/**
 * @brief  处理位置环使能状态切换
 *
 * 检测位置环开关沿并同步复位相关状态与参考量，使能时对齐当前位置。
 * 调用时机：快环内每次控制前调用。
 */
void program_handle_position_loop_mode_switch(void);

/**
 * @brief  处理电流环使能状态切换
 *
 * 在电流环模式切换时清空速度环和电流环残留状态。
 * 调用时机：快环内每次控制前调用。
 */
void program_handle_current_loop_mode_switch(void);

/* ── PI 控制器 ── */

/**
 * @brief  带抗饱和的浮点 PI 控制器
 *
 * 统一完成比例积分运算、积分限幅与抗积分饱和处理。
 * 输出越限时只在不加深饱和的方向更新积分。
 * @param ref        目标值
 * @param feedback   反馈值
 * @param kp         比例增益
 * @param ki         积分增益
 * @param dt_s       控制周期 (s)
 * @param integral   积分项指针（跨调用保持）
 * @param out_min    输出下限
 * @param out_max    输出上限
 * @return 限幅后的 PI 输出
 */
float program_run_pi_f32(float ref, float feedback, float kp, float ki,
                         float dt_s, float *integral, float out_min, float out_max);

/* ── 复位函数 ── */

/** @brief 复位速度环内部状态：清零积分、挂起标志 */
void program_reset_speed_loop(void);

/** @brief 复位位置环内部状态：清零积分器、hold 状态、LPF */
void program_reset_position_loop(void);

/** @brief 复位电流环内部状态：清零 d/q 给定、电压命令和 PI 积分 */
void program_reset_current_loop(void);

/** @brief 复位速度参考斜坡：清零机械/电角速度给定 */
void program_reset_speed_reference_ramp(void);

/** @brief 复位电流给定斜坡：清零实际生效的 id/iq 给定 */
void program_reset_current_reference_ramp(void);

/* ── 电压限制 ── */

/**
 * @brief  根据当前母线电压计算电压矢量幅值上限
 *
 * v_limit = vbus / √3，对应 SVPWM 线性调制区上限。
 * @return 允许的最大 dq 电压幅值 (V)
 */
float program_get_voltage_limit_v(void);

/**
 * @brief  限制 dq 电压矢量长度
 *
 * 在保持方向不变的前提下把矢量缩放到允许半径内，防止过调制。
 * @param ud_ref  d 轴电压指针（输入输出）
 * @param uq_ref  q 轴电压指针（输入输出）
 * @param v_limit 电压矢量幅值上限 (V)
 */
void  program_limit_voltage_vector(float *ud_ref, float *uq_ref, float v_limit);

/* ── 电流斜坡 ── */

/**
 * @brief  清洗并限幅电流给定
 *
 * 过滤 NaN/Inf 并把电流命令限制到 ±limit_abs_a 范围。
 * @param ref_cmd     原始电流给定 (A)
 * @param limit_abs_a 电流上限 (A)
 * @return 合法电流给定
 */
float program_sanitize_current_ref_cmd(float ref_cmd, float limit_abs_a);

/**
 * @brief  对目标量施加单步斜率限制
 *
 * 限制每次控制更新允许的最大变化量，用于电流和速度参考平滑过渡。
 * @param target   目标值
 * @param state    当前状态值
 * @param max_step 单步最大变化量
 * @return 限速后的新状态值
 */
float program_apply_slew_limit_f32(float target, float state, float max_step);

/**
 * @brief  平滑更新实际生效的 d/q 轴电流给定
 *
 * 先限幅目标值，再按 150A/s 斜率限制更新实际应用到 PI 的电流参考，
 * 避免电流突变造成转矩冲击。
 * 调用时机：快环进入电流环前调用。
 */
void  program_update_applied_current_references(void);

/* ── 电流环参数计算 ── */

/**
 * @brief  由电流环带宽计算 Kp
 *
 * 公式：Kp = 2π·fc·L，基于等效电感估算。
 * @param bandwidth_hz 电流环目标带宽 (Hz)
 * @return Kp 增益
 */
float program_current_loop_kp_from_bandwidth_hz(float bandwidth_hz);

/**
 * @brief  由电流环带宽计算 Ki
 *
 * 公式：Ki = 2π·fc·R，基于等效电阻估算。
 * @param bandwidth_hz 电流环目标带宽 (Hz)
 * @return Ki 增益
 */
float program_current_loop_ki_from_bandwidth_hz(float bandwidth_hz);

#endif /* PROGRAM_CURRENT_H */
