#ifndef FOC_CORE_H
#define FOC_CORE_H  

#include "stm32g4xx_hal.h"

/* ────────── 坐标系类型（须在 foc_core_t 之前定义）────────── */

/* 两相静止坐标系 (α-β)，由 Clarke 变换产生 */
typedef struct {
    float alpha;  /* α 轴分量 */
    float beta;   /* β 轴分量 */
} foc_alpha_beta_t;

/* 两相旋转坐标系 (d-q)，由 Park 变换产生
 * d 轴与转子磁场平行（励磁分量），q 轴超前 d 90°（转矩分量） */
typedef struct {
    float d;  /* d 轴分量（励磁电流 id，控制磁场强度） */
    float q;  /* q 轴分量（转矩电流 iq，控制输出扭矩） */
} foc_dq_t;

/* SVPWM 三相占空比输出 */
typedef struct {
    float a;  /* A 相占空比 (0.0 ~ 1.0) */
    float b;  /* B 相占空比 (0.0 ~ 1.0) */
    float c;  /* C 相占空比 (0.0 ~ 1.0) */
} foc_svpwm_duty_t;

/* ────────────────────────────────────────────────────────────────────────── */
/* FOC 核心对象，持有所有中间变量和输出                                    */
/* ────────────────────────────────────────────────────────────────────────── */
typedef struct
{
    /* 系统状态 */
    float vbus;          /* 母线电压 (V)，用于 SVPWM 电压归一化 */
    float theta_elec;    /* 当前电角度 (rad)，已归一化到 [0, 2π) */
    float sin_theta;     /* sin(θe) 缓存，供 Park/反Park 复用，避免重复计算 */
    float cos_theta;     /* cos(θe) 缓存 */

    /* 电流反馈：ADC → Clarke → Park */
    foc_alpha_beta_t i_ab;  /* α-β 坐标系电流测量值 */
    foc_dq_t         i_dq;  /* d-q 坐标系电流测量值 (id, iq) */

    /* 电压命令：PI 输出 → 反Park → SVPWM */
    foc_dq_t         v_dq_cmd;   /* d-q 坐标系电压命令 (ud, uq) */
    foc_alpha_beta_t v_ab_cmd;   /* α-β 坐标系电压命令 (vα, vβ) */

    /* 最终输出 */
    foc_svpwm_duty_t duty;  /* 三相 PWM 占空比，直接写入 TIM1 CCR */
} foc_core_t;

/* 函数作用：设置当前母线电压。
 * 输入：core 为 FOC 对象，vbus 为母线电压，单位 V。
 * 输出：无返回值。
 * 调用频率：测量量更新后按节拍调用。
 * 运行内容：写入母线电压，并保证最小值不小于 1V。 */
void foc_core_set_bus_voltage(foc_core_t *core, float vbus);

/* 函数作用：更新电角度及其正余弦缓存。
 * 输入：core 为 FOC 对象，theta_elec 为电角度，单位 rad。
 * 输出：无返回值。
 * 调用频率：每次执行 FOC 计算时调用。
 * 运行内容：完成角度归一化，并同步计算 sin/cos 供 Park/反 Park 复用。 
 */
void foc_core_set_electrical_angle(foc_core_t *core, float theta_elec);

/* 函数作用：把角度限制到 0~2pi。
 * 输入：theta 为任意浮点角度，单位 rad。
 * 输出：返回归一化后的角度。
 * 调用频率：每次更新电角度时调用。
 * 运行内容：通过加减 2pi 保持角度连续并落在一圈范围内。 */
static float foc_core_wrap_angle(float theta);

/*
 * 电压开环模式：给定 ud/uq 和电角度，一站式生成三相 PWM 占空比。
 * 内部步骤：更新 vbus → 更新 θe+sin/cos → 反Park(αβ) → SVPWM(占空比)
 * 结果写入 core->duty，上层调用 program_apply_svpwm_to_tim1(&core->duty) 下发。
 */
void foc_core_run_voltage_open_loop(foc_core_t *core,
                                    float ud,          /* d 轴电压命令 (V) */
                                    float uq,          /* q 轴电压命令 (V) */
                                    float theta_elec,  /* 电角度 (rad)，对齐时为 0 */
                                    float vbus) ;       /* 母线电压 (V) */

#endif // 
