#ifndef FOC_CORE_H
#define FOC_CORE_H

/*
 * 本模块是 FOC（磁场定向控制）的数学核心，负责坐标系变换和 SVPWM 生成。
 * 不涉及任何硬件寄存器操作，纯 C 计算库。
 *
 * 信号流向：
 *   三相电流(ia,ib) ──Clarke──→ (iα,iβ) ──Park──→ (id,iq)  → PI控制
 *   PI输出(ud,uq) ──反Park──→ (vα,vβ) ──SVPWM──→ 三相占空比 → TIM1
 */

/* ── 两相静止坐标系 (α-β)，由 Clarke 变换产生 ── */
/* α 轴与 A 相重合，β 轴超前 α 90° */
typedef struct
{
    float alpha;  /* α 轴分量 */
    float beta;   /* β 轴分量 */
} foc_alpha_beta_t;

/* ── 两相旋转坐标系 (d-q)，由 Park 变换产生 ── */
/* d 轴与转子磁场平行（励磁分量），q 轴超前 d 90°（转矩分量） */
typedef struct
{
    float d;  /* d 轴分量（励磁电流 id，控制磁场强度） */
    float q;  /* q 轴分量（转矩电流 iq，控制输出扭矩） */
} foc_dq_t;

/* ── 三相 PWM 占空比，由 SVPWM 生成，取值范围 0.0 ~ 1.0 ── */
/* 0.5 = 上下桥各 50%（零矢量），0.0 = 上桥全关，1.0 = 上桥全开 */
typedef struct
{
    float duty_a;  /* A 相 (U 相) 占空比 → TIM1_CH1 */
    float duty_b;  /* B 相 (V 相) 占空比 → TIM1_CH2 */
    float duty_c;  /* C 相 (W 相) 占空比 → TIM1_CH3 */
} foc_svpwm_duty_t;

/* ── FOC 核心对象，持有所有中间变量和输出 ── */
typedef struct
{
    /* 系统状态 */
    float vbus;          /* 母线电压 (V)，用于 SVPWM 电压归一化 */
    float theta_elec;    /* 当前电角度 (rad)，已归一化到 [0, 2π) */
    float sin_theta;     /* sin(θe) 缓存，供 Park/反Park 复用 */
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

/* ── API ── */

/* 初始化 FOC 核心对象，清零所有变量，占空比默认 50%（零矢量） */
void foc_core_init(foc_core_t *core);

/* 复位输出：清零电压命令 + 三相占空比拉回 50% */
void foc_core_reset_output(foc_core_t *core);

/* 更新母线电压（最小值保护为 1V，避免除零） */
void foc_core_set_bus_voltage(foc_core_t *core, float vbus);

/* 更新电角度并同步计算 sinθ / cosθ 缓存 */
void foc_core_set_electrical_angle(foc_core_t *core, float theta_elec);

/* Clarke 变换：三相静止 (ia, ib) → 两相静止 (α, β) */
void foc_core_clarke(float ia, float ib, foc_alpha_beta_t *out);

/* Park 变换：两相静止 (α, β) → 两相旋转 (d, q) */
void foc_core_park(const foc_alpha_beta_t *ab, float sin_theta, float cos_theta, foc_dq_t *out);

/* 反 Park 变换：两相旋转 (d, q) → 两相静止 (α, β) */
void foc_core_inv_park(const foc_dq_t *dq, float sin_theta, float cos_theta, foc_alpha_beta_t *out);

/* SVPWM：两相静止电压 (vα, vβ) → 三相占空比 */
void foc_core_svpwm(foc_core_t *core, float v_alpha, float v_beta, float vbus);

/* 一键式电压开环输出：反Park + SVPWM 串行调用 */
void foc_core_run_voltage_open_loop(foc_core_t *core,
                                    float ud,
                                    float uq,
                                    float theta_elec,
                                    float vbus);

#endif /* FOC_CORE_H */
