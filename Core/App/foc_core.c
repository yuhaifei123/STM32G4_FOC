#include "foc_core.h"
#include "program_utils.h"
#include <math.h>

#define FOC_TWO_PI      6.28318530718f
#define FOC_SQRT3       1.73205080757f
#define FOC_INV_SQRT3   0.57735026919f

/* ── 前置声明 ── */
static void foc_core_inv_park(const foc_dq_t *v_dq, float sin_theta, float cos_theta, foc_alpha_beta_t *v_ab);
static void foc_core_svpwm(foc_core_t *core, float u_alpha, float u_beta, float vbus);

/* 函数作用：设置当前母线电压。
 * 输入：core 为 FOC 对象，vbus 为母线电压，单位 V。
 * 输出：无返回值。
 * 调用频率：测量量更新后按节拍调用。
 * 运行内容：写入母线电压，并保证最小值不小于 1V。 */
void foc_core_set_bus_voltage(foc_core_t *core, float vbus)
{
    if (core == 0) {
        return;
    }

    core->vbus = vbus > 1.0f ? vbus : 1.0f;
}

/* 函数作用：更新电角度及其正余弦缓存。
 * 输入：core 为 FOC 对象，theta_elec 为电角度，单位 rad。
 * 输出：无返回值。
 * 调用频率：每次执行 FOC 计算时调用。
 * 运行内容：完成角度归一化，并同步计算 sin/cos 供 Park/反 Park 复用。 
 */
void foc_core_set_electrical_angle(foc_core_t *core, float theta_elec)
{
    // 空指针检查
    if (core == 0) {
        return;
    }

    // 角度归一化到 [0, 2π)
    core->theta_elec = foc_core_wrap_angle(theta_elec);
    // 预计算 sin/cos，供 Park 变换/反 Park 复用
    core->sin_theta = sinf(core->theta_elec);
    core->cos_theta = cosf(core->theta_elec);
}

/* 函数作用：把角度限制到 0~2pi。
 * 输入：theta 为任意浮点角度，单位 rad。
 * 输出：返回归一化后的角度。
 * 调用频率：每次更新电角度时调用。
 * 运行内容：通过加减 2pi 保持角度连续并落在一圈范围内。 */
static float foc_core_wrap_angle(float theta)
{
     // 角度包含的整圈数
    float turns;

    // 过滤 NaN/Inf
    if (!isfinite(theta)) {
        return 0.0f;
    }

    // 计算整圈数
    turns = floorf(theta / FOC_TWO_PI);
    // 减去整圈，余数在 [0, 2π) 附近
    theta -= turns * FOC_TWO_PI;

    // 处理负角度（floor 会向负无穷取整）
    if (theta < 0.0f) {
        // 负余数补 2π
        theta += FOC_TWO_PI;
    // 处理浮点精度导致恰好等于 2π
    } else if (theta >= FOC_TWO_PI) {
        theta -= FOC_TWO_PI;
    }

    // 归一化到 [0, 2π)
    return theta;
}

/*
 * 电压开环模式：给定 ud/uq 和电角度，一站式生成三相 PWM 占空比。
 * 内部步骤：更新 vbus → 更新 θe+sin/cos → 反Park(αβ) → SVPWM(占空比)
 * 结果写入 core->duty，上层调用 program_apply_svpwm_to_tim1(&core->duty) 下发。
 */
void foc_core_run_voltage_open_loop(foc_core_t *core,
                                    float ud,          /* d 轴电压命令 (V) */
                                    float uq,          /* q 轴电压命令 (V) */
                                    float theta_elec,  /* 电角度 (rad)，对齐时为 0 */
                                    float vbus)        /* 母线电压 (V) */
{
    if (core == 0) {
        return;
    }

    // ① 更新母线电压，内部含 ≥1V 保护
    foc_core_set_bus_voltage(core, vbus);

    // ② 归一化电角度 + 缓存 sinθ/cosθ
    foc_core_set_electrical_angle(core, theta_elec);

    // ③ 记录 dq 轴电压命令
    core->v_dq_cmd.d = ud;
    core->v_dq_cmd.q = uq;

    // ④ 反 Park 变换：dq → αβ
    foc_core_inv_park(&core->v_dq_cmd,
                      core->sin_theta, core->cos_theta,
                      &core->v_ab_cmd);

    // ⑤ SVPWM：αβ → 三相占空比，写入 core->duty
    foc_core_svpwm(core,
                   core->v_ab_cmd.alpha, core->v_ab_cmd.beta, core->vbus);
}

/*
 * 反 Park 变换：dq 旋转坐标系 → αβ 静止坐标系
 * u_alpha = ud*cosθ - uq*sinθ
 * u_beta  = ud*sinθ + uq*cosθ
 */
static void foc_core_inv_park(const foc_dq_t *v_dq,
                              float sin_theta, float cos_theta,
                              foc_alpha_beta_t *v_ab)
{
    v_ab->alpha = v_dq->d * cos_theta - v_dq->q * sin_theta;
    v_ab->beta  = v_dq->d * sin_theta + v_dq->q * cos_theta;
}

/*
 * 简易 SVPWM：αβ 电压 → 三相占空比（三次谐波注入法）
 * 占空比范围 [0, 1]，结果写入 core->duty
 */
static void foc_core_svpwm(foc_core_t *core,
                           float u_alpha, float u_beta, float vbus)
{
    // Clarke 反变换：αβ → 三相
    float va = u_alpha;
    float vb = -0.5f * u_alpha + 0.86602540378f * u_beta;
    float vc = -0.5f * u_alpha - 0.86602540378f * u_beta;

    // 三次谐波注入：找 vmax/vmin，计算偏移量
    float vmax = va;
    if (vb > vmax) {
        vmax = vb;
    }
    if (vc > vmax) {
        vmax = vc;
    }
    float vmin = va;
    if (vb < vmin) {
        vmin = vb;
    }
    if (vc < vmin) {
        vmin = vc;
    }
    float v_offset = (vmax + vmin) * 0.5f;

    va -= v_offset;
    vb -= v_offset;
    vc -= v_offset;

    // 归一化到占空比 [0, 1]，含限幅保护
    float scale = 1.0f / (vbus * 0.57735026919f);  // v_limit = vbus / √3
    core->duty.a = program_clamp_f32(0.5f + va * scale, 0.0f, 1.0f);
    core->duty.b = program_clamp_f32(0.5f + vb * scale, 0.0f, 1.0f);
    core->duty.c = program_clamp_f32(0.5f + vc * scale, 0.0f, 1.0f);
}
