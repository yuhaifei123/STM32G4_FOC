#include "program_svpwm.h"
#include "program.h"
#include "program_utils.h"
#include "foc_core.h"
#include "drv_pid.h"
#include <math.h>
#include "program_current.h"


#define SPEED_OBSERVER_WINDOW  20U

/* ── 来自 program_current.c 的共享变量 ── */
extern uint32_t g_speed_window_count;
extern volatile float g_speed_meas_mech;
extern uint8_t g_speed_ready;

/** 开环拖动电角速度 (rad/s) */
#define OPEN_LOOP_SPEED_ELEC  2000.0f
/** 开环拖动 q 轴电压 (V) */
#define OPEN_LOOP_UQ_V         1.00f

/** 开环电角度积分器 */
static float g_open_loop_theta = 0.0f;

/**
 * 开环拖动 SVPWM 输出
 * 积分器生成电角度 → 反Park → Clarke反 → 三次谐波注入 → 占空比 → TIM1
 */
void program_open_loop_svpwm(void)
{
    /** 电角度用积分器生成 */
    float dt = 0.0001f; /* 100us ≈ 10kHz */
    g_open_loop_theta += OPEN_LOOP_SPEED_ELEC * dt;
    g_open_loop_theta = program_wrap_angle_0_2pi(g_open_loop_theta);

    float ud = 0.0f;
    float uq = OPEN_LOOP_UQ_V;
    float theta = g_open_loop_theta;

    /** 反 Park 变换：dq → αβ */
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);
    float u_alpha = ud * cos_t - uq * sin_t;
    float u_beta = ud * sin_t + uq * cos_t;

    /** SVPWM：母线电压获取 */
    float vbus = program_clamp_f32(
        (float)g_adc2_dma_buf[0] * 3.3f / 4095.0f * (240000.0f + 10000.0f) / 10000.0f,
        1.0f, 100.0f);

    /** Clarke 反变换：αβ → 三相电压 */
    float va = u_alpha;
    float vb = -0.5f * u_alpha + 0.86602540378f * u_beta;
    float vc = -0.5f * u_alpha - 0.86602540378f * u_beta;

    /** 三次谐波注入：找 vmax/vmin，计算零序偏移 */
    float vmax = va;
    if (vb > vmax)
        vmax = vb;
    if (vc > vmax)
        vmax = vc;
    float vmin = va;
    if (vb < vmin)
        vmin = vb;
    if (vc < vmin)
        vmin = vc;
    float v_offset = (vmax + vmin) * 0.5f;
    va -= v_offset;
    vb -= v_offset;
    vc -= v_offset;

    /** 归一化到占空比 [0, 1] */
    float da = 0.5f + va / vbus;
    float db = 0.5f + vb / vbus;
    float dc = 0.5f + vc / vbus;

    foc_svpwm_duty_t duty = { .a = da, .b = db, .c = dc };
    program_apply_svpwm_to_tim1(&duty);
}

// ── 电流/速度环 PI 参数 ──
#define CURRENT_KP       5.15f
#define CURRENT_KI    9110.6f
#define SPEED_KP        0.0015f
#define SPEED_KI        0.015f
#define IQ_LIMIT_A      12.0f

static float g_id_integral    = 0.0f;
static float g_iq_integral    = 0.0f;
static float g_speed_integral = 0.0f;
static float g_speed_ref      = 100.0f;

/**
 * 闭环拖动 FOC 主逻辑
 * ①电流换算 → ②Clarke → ③电角 → ④Park → ⑤速度PI → ⑥vbus → ⑦电流PI → ⑧限幅 → ⑨反Park+SVPWM
 */
void program_closed_loop_svpwm(uint16_t ia_raw, uint16_t ib_raw,
                                uint16_t ia_offset, uint16_t ib_offset,
                                float elec_offset)
{
    // ① 电流换算
    float ia = ((float)ia_raw - (float)ia_offset) * 3.3f / 4095.0f / (20.0f * 0.01f);
    float ib = ((float)ib_raw - (float)ib_offset) * 3.3f / 4095.0f / (20.0f * 0.01f);

    // ② Clarke：ia/ib → αβ
    float i_alpha = ia;
    float i_beta = (ia + 2.0f * ib) / 1.73205080757f;

    // ③ 获取电角度
    float mech_angle = g_ma600a.angle_rad;
    float elec_angle = program_wrap_angle_0_2pi(mech_angle * 14.0f - elec_offset);

    // ④ Park：αβ → dq
    float sin_t = sinf(elec_angle);
    float cos_t = cosf(elec_angle);
    float id =  i_alpha * cos_t + i_beta * sin_t;
    float iq = -i_alpha * sin_t + i_beta * cos_t;

    // ⑤ 速度环 PI → iq_ref
    float iq_ref = 0.0f;
    if (g_speed_ready && (g_speed_window_count == 0)) {
        float speed_dt = 0.0001f * (float)SPEED_OBSERVER_WINDOW;
        iq_ref = run_pi_f32(g_speed_ref, g_speed_meas_mech,
                            SPEED_KP, SPEED_KI, speed_dt,
                            &g_speed_integral, -IQ_LIMIT_A, IQ_LIMIT_A);
    }

    // ⑥ 母线电压 & 限幅
    float vbus = program_clamp_f32(
        (float)g_adc2_dma_buf[0] * 3.3f / 4095.0f * 25.0f, 1.0f, 100.0f);
    float v_limit = vbus * 0.577f;

    // ⑦ 电流环 PI：id→0, iq→iq_ref
    float ud = run_pi_f32(0.0f, id, CURRENT_KP, CURRENT_KI, 0.0001f,
                          &g_id_integral, -v_limit, v_limit);
    float uq = run_pi_f32(iq_ref, iq, CURRENT_KP, CURRENT_KI, 0.0001f,
                          &g_iq_integral, -v_limit, v_limit);

    // ⑧ 电压矢量限幅
    float v_mag = sqrtf(ud * ud + uq * uq);
    if (v_mag > v_limit && v_mag > 0.0f) {
        float scale = v_limit / v_mag;
        ud *= scale; uq *= scale;
    }

    // ⑨ 反Park + SVPWM
    foc_core_run_voltage_open_loop(&g_foc, ud, uq, elec_angle, vbus);
    program_apply_svpwm_to_tim1(&g_foc.duty);
    HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin, GPIO_PIN_SET);
}
