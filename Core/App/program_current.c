#include "program_current.h"
#include "program.h"
#include "program_svpwm.h"
#include "main.h"
#include "drv_pid.h"

/** 速度观测窗口拍数（20拍 × 100μs = 2ms） */
#define SPEED_OBSERVER_WINDOW  20U

/** 速度观测：当前窗口已累积拍数 */
uint32_t g_speed_window_count = 0;
/** 速度观测：机械角速度 (rad/s)，ISR 中更新 */
volatile float g_speed_meas_mech = 0.0f;
/** 速度观测：首次角度已记录标志 */
uint8_t g_speed_ready = 0;

/* ── 零偏采集状态 ── */
static volatile uint32_t g_ia_offset_sum = 0;       /* A 相累加和 */
static volatile uint32_t g_ib_offset_sum = 0;       /* B 相累加和 */
static volatile uint32_t g_ic_offset_sum = 0;       /* C 相累加和 */
static volatile uint32_t g_offset_sample_count = 0; /* 已采集样本数 */
static volatile uint8_t g_offset_ready = 0;         /* 1=校准完成 */

/** 默认零偏 = 2048（12bit ADC 中点，对应 INA240 零电流输出 Vref/2 = 1.65V） */
static volatile uint16_t g_ia_offset = 2048;
static volatile uint16_t g_ib_offset = 2048;
static volatile uint16_t g_ic_offset = 2048;

/** 编码器零位对齐：施加 Ud 电压将转子锁到 θe=0 */
static volatile uint8_t g_align_done = 0;     /* 1=对齐完成 */
static volatile uint32_t g_align_counter = 0; /* 对齐计时器，每 100μs 加 1 */
/** 电角偏置：θe = 编码器电角 - offset */
static volatile float g_elec_offset_rad = 0.0f;

void program_start_adc1_injected(void)
{
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/**
 * ADC1 注入采样完成中断回调（薄封装）
 * 读取三路 ADC 后转调 program_current_fast_loop
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
        return;

    uint16_t ia_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint16_t ib_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    uint16_t ic_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

    program_current_fast_loop(ia_raw, ib_raw, ic_raw);
}

/**
 * 快环处理（10kHz）
 * 流程：零偏校准 → 编码器 → 速度观测 → 对齐 → 开环拖动
 * @param ia_raw  A 相 ADC 原始值
 * @param ib_raw  B 相 ADC 原始值
 * @param ic_raw  C 相 ADC 原始值
 */
static void program_current_fast_loop(uint16_t ia_raw, uint16_t ib_raw, uint16_t ic_raw)
{
    /** ── 零偏校准：采集 1024 次取均值 ── */
    if (!g_offset_ready)
    {
        g_ia_offset_sum += ia_raw;
        g_ib_offset_sum += ib_raw;
        g_ic_offset_sum += ic_raw;
        g_offset_sample_count++;
        if (g_offset_sample_count >= CURRENT_OFFSET_TARGET_SAMPLES)
        {
            g_ia_offset = (uint16_t)(g_ia_offset_sum / CURRENT_OFFSET_TARGET_SAMPLES);
            g_ib_offset = (uint16_t)(g_ib_offset_sum / CURRENT_OFFSET_TARGET_SAMPLES);
            g_ic_offset = (uint16_t)(g_ic_offset_sum / CURRENT_OFFSET_TARGET_SAMPLES);
            g_offset_ready = 1;
        }
        return; /* 校准期间不执行对齐 */
    }

    /** 读取编码器角度（每拍都要读，10kHz） */
    ma600a_read_angle(&g_ma600a);

    /** 速度观测 */
    program_update_speed_measurement(g_ma600a.angle_rad);

    /** ── 编码器零位对齐：Ud=1.8V 将转子锁到 θe=0 ── */
    if (!g_align_done)
    {
        /** ① 电压开环输出 Ud 锁定转子 */
        foc_core_run_voltage_open_loop(&g_foc, ALIGN_UD_V, 0.0f, 0.0f, g_program_telemetry.vbus);
        /** ② 下发占空比到 TIM1 */
        program_apply_svpwm_to_tim1(&g_foc.duty);
        /** ③ 使能功率级（N_SLEEP 拉高） */
        HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin, GPIO_PIN_SET);

        /** ④ 对齐计时 */
        g_align_counter++;
        if (g_align_counter >= ALIGN_HOLD_TICKS)
        {
            /** ⑤ 转子已锁定，记录当前编码器电角作为偏置 */
            float raw_elec = g_ma600a.angle_rad * 14.0f; /* 机械角 → 电角（14 对极） */
            g_elec_offset_rad = program_wrap_angle_0_2pi(raw_elec);
            g_align_done = 1;
        }
        return;
    }

    /** 对齐完成后执行开环拖动 */
    if (g_align_done)
    {
        // 开环拖动
       // program_open_loop_svpwm();

        // 闭环拖动
        program_closed_loop_svpwm(ia_raw, ib_raw, g_ia_offset, g_ib_offset, g_elec_offset_rad);
    }
}

/** 速度观测：上一次机械角 */
static float g_prev_mech_angle = 0.0f;
/** 速度观测：连续累加机械角（无 0/360 跳变） */
static float g_continuous_mech_rad = 0.0f;
/** 速度观测：窗口起点角度 */
static float g_speed_window_start = 0.0f;

/**
 * 速度观测：基于编码器差分
 * 每 SPEED_OBSERVER_WINDOW 拍计算一次窗口内平均速度
 * @param mech_angle  编码器机械角 (rad)
 */
void program_update_speed_measurement(float mech_angle)
{
    /** 首次调用：初始化基准角度 */
    if (!g_speed_ready)
    {
        g_prev_mech_angle = mech_angle;
        g_continuous_mech_rad = mech_angle;
        g_speed_window_start = mech_angle;
        g_speed_window_count = 0;
        g_speed_ready = 1;
        return;
    }

    /** 计算本拍角差（处理 0°/360° 跳变） */
    float delta = program_wrap_delta_pm_pi(mech_angle - g_prev_mech_angle);
    g_prev_mech_angle = mech_angle;
    g_continuous_mech_rad += delta; /* 连续累加（无跳变） */

    /** 窗口累积 */
    g_speed_window_count++;
    if (g_speed_window_count >= SPEED_OBSERVER_WINDOW)
    {
        /** 满 20 拍（2ms）：计算窗口内平均速度 */
        float dt = 0.0001f * (float)g_speed_window_count;
        g_speed_meas_mech = (g_continuous_mech_rad - g_speed_window_start) / dt;
        g_speed_window_start = g_continuous_mech_rad;
        g_speed_window_count = 0; /* 开启下一窗口 */
    }
}

/**
 * ADC 原始值 → 相电流(A)
 * 公式：I = (raw - offset) × 3.3V / 4095 / (20 × 0.01Ω)
 * @param raw     ADC 原始码值
 * @param offset  零偏码值（约 2048，对应 INA240 Vref/2 = 1.65V）
 * @return 相电流 (A)
 */
float convert_current(uint16_t raw, uint16_t offset)
{
    return ((float)raw - (float)offset) * 3.3f / 4095.0f / (20.0f * 0.01f);
}
