#include "program_current.h"

/* ── 零偏采集状态 ── */
/* 零偏校准：在 ADC1 注入中断中累加，1024 次后取均值 */
static volatile uint32_t g_ia_offset_sum = 0;         /* A 相累加和 */
static volatile uint32_t g_ib_offset_sum = 0;         /* B 相累加和 */
static volatile uint32_t g_ic_offset_sum = 0;         /* C 相累加和 */
static volatile uint32_t g_offset_sample_count = 0;   /* 已采集样本数 */
static volatile uint8_t  g_offset_ready = 0;          /* 1=校准完成，中断中写，主循环读 */

/* 默认零偏 = 2048（12bit ADC 中点，对应 INA240 零电流输出 Vref/2 = 1.65V） */
static uint16_t g_ia_offset = 2048;
static uint16_t g_ib_offset = 2048;
static uint16_t g_ic_offset = 2048;

/**
 * 启动 ADC1 注入采样
 */
void program_start_adc1_injected(void)
{
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 &&  g_offset_ready == 0) {
        // 处理 ADC1 注入采样完成中断
        // 读取 ADC1 注入采样结果
        uint16_t ia_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint16_t ib_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        uint16_t ic_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

        // 累加到零偏和样本数
        g_ia_offset_sum += ia_raw;
        g_ib_offset_sum += ib_raw;
        g_ic_offset_sum += ic_raw;
        g_offset_sample_count++;

        // 如果样本数达到目标，则计算零偏
        if (g_offset_sample_count >= CURRENT_OFFSET_TARGET_SAMPLES) {
            g_ia_offset = g_ia_offset_sum / g_offset_sample_count;
            g_ib_offset = g_ib_offset_sum / g_offset_sample_count;
            g_ic_offset = g_ic_offset_sum / g_offset_sample_count;
            g_offset_ready = 1;
        }
    }
}
