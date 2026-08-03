#ifndef PROGRAM_CURRENT_H
#define PROGRAM_CURRENT_H

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "stm32g4xx_hal.h"
#include <math.h>

////////////////   ADC 电流采样
/* ── 电流采样硬件参数与校准常量 ── */
/* 零偏采集样本数：1024 个 × 100μs = 约 102ms 完成校准 */
#define CURRENT_OFFSET_TARGET_SAMPLES  1024
/* 采样电阻 10mΩ，串在电机相线上 */
#define CURRENT_SHUNT_OHM              0.01f
/* INA240 电流检测放大器固定增益 20V/V */
#define CURRENT_SENSE_GAIN             20.0f
/* 电流换算：I = (ADC码值 - 零偏码值) × 3.3V/4095 / (20 × 0.01Ω) */

/**
 * 启动 ADC1 注入采样
 */
void program_start_adc1_injected(void);
#endif // !PROGRAM_CURRENT_H
