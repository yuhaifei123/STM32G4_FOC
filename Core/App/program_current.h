#ifndef PROGRAM_CURRENT_H
#define PROGRAM_CURRENT_H

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include "program.h"
#include "foc_core.h"

////////////////   ADC 电流采样
/* ── 电流采样硬件参数与校准常量 ── */
/* 零偏采集样本数：1024 个 × 100μs = 约 102ms 完成校准 */
#define CURRENT_OFFSET_TARGET_SAMPLES  1024
/* 采样电阻 10mΩ，串在电机相线上 */
#define CURRENT_SHUNT_OHM              0.01f
/* INA240 电流检测放大器固定增益 20V/V */
#define CURRENT_SENSE_GAIN             20.0f
/* 电流换算：I = (ADC码值 - 零偏码值) × 3.3V/4095 / (20 × 0.01Ω) */

/* Ud=1.8V 直流电压固定到 d 轴，将转子磁吸锁定到电角度 0° 位置 */
#define ALIGN_UD_V          1.80f
/* 对齐保持 8000 ticks × 100μs = 800ms，确保转子停止震荡后采样编码器偏置 */
#define ALIGN_HOLD_TICKS    8000

/**
 * 启动 ADC1 注入采样
 */
void program_start_adc1_injected(void);


/**
 * 快环处理（10kHz ISR）
 * 流程：零偏校准 → 编码器读取 → 对齐 → 开环/闭环
 * @param ia_raw  A 相 ADC 原始值
 * @param ib_raw  B 相 ADC 原始值
 * @param ic_raw  C 相 ADC 原始值
 */
void program_current_fast_loop(uint16_t ia_raw, uint16_t ib_raw, uint16_t ic_raw);

/**
 * 速度观测：基于编码器机械角差分
 * 每 SPEED_OBSERVER_WINDOW 拍（2ms）计算一次平均速度
 * @param mech_angle  编码器机械角 (rad)，由 g_ma600a.angle_rad 传入
 */
void program_update_speed_measurement(float mech_angle);

// ADC 原始值 → 相电流(A)
float convert_current(uint16_t raw, uint16_t offset);
#endif // !PROGRAM_CURRENT_H
