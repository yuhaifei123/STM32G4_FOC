#ifndef PROGRAM_UTILS_H
#define PROGRAM_UTILS_H

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "stm32g4xx_hal.h"
#include <math.h>

/* ── 常量 ── */
#define ADC2_DMA_LEN        2               /* ADC2 DMA 缓冲长度（2 通道） */
#define ADC_REF_V           3.30f           /* ADC 基准电压 (V) */
#define ADC_FULL_SCALE      4095.0f         /* 12bit ADC 满量程 */
#define VBUS_R_UP           240000.0f       /* 母线分压上电阻 240kΩ */
#define VBUS_R_DOWN         10000.0f        /* 母线分压下电阻 10kΩ（分压比 25:1） */
#define TWO_PI              6.28318530718f   /* 2π */
#define PI                  3.14159265359f   /* π */

/* 角度归一化到 [0, 2π) */
float program_wrap_angle_0_2pi(float angle);

/* 角度差归一化到 [-π, π) */
float program_wrap_delta_pm_pi(float angle);

/* 约束 float 值在指定范围内 */
float program_clamp_f32(float value, float min_value, float max_value);

#endif
