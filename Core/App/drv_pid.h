#ifndef FOC_PID_H
#define FOC_PID_H

#include "stm32g4xx_hal.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include <math.h>
#include "program.h"
#include "foc_core.h"

/**
 * 带抗饱和的浮点 PI 控制器
 * @param ref       目标值
 * @param fb        反馈值
 * @param kp        比例增益
 * @param ki        积分增益
 * @param dt        控制周期 (s)
 * @param integral  积分项指针（跨调用保持）
 * @param lo        输出下限
 * @param hi        输出上限
 * @return 限幅后的 PI 输出
 */
float run_pi_f32(float ref, float fb, float kp, float ki,float dt, float *integral, float lo, float hi);

#endif // !FOC_PID_H
