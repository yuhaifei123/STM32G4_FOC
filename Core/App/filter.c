#include "filter.h"

#define FILTER_Q15_ONE  32768L

/* 限制浮点滤波系数范围 [0, 1] */
static float filter_clamp_alpha(float alpha)
{
    if (alpha < 0.0f) return 0.0f;
    if (alpha > 1.0f) return 1.0f;
    return alpha;
}

/* 限制 Q15 滤波系数范围 [0, 32768] */
static int32_t filter_clamp_q15(int32_t alpha_q15)
{
    if (alpha_q15 < 0) return 0;
    if (alpha_q15 > FILTER_Q15_ONE) return FILTER_Q15_ONE;
    return alpha_q15;
}

/**
 * @brief  初始化浮点一阶低通滤波器
 * @param filter        滤波器对象指针
 * @param alpha         滤波系数 (0~1)，越大响应越快
 * @param initial_value 初始输出值
 */
void filter_lpf_f32_init(filter_lpf_f32_t *filter, float alpha, float initial_value)
{
    if (filter == 0) return;
    filter->alpha       = filter_clamp_alpha(alpha);
    filter->value       = initial_value;
    filter->initialized = 1U;
}

/**
 * @brief  执行一次浮点一阶低通更新 y += α·(x - y)
 * @param filter 滤波器对象指针
 * @param input  当前输入值
 * @return 本次滤波输出
 */
float filter_lpf_f32_update(filter_lpf_f32_t *filter, float input)
{
    if (filter == 0) return input;
    if (filter->initialized == 0U) {
        filter_lpf_f32_init(filter, filter->alpha, input);
        return input;
    }
    filter->value += filter->alpha * (input - filter->value);
    return filter->value;
}

/**
 * @brief  初始化 Q15 定点一阶低通滤波器
 * @param filter        滤波器对象指针
 * @param alpha_q15     Q15 滤波系数 (0~32768)
 * @param initial_value 初始整数值
 */
void filter_lpf_s32_init(filter_lpf_s32_t *filter, int32_t alpha_q15, int32_t initial_value)
{
    if (filter == 0) return;
    filter->alpha_q15   = filter_clamp_q15(alpha_q15);
    filter->value_q15   = initial_value << 15;
    filter->initialized = 1U;
}

/**
 * @brief  执行一次 Q15 定点一阶低通更新
 * @param filter 滤波器对象指针
 * @param input  当前整数输入值
 * @return 本次整数滤波输出
 */
int32_t filter_lpf_s32_update(filter_lpf_s32_t *filter, int32_t input)
{
    if (filter == 0) return input;
    if (filter->initialized == 0U) {
        filter_lpf_s32_init(filter, filter->alpha_q15, input);
        return input;
    }
    int32_t input_q15 = input << 15;
    int32_t delta_q15 = input_q15 - filter->value_q15;
    filter->value_q15 += (int32_t)(((int64_t)filter->alpha_q15 * delta_q15) >> 15);
    return filter->value_q15 >> 15;
}
