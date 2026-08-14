#include "filter.h"

/* 限制浮点滤波系数范围 [0, 1] */
static float filter_clamp_alpha(float alpha)
{
    if (alpha < 0.0f) return 0.0f;
    if (alpha > 1.0f) return 1.0f;
    return alpha;
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
