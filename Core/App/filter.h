#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

/** 浮点一阶低通滤波器 */
typedef struct
{
    float   alpha;       /* 滤波系数 (0~1)，越大响应越快 */
    float   value;       /* 当前滤波输出 */
    uint8_t initialized; /* 1=已完成初始化 */
} filter_lpf_f32_t;

/**
 * 初始化浮点一阶低通滤波器
 * 公式：y += α × (x - y)
 * @param filter        滤波器对象指针
 * @param alpha         滤波系数 (0~1)，越小滤波越强
 * @param initial_value 初始输出值
 */
void    filter_lpf_f32_init(filter_lpf_f32_t *filter, float alpha, float initial_value);
/**
 * 更新浮点一阶低通滤波器（输入一个样本，输出滤波后的值）
 * 首次调用时自动用输入值完成初始化
 * @param filter  滤波器对象指针
 * @param input   本次输入样本
 * @return 滤波后的输出值
 */
float   filter_lpf_f32_update(filter_lpf_f32_t *filter, float input);

#endif /* FILTER_H */
