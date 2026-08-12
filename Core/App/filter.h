#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

/** 浮点一阶低通滤波器 */
typedef struct
{
    float   alpha;       /* 滤波系数 (0~1) */
    float   value;       /* 当前滤波输出 */
    uint8_t initialized; /* 1=已完成初始化 */
} filter_lpf_f32_t;

/** Q15 定点一阶低通滤波器 */
typedef struct
{
    int32_t alpha_q15;   /* 滤波系数 (Q15, 0~32768) */
    int32_t value_q15;   /* 当前滤波输出 (Q15) */
    uint8_t initialized; /* 1=已完成初始化 */
} filter_lpf_s32_t;

void    filter_lpf_f32_init(filter_lpf_f32_t *filter, float alpha, float initial_value);
float   filter_lpf_f32_update(filter_lpf_f32_t *filter, float input);

void    filter_lpf_s32_init(filter_lpf_s32_t *filter, int32_t alpha_q15, int32_t initial_value);
int32_t filter_lpf_s32_update(filter_lpf_s32_t *filter, int32_t input);

#endif /* FILTER_H */
