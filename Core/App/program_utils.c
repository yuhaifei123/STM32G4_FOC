#include "program_utils.h"

/**
 * 工具函数：角度归一化到 [0, 2π)
 */
float program_wrap_angle_0_2pi(float angle)
{
    if (!isfinite(angle)) return 0.0f;   /* 过滤 NaN(非数字)/Inf(无穷大)，防角度计算溢出 */
    float turns = floorf(angle / TWO_PI);
    angle -= turns * TWO_PI;
    if (angle < 0.0f) angle += TWO_PI;
    return angle;
}

/* ── 工具函数：角度差归一化到 [-π, π) ── */
float program_wrap_delta_pm_pi(float angle)
{
    if (!isfinite(angle)) return 0.0f;
    return program_wrap_angle_0_2pi(angle + PI) - PI;
}

/**
 * 约束 float 值在指定范围内
 * @param  value       输入值
 * @param  min_value   下限
 * @param  max_value   上限
 * @return 限幅后的值 [min_value, max_value]
 */
float program_clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}
