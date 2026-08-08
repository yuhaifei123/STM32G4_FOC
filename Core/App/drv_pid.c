#include "drv_pid.h"

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
float run_pi_f32(float ref, float fb, float kp, float ki, float dt, float *integral, float lo, float hi)
{
    /** 计算误差 */
    float error = ref - fb;
    /** 比例项 */
    float p_out = kp * error;
    /** 积分候选值 = 上次积分 + ki × dt × error */
    float i_cand = *integral + ki * dt * error;
    /** 积分项限幅 */
    i_cand = program_clamp_f32(i_cand, lo, hi);
    /** 总输出 = P + I */
    float out = p_out + i_cand;
    /** 抗饱和：仅在不加深饱和的方向更新积分 */
    /** 抗饱和处理：输出越限时只在不加深饱和的方向更新积分 */
   if (out > hi)
    {
        /** 正向饱和：限制输出，仅负误差时更新积分（向饱和反方向） */
        out = hi;
        // 电机转太快了，需要减速
        if (error < 0.0f)
        {
            // 允许积分更新（往减小输出方向走，对退出饱和有利）
            *integral = i_cand;
        }
    }
    // error > 0（电机还没转够，还需要更大输出）
    else if (out < lo)
    {
        /** 负向饱和：限制输出，仅正误差时更新积分 */
        out = lo;
        // 跳过！积分不攒（本来就到顶了，再攒就是"欠债"）
        if (error > 0.0f)
        {
            *integral = i_cand;
        }
    }
    else
    {
        /** 未饱和：正常更新积分 */
        *integral = i_cand;
    }
    return out;
}
