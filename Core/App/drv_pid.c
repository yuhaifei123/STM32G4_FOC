#include "drv_pid.h"

#define DRV_PID_Q15_SHIFT    15
#define DRV_PID_Q15_HALF     (1L << (DRV_PID_Q15_SHIFT - 1))

/**
 * @brief  限制 32 位有符号数范围
 * @param  value      待限制值
 * @param  min_value  下限
 * @param  max_value  上限
 * @return 限幅后的结果
 * @note   运行频率: PI 每次运行时间接调用
 *          运行内容: 给积分项和最终输出做统一限幅
 */
static int32_t drv_pid_clamp_s32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * @brief  把整数提升到 Q15 定点格式
 * @param  value  普通整数值
 * @return Q15 格式结果
 * @note   运行频率: 初始化和积分计算时按需调用
 *          运行内容: 通过左移 15 位完成格式转换
 */
static int32_t drv_pid_s32_to_q15(int32_t value)
{
    return value << DRV_PID_Q15_SHIFT;
}

/**
 * @brief  把 Q15 定点数四舍五入还原成整数
 * @param  value_q15  Q15 定点值
 * @return 整数结果
 * @note   运行频率: PI 每次更新时按需调用
 *          运行内容: 对正负数分别做对称四舍五入，减小量化误差
 */
static int32_t drv_pid_q15_to_s32_round(int32_t value_q15)
{
    if (value_q15 >= 0) {
        return (value_q15 + DRV_PID_Q15_HALF) >> DRV_PID_Q15_SHIFT;
    }

    return -(((-value_q15) + DRV_PID_Q15_HALF) >> DRV_PID_Q15_SHIFT);
}

/**
 * @brief  初始化一个 PI 控制器对象
 * @param  pid      控制器对象
 * @param  kp_q15   比例增益 (Q15)
 * @param  ki_q15   积分增益 (Q15)
 * @param  out_min  输出下限
 * @param  out_max  输出上限
 * @param  out_init 初始输出
 * @note   运行频率: 系统启动或控制器创建时调用一次
 *          运行内容: 写入参数并调用 reset 建立初始积分状态
 *          输入输出量纲必须在上层先统一好
 */
void drv_pid_pi_init(drv_pid_pi_t *pid,
                     int32_t kp_q15,
                     int32_t ki_q15,
                     int32_t out_min,
                     int32_t out_max,
                     int32_t out_init)
{
    if (pid == 0) {
        return;
    }

    pid->kp_q15 = kp_q15;
    pid->ki_q15 = ki_q15;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->ref = 0;
    pid->feedback = 0;
    pid->error = 0;
    pid->p_out = 0;

    drv_pid_pi_reset(pid, out_init);
}

/**
 * @brief  复位 PI 控制器内部状态
 * @param  pid       控制器对象
 * @param  out_init  希望恢复到的初始输出
 * @note   运行频率: 模式切换、故障恢复或重新接管时按需调用
 *          运行内容: 清空误差与比例项，并把积分项预装到期望初始输出
 */
void drv_pid_pi_reset(drv_pid_pi_t *pid, int32_t out_init)
{
    if (pid == 0) {
        return;
    }

    out_init = drv_pid_clamp_s32(out_init, pid->out_min, pid->out_max);
    pid->ref = 0;
    pid->feedback = 0;
    pid->error = 0;
    pid->p_out = 0;
    pid->output = out_init;
    pid->i_out = out_init;
    pid->i_term_q15 = drv_pid_s32_to_q15(out_init);
}

/**
 * @brief  执行一步 PI 运算
 * @param  pid       控制器对象
 * @param  ref       本次参考值（目标值）
 * @param  feedback  本次反馈值（实际测量值）
 * @return 限幅后的控制器输出
 * @note   运行频率: 每个控制周期调用一次（如 10kHz 电流环）
 *          运行内容: 计算误差 → 比例项 → 积分候选（限幅）→ 总输出
 *          带抗饱和处理：输出越限时只在不加深饱和的方向更新积分
 */
int32_t drv_pid_pi_step(drv_pid_pi_t *pid, int32_t ref, int32_t feedback)
{
    int32_t p_term_q15, i_candidate_q15, out_q15, out_s32;

    if (pid == 0) {
        return 0;
    }

    /** 记录本次输入并计算误差 */
    pid->ref = ref;
    pid->feedback = feedback;
    pid->error = ref - feedback;

    /** 比例项 = error × kp（Q15 域） */
    p_term_q15 = pid->error * pid->kp_q15;
    /** 积分候选 = 上次积分 + error × ki（Q15 域，先限幅） */
    i_candidate_q15 = pid->i_term_q15 + (pid->error * pid->ki_q15);
    i_candidate_q15 = drv_pid_clamp_s32(i_candidate_q15,
        drv_pid_s32_to_q15(pid->out_min), drv_pid_s32_to_q15(pid->out_max));

    /** 总输出 = P + I，四舍五入还原 */
    out_q15 = p_term_q15 + i_candidate_q15;
    out_s32 = drv_pid_q15_to_s32_round(out_q15);

    /** 抗饱和：输出越限时只在不加深饱和的方向更新积分 */
    if (out_s32 > pid->out_max) {
        out_s32 = pid->out_max;
        if (pid->error < 0) pid->i_term_q15 = i_candidate_q15;
    } else if (out_s32 < pid->out_min) {
        out_s32 = pid->out_min;
        if (pid->error > 0) pid->i_term_q15 = i_candidate_q15;
    } else {
        pid->i_term_q15 = i_candidate_q15;
    }

    /** 回写诊断字段 */
    pid->p_out = drv_pid_q15_to_s32_round(p_term_q15);
    pid->i_out = drv_pid_q15_to_s32_round(pid->i_term_q15);
    pid->output = out_s32;

    return out_s32;
}

//
// End of File
//
