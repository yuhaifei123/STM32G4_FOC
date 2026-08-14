#ifndef _DRV_PID_
#define _DRV_PID_

#include <stdint.h>

/** Q15 定点 PI 控制器对象 */
typedef struct {
    int32_t ref;            /* 参考值（目标值） */
    int32_t feedback;       /* 反馈值（实际测量值） */
    int32_t error;          /* 误差 = 参考值 - 反馈值 */
    int32_t kp_q15;         /* 比例增益（Q15 定点格式） */
    int32_t ki_q15;         /* 积分增益（Q15 定点格式） */
    int32_t p_out;          /* 比例项输出 */
    int32_t i_out;          /* 积分项输出 */
    int32_t output;         /* 控制器最终输出（已限幅） */
    int32_t i_term_q15;     /* 积分项累积值（Q15 定点格式） */
    int32_t out_min;        /* 输出下限（限幅） */
    int32_t out_max;        /* 输出上限（限幅） */
} drv_pid_pi_t;

/**
 * @brief  初始化一个 PI 控制器对象
 * @param  pid       控制器对象
 * @param  kp_q15    比例增益（Q15 定点格式）
 * @param  ki_q15    积分增益（Q15 定点格式）
 * @param  out_min   输出下限
 * @param  out_max   输出上限
 * @param  out_init  初始输出
 * @note   运行频率: 系统启动或控制器创建时调用一次
 *          运行内容: 写入参数并调用 reset 建立初始积分状态
 */
void drv_pid_pi_init(drv_pid_pi_t *pid,
                     int32_t kp_q15,
                     int32_t ki_q15,
                     int32_t out_min,
                     int32_t out_max,
                     int32_t out_init);

/**
 * @brief  复位 PI 控制器内部状态
 * @param  pid       控制器对象
 * @param  out_init  希望恢复到的初始输出
 * @note   运行频率: 模式切换、故障恢复或重新接管时按需调用
 *          运行内容: 清空误差与比例项，并把积分项预装到期望初始输出
 */
void drv_pid_pi_reset(drv_pid_pi_t *pid, int32_t out_init);

/**
 * @brief  执行一次 PI 更新
 * @param  pid       控制器对象
 * @param  ref       参考值
 * @param  feedback  反馈值（与 ref 量纲必须一致）
 * @return 本次 PI 输出，同时内部状态会回写到 pid 结构体
 * @note   运行频率: 由上层控制环按固定节拍调用
 *          运行内容: 计算误差、比例项和积分项，做抗饱和处理后输出控制量
 */
int32_t drv_pid_pi_step(drv_pid_pi_t *pid, int32_t ref, int32_t feedback);

#endif /* _DRV_PID_ */
