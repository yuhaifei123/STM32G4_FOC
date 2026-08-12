#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <stdint.h>

#include "drv_pid.h"
#include "filter.h"
#include "foc_core.h"

/** 电机运行状态枚举 */
typedef enum
{
    MOTOR_STATE_INIT = 0,        /**< 初始化 */
    MOTOR_STATE_READY,           /**< 就绪（功率级休眠，等待启动命令） */
    MOTOR_STATE_ALIGN,           /**< 编码器零位对齐（Ud 电压锁定转子） */
    MOTOR_STATE_OPEN_LOOP,       /**< 开环拖动（积分器生成电角度） */
    MOTOR_STATE_CLOSED_LOOP,     /**< 闭环 FOC（速度环 + 电流环级联） */
    MOTOR_STATE_FAULT            /**< 故障停机（nFAULT 触发或编码器异常） */
} motor_state_id_t;

/** 电机故障类型枚举 */
typedef enum
{
    MOTOR_FAULT_NONE      = 0,  /**< 无故障 */
    MOTOR_FAULT_DRIVER    = 1,  /**< 驱动芯片故障（nFAULT 引脚拉低） */
    MOTOR_FAULT_OVERVOLTAGE = 2 /**< 母线过压 */
} motor_fault_t;

/**
 * 电机状态机对象
 * 包含所有控制参数、运行状态和 FOC 命令量
 */
typedef struct
{
    motor_state_id_t state;                     /* 当前状态 */
    uint8_t  run_request;                       /* 运行请求（0=停止，1=运行） */
    uint8_t  speed_loop_enable;                 /* 速度环使能 */
    uint8_t  current_loop_enable;               /* 电流环使能 */
    uint8_t  position_loop_enable;              /* 位置环使能 */
    uint8_t  control_angle_open_loop_enable;    /* 控制角度开环模式 */
    uint8_t  align_done;                        /* 对齐完成标志 */
    uint8_t  fault_code;                        /* 故障码 */
    uint32_t state_enter_ms;                    /* 进入当前状态的时刻 (ms) */

    float theta_open_loop;                      /* 开环积分器生成的电角度 */
    float open_loop_speed_elec;                 /* 开环电角速度 */
    float control_angle_open_loop_speed_elec;   /* 控制角度开环电角速度 */

    /* 电流/电压命令 */
    float id_ref;                               /* d 轴电流目标 */
    float iq_ref;                               /* q 轴电流目标 */
    float ud_ref;                               /* d 轴电压命令 */
    float uq_ref;                               /* q 轴电压命令 */

    /* 速度给定 */
    float speed_ref_mech_rpm;                   /* 目标转速 (rpm) */
    float speed_ref_mech_rad_s;                 /* 目标机械角速度 (rad/s) */
    float speed_ref_mech_applied_rad_s;         /* 经斜坡后的实际机械角速度 */
    float speed_meas_mech_rad_s;                /* 测量机械角速度 */
    float speed_ref_elec_rad_s;                 /* 电角速度参考 */
    float speed_meas_elec_rad_s;                /* 测量电角速度 */

    /* 速度环 PI 参数 */
    float speed_kp;
    float speed_ki;

    /* 位置环 */
    float position_ref_mech_deg;                /* 目标位置 (°) */
    float position_ref_mech_rad;                /* 目标位置 (rad) */
    float position_meas_mech_deg;               /* 测量位置 (°) */
    float position_meas_mech_rad;               /* 测量位置 (rad) */
    float position_error_mech_deg;              /* 位置误差 (°) */
    float position_error_mech_rad;              /* 位置误差 (rad) */
    float position_kp;                          /* 位置环 Kp */
    float position_ki;                          /* 位置环 Ki */
    float position_kd;                          /* 位置环 Kd */
    float position_integral_speed;              /* 位置环积分速度项 */
    float position_speed_limit_mech_rad_s;      /* 位置环速度限幅 */

    float speed_meas_lpf_cutoff_hz;             /* 速度测量 LPF 截止频率 */

    /* 速度环 PI 积分项 */
    float speed_integral_iq;                    /* 电流模式速度环积分 */
    float speed_integral_uq;                    /* 电压模式速度环积分 */

    /* 电流环 */
    float iq_limit;                             /* iq 电流上限 */
    float current_kp;                           /* 电流环 Kp */
    float current_ki;                           /* 电流环 Ki */
    float id_integral_v;                        /* d 轴积分项 */
    float iq_integral_v;                        /* q 轴积分项 */
    float voltage_limit;                        /* 电压矢量限幅值 */

    filter_lpf_f32_t speed_lpf;                 /* 速度测量 LPF */
    drv_pid_pi_t      speed_pi;                 /* 速度环 PI 占位对象 */
} motor_state_t;

/**
 * 初始化状态机对象，设置默认参数和 READY 状态
 * @param motor  状态机对象指针
 */
void motor_state_init(motor_state_t *motor);
/**
 * 状态机主任务（当前为独立框架，未被主流程调用）
 * @param motor   状态机对象指针
 * @param foc     FOC 数学核心对象
 * @param now_ms  当前系统时间 (ms)
 */
void motor_state_task(motor_state_t *motor, foc_core_t *foc, uint32_t now_ms);
/**
 * 设置运行请求标志
 * @param motor   状态机对象指针
 * @param enable  1=启动, 0=停机
 */
void motor_state_set_run_request(motor_state_t *motor, uint8_t enable);

/**
 * 设置故障码并触发故障状态
 * @param motor      状态机对象指针
 * @param fault_code  故障码（motor_fault_t 枚举值）
 */
void motor_state_set_fault(motor_state_t *motor, uint8_t fault_code);

/**
 * 清除故障码，允许从 FAULT 状态恢复
 * @param motor  状态机对象指针
 */
void motor_state_clear_fault(motor_state_t *motor);


#endif /* MOTOR_STATE_H */
