#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include <math.h>

/* ── 电机本体参数（硬件属性，与控制程序配置分离管理） ── */
/** 电机极对数 */
#define MOTOR_POLE_PAIRS         14.0f
/** 减速比 */
#define MOTOR_GEAR_RATIO         8.0f
/** 编码器安装位置：0=电机轴，1=输出轴 */
#define MOTOR_ENCODER_ON_OUTPUT_SHAFT 0U
/** 编码器方向符号（±1.0） */
#define MOTOR_ENCODER_DIRECTION_SIGN (-1.0f)
/** 2π (rad)，角度圈数换算 */
#define MOTOR_TWO_PI             6.28318530718f

/* 函数作用：把角度归一化到 0~2pi。*/
static inline float motor_params_wrap_angle_rad(float angle_rad)
{
    float turns;
    if (!isfinite(angle_rad)) return 0.0f;
    turns = floorf(angle_rad / MOTOR_TWO_PI);
    angle_rad -= turns * MOTOR_TWO_PI;
    if (angle_rad < 0.0f) angle_rad += MOTOR_TWO_PI;
    else if (angle_rad >= MOTOR_TWO_PI) angle_rad -= MOTOR_TWO_PI;
    return angle_rad;
}

/* 函数作用：转子机械角 → 电角（×极对数后归一化）。*/
static inline float motor_params_rotor_mech_to_elec_rad(float rotor_mech_angle_rad)
{
    return motor_params_wrap_angle_rad(rotor_mech_angle_rad * MOTOR_POLE_PAIRS);
}

/* 函数作用：编码器机械角 → 转子机械角（方向 + 减速比）。*/
static inline float motor_params_encoder_mech_to_rotor_mech_rad(float encoder_mech_angle_rad)
{
#if MOTOR_ENCODER_ON_OUTPUT_SHAFT
    return encoder_mech_angle_rad * MOTOR_ENCODER_DIRECTION_SIGN * MOTOR_GEAR_RATIO;
#else
    return encoder_mech_angle_rad * MOTOR_ENCODER_DIRECTION_SIGN;
#endif
}

#endif /* MOTOR_PARAMS_H */
