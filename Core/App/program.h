#ifndef PROGRAM_H
#define PROGRAM_H

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "motor_state.h"
#include "foc_core.h"
#include "motor_params.h"

/* ── 系统级宏定义（program.c / program_current.c / program_svpwm.c 共用）── */
/** 圆周率 π */
#define PROGRAM_PI                            3.14159265359f
/** 快环频率 (Hz) = 10kHz */
#define PROGRAM_FAST_LOOP_HZ                  10000.0f
/** 快环周期 (s) = 100μs */
#define PROGRAM_FAST_LOOP_DT_S                (1.0f / PROGRAM_FAST_LOOP_HZ)
/** 快环周期 (μs) = 100μs，DWT 超时判定用 */
#define PROGRAM_FAST_LOOP_PERIOD_US           (1000000.0f / PROGRAM_FAST_LOOP_HZ)
/** 速度参考斜坡速率 (rad/s?)，防止速度指令突变 */
#define PROGRAM_SPEED_REF_RAMP_RAD_S2         100.0f
/** 电流参考斜坡速率 (A/s)，防止电流指令突变造成转矩冲击 */
#define PROGRAM_CURRENT_REF_RAMP_A_PER_S      150.0f

/* ── ADC 硬件参数 ── */
/** ADC 参考电压 (V) */
#define PROGRAM_ADC_REF_V                     3.3f
/** ADC 满量程码值 (12bit) */
#define PROGRAM_ADC_FULL_SCALE_COUNTS         4095.0f
/** 采样电阻阻值 (Ω)，串在电机相线上 */
#define PROGRAM_SHUNT_RESISTOR_OHM            0.01f
/** INA240 电流检测放大器固定增益 (V/V) */
#define PROGRAM_CURRENT_SENSE_GAIN            20.0f
/** 母线分压电阻上臂 (Ω) */
#define PROGRAM_VBUS_R_UP_OHM                 240000.0f
/** 母线分压电阻下臂 (Ω) */
#define PROGRAM_VBUS_R_DOWN_OHM               10000.0f

/* ── 位置环 hold/creep 阈值 ── */
/** 位置 hold：进入保持的误差阈值 (rad) ≈ 1.2° */
#define PROGRAM_POSITION_HOLD_ERR_RAD         0.021f
/** 位置 hold：退出保持的误差阈值 (rad) ≈ 1.8°（滞回） */
#define PROGRAM_POSITION_HOLD_RELEASE_ERR_RAD 0.031f
/** 位置 hold：保持时允许的最大输出速度 (rad/s) */
#define PROGRAM_POSITION_HOLD_SPEED_MECH_RAD_S 0.50f
/** 位置 hold：连续超阈值周期数才释放（防抖） */
#define PROGRAM_POSITION_HOLD_RELEASE_CONFIRM_CYCLES 12U
/** 位置 creep：启动蠕动的误差阈值 (rad) ≈ 2.6° */
#define PROGRAM_POSITION_CREEP_ENABLE_ERR_RAD 0.045f
/** 位置 creep：蠕动补偿速度 (rad/s)，克服摩擦死区 */
#define PROGRAM_POSITION_CREEP_SPEED_MECH_RAD_S 0.020f

/* ── 电流符号（硬件接线方向校正） ── */
/** A 相电流符号（±1，硬件接线方向校正） */
#define PROGRAM_CURRENT_SIGN_IA               (1.0f)
/** B 相电流符号 */
#define PROGRAM_CURRENT_SIGN_IB               (1.0f)
/** C 相电流符号 */
#define PROGRAM_CURRENT_SIGN_IC               (1.0f)

/* ── 编码器观测与量化保护 ── */
/** 连续角重归一化阈值 (rad) = 32 圈，防止浮点精度丢失 */
#define PROGRAM_ENCODER_OBSERVER_RENORM_RAD   (32.0f * MOTOR_TWO_PI)
/** 编码器 1 LSB 对应的机械角分辨率 (rad) */
#define PROGRAM_ENCODER_LSB_RAD               (MOTOR_TWO_PI / 65536.0f)
/** 速度零位保持：量化噪声倍数阈值 */
#define PROGRAM_SPEED_MEAS_ZERO_HOLD_SCALE    8.0f
/** 速度零位保持：最小机械转速死区 (rad/s)，低于此值强制归零 */
#define PROGRAM_SPEED_MEAS_ZERO_HOLD_MIN_MECH_RAD_S 0.35f

/*
 * 程序层遥测结构体：集中存放所有调试和监控变量。
 * 快环中断写入（volatile），慢环/调试器只读。
 */
typedef struct
{
    /* ── ADC 原始值 ── */
    uint16_t ia_raw;                        /* A 相电流 ADC 原始码值 */
    uint16_t ib_raw;                        /* B 相电流 ADC 原始码值 */
    uint16_t ic_raw;                        /* C 相电流 ADC 原始码值 */
    uint16_t vbus_raw;                      /* 母线电压 ADC 原始码值 */
    uint16_t ntc_raw;                       /* NTC 温度 ADC 原始码值 */
    uint16_t ma600a_angle_raw;              /* 编码器原始角度码 0~16383 */

    /* ── 零偏校准 ── */
    uint16_t ia_offset_raw;                 /* A 相零电流偏置码值（默认2048） */
    uint16_t ib_offset_raw;                 /* B 相零电流偏置码值 */
    uint16_t ic_offset_raw;                 /* C 相零电流偏置码值 */
    uint16_t current_offset_sample_count;   /* 校准已采集样本数 */
    uint8_t  current_offset_ready;          /* 1=零偏校准完成 */

    /* ── 运行状态标志 ── */
    uint8_t pwm_enable_cmd;                 /* PWM 使能命令（=run_request） */
    uint8_t power_stage_enabled;            /* 功率级是否已使能 */
    uint8_t control_state;                  /* 当前状态机状态枚举值 */
    uint8_t encoder_align_done;             /* 1=编码器零位对齐完成 */
    uint8_t speed_loop_ready;               /* 1=速度观测器就绪 */
    uint8_t current_loop_enable;            /* 电流环使能标志 */
    uint8_t position_loop_enable;           /* 位置环使能标志 */
    uint8_t control_angle_open_loop_enable; /* 控制角度开环模式使能 */
    uint8_t driver_fault_active;            /* 1=驱动器 nFAULT 引脚拉低 */
    uint8_t ma600a_angle_valid;             /* 1=编码器当前数据有效 */
    uint8_t ma600a_consecutive_bad_count;   /* 编码器连续异常样本数 */
    uint8_t fast_loop_overrun;              /* 1=当前快环超时 */

    /* ── 快环性能统计 ── */
    uint32_t fast_loop_overrun_count;       /* 累计超时次数 */
    uint32_t fast_loop_cycles;              /* 当前快环耗时（CPU 周期） */
    uint32_t fast_loop_cycles_max;          /* 历史最大耗时 */

    /* ── 编码器统计 ── */
    uint32_t ma600a_sample_counter;         /* 累计采样次数 */
    uint32_t ma600a_reject_count;           /* 跳变过大被拒绝次数 */
    uint32_t ma600a_comm_error_count;       /* SPI 通信失败次数 */
    uint32_t speed_observer_window_samples; /* 速度观测窗口样本数 */

    /* ── 电流反馈 (A) ── */
    float ia;                               /* A 相电流 */
    float ib;                               /* B 相电流 */
    float ic;                               /* C 相电流（由 ia+ib+ic=0 推算） */
    float ic_meas;                          /* C 相电流实测值（单独校准用） */
    float i_abc_sum;                        /* 三相电流和（应≈0，用于诊断） */
    float id;                               /* d 轴电流反馈 */
    float iq;                               /* q 轴电流反馈 */

    /* ── 电压与角度 ── */
    float theta_elec;                       /* 控制电角度 (rad) */
    float duty_a;                           /* A 相占空比 0~1 */
    float duty_b;                           /* B 相占空比 */
    float duty_c;                           /* C 相占空比 */
    float vbus;                             /* 滤波后母线电压 (V) */
    float ma600a_angle_deg;                 /* 编码器角度 (°) */
    float ma600a_angle_rad;                 /* 编码器角度 (rad) */

    /* ── 开环控制 ── */
    float theta_open_loop;                  /* 开环积分器生成的电角度 */
    float control_angle_open_loop_speed_elec; /* 开环角速度设定 */

    /* ── 电流/电压命令 ── */
    float id_ref_cmd;                       /* id 目标命令 */
    float iq_ref_cmd;                       /* iq 目标命令 */
    float id_ref_applied_cmd;               /* 经斜坡后的实际 id 给定 */
    float iq_ref_applied_cmd;               /* 经斜坡后的实际 iq 给定 */
    float ud_ref_cmd;                       /* ud 电压命令 */
    float uq_ref_cmd;                       /* uq 电压命令 */
    float open_loop_speed_elec;             /* 开环电角速度 */

    /* ── 速度测量与控制 (rad/s, rpm) ── */
    float speed_ref_mech_rad_s;             /* 目标机械角速度 */
    float speed_ref_mech_applied_rad_s;     /* 经斜坡后的实际机械角速度 */
    float speed_meas_raw_mech_rad_s;        /* 原始测速（未滤波） */
    float speed_meas_mech_rad_s;            /* 滤波后机械角速度 */
    float speed_ref_mech_rpm;               /* 目标转速 (rpm) */
    float speed_ref_mech_applied_rpm;       /* 斜坡后转速 (rpm) */
    float speed_meas_raw_mech_rpm;          /* 原始测速 (rpm) */
    float speed_meas_mech_rpm;              /* 滤波后转速 (rpm) */
    float speed_error_mech_rad_s;           /* 速度误差 */

    /* ── 位置环 ── */
    float position_ref_mech_rad;            /* 目标位置 (rad) */
    float position_meas_mech_rad;           /* 测量位置 (rad) */
    float position_error_mech_rad;          /* 位置误差 (rad) */
    float position_ref_mech_deg;            /* 目标位置 (°) */
    float position_meas_mech_deg;           /* 测量位置 (°) */
    float position_error_mech_deg;          /* 位置误差 (°) */
    float position_kd;                      /* 位置环 D 增益 */

    /* ── 其他参数 ── */
    float speed_loop_dt_s;                  /* 速度环实际执行周期 */
    float speed_meas_lpf_cutoff_hz;         /* 速度测量 LPF 截止频率 */
    float speed_ref_elec_rad_s;             /* 电角速度参考 */
    float speed_meas_elec_rad_s;            /* 电角速度测量 */
    float uq_limit_v;                       /* uq 电压上限 */
    float iq_limit_a;                       /* iq 电流上限 */
    float voltage_limit_v;                  /* 电压矢量幅值上限 */
    float encoder_elec_offset_rad;          /* 编码器电角偏置 */

    /* ── 快环耗时 ── */
    float fast_loop_time_us;                /* 当前快环耗时 (μs) */
    float fast_loop_time_max_us;            /* 历史最大耗时 (μs) */
    float fast_loop_period_us;              /* 快环周期 (μs，应为100) */
} program_telemetry_t;

/** 调试 PWM 测试参数 */
typedef struct
{
    uint8_t enable;
    float   duty_a;
    float   duty_b;
    float   duty_c;
} program_debug_pwm_test_t;

/* ── 全局对象 extern 声明 ── */
extern volatile program_telemetry_t      g_program_telemetry;
extern volatile program_debug_pwm_test_t g_program_debug_pwm_test;
extern motor_state_t g_motor;
extern foc_core_t    g_foc;

/* ── 公共接口 ── */
void program_init(void);
void program_task(void);
void program_tim_period_elapsed_callback(TIM_HandleTypeDef *htim);
void program_adc_conv_cplt_callback(ADC_HandleTypeDef *hadc);
void program_adc_injected_conv_cplt_callback(ADC_HandleTypeDef *hadc);

/* ── 系统层导出（供 program_current.c / program_svpwm.c 调用）── */
/**
 * 控制驱动芯片休眠/唤醒
 * @param enable  0=休眠（N_SLEEP 拉低），1=唤醒使能功率级
 */
void     program_set_power_stage_enable(uint8_t enable);
/**
 * 读取驱动芯片 nFAULT 引脚状态
 * @return 1=故障有效（引脚低电平），0=正常
 */
uint8_t  program_is_driver_fault_active(void);
/**
 * 查询 debug PWM 测试模式是否激活
 * @return 1=激活，0=关闭
 */
uint8_t  program_debug_pwm_test_is_enabled(void);
/**
 * 用 debug PWM 固定占空比直接驱动三相输出（绕过 FOC 控制）
 */
void     program_apply_debug_pwm_test_output(void);
void     program_apply_svpwm_to_tim1(const foc_svpwm_duty_t *duty);
void     program_update_debug_telemetry(void);

/* ── 快环控制函数（供 motor_state_task 调用） ── */
void     program_reset_speed_loop(void);
void     program_reset_position_loop(void);
void     program_reset_current_loop(void);
void     program_reset_speed_reference_ramp(void);
void     program_reset_encoder_observer(void);
void     program_reset_encoder_alignment(void);
void     program_reset_encoder_align_runtime(void);
void     program_capture_encoder_alignment_sample(void);
float    program_get_encoder_alignment_angle_rad(void);
float    program_get_control_elec_angle_rad(void);
void     program_run_current_loop(float theta_cmd);
void     program_run_voltage_mode(float theta_cmd);
void     program_update_speed_loop(void);
void     program_handle_position_loop_mode_switch(void);
void     program_handle_current_loop_mode_switch(void);

/* ── 获取器 ── */
const volatile program_telemetry_t *program_get_telemetry(void);
motor_state_t *program_get_motor(void);
foc_core_t     *program_get_foc(void);

#endif /* PROGRAM_H */
