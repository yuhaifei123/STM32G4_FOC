#ifndef PROGRAM_H
#define PROGRAM_H

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "motor_state.h"
#include "foc_core.h"
#include "motor_params.h"
#include "program_config.h"
#include "filter.h"
#include "ma600a.h"

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

/** 调试 PWM 测试参数（调试器中设 enable=1 直接输出固定占空比） */
typedef struct
{
    uint8_t enable;   /** 1=启用调试 PWM，绕过全部 FOC 控制逻辑 */
    float   duty_a;   /** A 相固定占空比 (0~1) */
    float   duty_b;   /** B 相固定占空比 (0~1) */
    float   duty_c;   /** C 相固定占空比 (0~1) */
} program_debug_pwm_test_t;

/* ── 全局对象 extern 声明 ── */
extern volatile program_telemetry_t      g_program_telemetry;
extern volatile program_debug_pwm_test_t g_program_debug_pwm_test;
extern motor_state_t g_motor;
extern foc_core_t    g_foc;
extern ma600a_t      g_ma600a;

/* ── 业务状态结构体 ── */

/**
 * @brief 编码器跨文件接口状态（定义于 program.c）
 * 使用方: program_svpwm.c(测速/对齐) program_current.c(速度环) motor_state.c(对齐状态机) program.c(遥测)
 */
typedef struct
{
    uint8_t  speed_primed;               /* 速度观测器已初始化（首次角度已记录） */
    uint8_t  speed_ready;                /* 速度测量就绪（满一个窗口后置位） */
    uint8_t  speed_loop_update_pending;  /* 速度环更新挂起标志（svpwm 置位, current 消费） */
    float    speed_raw_mech_rad_s;       /* 未滤波机械角速度 (rad/s)，窗口差分原始值 */
    uint8_t  align_done;                 /* 零位对齐完成标志 */
    uint32_t align_counter;              /* 对齐计时器（每 100μs 加 1） */
    float    elec_offset_rad;            /* 电角偏置 (rad)：θe = 编码器电角 - offset */
} program_encoder_t;

/**
 * @brief 控制环运行时状态（定义于 program.c）
 * 使用方: program_current.c(三环控制) program_svpwm.c(滤波/观测) program.c(功率级/性能统计)
 */
typedef struct
{
    uint8_t  power_stage_enabled;                 /* 功率级使能状态：1=唤醒, 0=休眠 */
    float    id_ref_applied_a;                    /* 电流斜坡：实际生效的 id 给定 (A) */
    float    iq_ref_applied_a;                    /* 电流斜坡：实际生效的 iq 给定 (A) */
    filter_lpf_f32_t speed_meas_lpf;              /* 速度测量一阶低通滤波器 */
    filter_lpf_f32_t position_meas_lpf;           /* 位置测量一阶低通滤波器 */
    float    speed_loop_dt_s;                     /* 速度环当前控制周期 (s) */
    float    position_loop_elapsed_s;             /* 位置环累计时间 (s)，用于 200Hz 分频 */
    float    position_meas_output_continuous_rad; /* 位置测量：输出轴连续机械角 (rad) */
    uint8_t  position_loop_enable_prev;           /* 位置环上一拍使能状态（模式切换检测） */
    uint8_t  current_loop_enable_prev;            /* 电流环上一拍使能状态（模式切换检测） */
    uint8_t  position_hold_active;                /* 位置 hold 激活标志 */
    uint8_t  position_hold_release_counter;       /* 位置 hold 释放确认计数器 */
} program_control_t;

extern program_encoder_t g_encoder;   /* 编码器跨文件接口状态实例 */
extern program_control_t g_control;   /* 控制环运行时状态实例 */

/* ── 公共接口 ── */
/**
 * 程序层初始化入口
 * @note 系统启动后只调用一次；保持驱动休眠→ADC校准→初始化对象→启动采样链→使能UART
 */
void program_init(void);
/**
 * 程序层后台任务入口（慢环 1kHz）
 * @note 在 while(1) 中持续调用，内部按 TIM6 1ms 节拍执行；
 *       UART 命令解析→ADC 工程量换算→故障检测→VOFA 波形发送
 */
void program_task(void);
/**
 * 定时器周期回调转发入口
 * @param htim  触发中断的定时器句柄（当前主要是 TIM6 1kHz）
 * @note 识别 TIM6 后递增慢任务时基 g_sys.tim6_tick_ms
 */
void program_tim_period_elapsed_callback(TIM_HandleTypeDef *htim);
/**
 * regular ADC 完成回调转发入口
 * @param hadc  完成转换的 ADC 句柄
 * @note 当前 ADC2 走 DMA 环形缓冲，此处保留空接口
 */
void program_adc_conv_cplt_callback(ADC_HandleTypeDef *hadc);
/**
 * injected ADC 完成回调转发入口（快环 10kHz 入口）
 * @param hadc  完成注入组转换的 ADC 句柄（应为 ADC1）
 * @note 读取三相电流→零偏校准→编码器读取→电流反馈→状态机控制→耗时统计
 */
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
/**
 * 将 SVPWM 三相占空比写入 TIM1 CCR 寄存器
 * @param duty  三相占空比结构体指针（0~1，NULL 安全）
 * @note 占空比→计数值→钳位保护→写入 CH1~CH3 比较寄存器
 */
void     program_apply_svpwm_to_tim1(const foc_svpwm_duty_t *duty);
/**
 * 更新程序层遥测对象到最新状态
 * @note 将电机状态、ADC、编码器、速度/位置/电流等关键变量同步写入 g_program_telemetry
 */
void     program_update_debug_telemetry(void);

/* ── 快环控制函数（供 motor_state_task 调用） ── */
/** 复位速度环：清空积分项与挂起标志，恢复默认速度环周期 */
void     program_reset_speed_loop(void);
/** 复位位置环：清空积分、Hold 状态和累计时间，重置位置测量 LPF */
void     program_reset_position_loop(void);
/** 复位电流环：清零 id/iq 给定/反馈、电压命令和积分项 */
void     program_reset_current_loop(void);
/** 复位速度参考斜坡：机械/电角速度给定及开环速度同步归零 */
void     program_reset_speed_reference_ramp(void);
/** 复位编码器测速观测器：清空连续角、测速窗口和滤波器 */
void     program_reset_encoder_observer(void);
/** 复位编码器零位对齐结果：清空零位偏置和对齐完成标志 */
void     program_reset_encoder_alignment(void);
/** 复位编码器对齐运行时累积量：清空计数器与 sin/cos 累积和 */
void     program_reset_encoder_align_runtime(void);
/** 在对齐保持阶段采集电角度样本（最后 512 拍窗口内累积 sin/cos） */
void     program_capture_encoder_alignment_sample(void);
/** 计算对齐得到的平均电角度（atan2 抗噪声） */
float    program_get_encoder_alignment_angle_rad(void);
/** 获取当前控制电角度：开环模式用积分器角，否则用编码器对齐角 */
float    program_get_control_elec_angle_rad(void);
/** 执行电流环：id/iq PI → 电压限幅 → 反Park+SVPWM */
void     program_run_current_loop(float theta_cmd);
/** 执行电压模式：仅限幅后直接反Park+SVPWM 输出 ud/uq */
void     program_run_voltage_mode(float theta_cmd);
/** 执行速度环：位置环分频调度 → 斜坡 → 速度 PI → iq/uq */
void     program_update_speed_loop(void);
/** 检测位置环使能边沿并处理模式切换（初始化目标位置） */
void     program_handle_position_loop_mode_switch(void);
/** 检测电流环使能边沿并处理模式切换（复位相关控制环） */
void     program_handle_current_loop_mode_switch(void);

/* ── 获取器 ── */
/** 获取当前程序层遥测对象（只读，调试器 Watch 窗口用） */
const volatile program_telemetry_t *program_get_telemetry(void);
/** 获取当前电机状态机对象（调试器直接读写控制参数用） */
motor_state_t *program_get_motor(void);
/** 获取当前 FOC 核心对象 */
foc_core_t     *program_get_foc(void);

#endif /* PROGRAM_H */
