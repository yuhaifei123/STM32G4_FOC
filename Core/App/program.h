#ifndef __PROGRAM_H
#define __PROGRAM_H

#include "program_utils.h"
#include "ma600a.h"
#include "foc_core.h"

/* 调试用 PWM 控制参数 */
typedef struct {
    uint8_t enable;     /* PWM 使能标志：0=关闭, 非0=开启 */
    float   duty_a;     /* A 相占空比 (0.0 ~ 1.0) */
    float   duty_b;     /* B 相占空比 (0.0 ~ 1.0) */
    float   duty_c;     /* C 相占空比 (0.0 ~ 1.0) */
} program_debug_pwm_t;

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

/* 调试 PWM 全局控制实例 */
extern volatile program_debug_pwm_t program_debug_pwm;  
/* ADC2 DMA 循环缓冲：[0]=CH12(PB2), [1]=CH14(PB11) */
extern volatile uint16_t g_adc2_dma_buf[ADC2_DMA_LEN]; 
// 程序 telemetry 数据 
extern volatile program_telemetry_t g_program_telemetry;
extern ma600a_t g_ma600a;
// FOC 核心对象，持有所有中间变量和输出  
extern foc_core_t g_foc;

/**
 * 初始化程序
 */
void Program_Init(void);

/**
 * 程序任务
 */
void program_Task(void);

/* 函数作用：启动 ADC2 DMA + TIM6 触发链，用于 VBUS/NTC 慢变量采样。
 * 输入：无。
 * 输出：无返回值；任一步失败则进入 Error_Handler()。
 * 调用频率：系统启动时只调用一次。
 * 运行内容：先启动 ADC2 DMA，再启动 TIM6 更新中断和 TRGO，让 ADC2 按 1 kHz 节拍采样两路慢变量。 
 */
void program_start_adc2_dma_chain(void);


/* SVPWM 占空比应用到 TIM1 */
void program_apply_svpwm_to_tim1(const foc_svpwm_duty_t *duty);

/* 获取 MA600A 数据 */
void get_ma600a(ma600a_t *ma600a); 

#endif /* __PROGRAM_H */
