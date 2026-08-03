#ifndef __PROGRAM_H
#define __PROGRAM_H

#include "program_utils.h"

/* 调试用 PWM 控制参数 */
typedef struct {
    uint8_t enable;     /* PWM 使能标志：0=关闭, 非0=开启 */
    float   duty_a;     /* A 相占空比 (0.0 ~ 1.0) */
    float   duty_b;     /* B 相占空比 (0.0 ~ 1.0) */
    float   duty_c;     /* C 相占空比 (0.0 ~ 1.0) */
} program_debug_pwm_t;

/* 调试 PWM 全局控制实例 */
extern volatile program_debug_pwm_t program_debug_pwm;  

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


#endif /* __PROGRAM_H */
