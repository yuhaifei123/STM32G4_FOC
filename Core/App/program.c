#include "program.h"


/* ── 常量 ── */
#define ADC2_DMA_LEN        2               /* ADC2 DMA 缓冲长度（2 通道） */
#define ADC_REF_V           3.30f           /* ADC 基准电压 (V) */
#define ADC_FULL_SCALE      4095.0f         /* 12bit ADC 满量程 */
#define VBUS_R_UP           240000.0f       /* 母线分压上电阻 240kΩ */
#define VBUS_R_DOWN         10000.0f        /* 母线分压下电阻 10kΩ（分压比 25:1） */

/* ── 全局对象 ── */
static volatile uint16_t g_adc2_dma_buf[ADC2_DMA_LEN]; /* ADC2 DMA 循环缓冲：[0]=CH12(PB2), [1]=CH14(PB11) */
static volatile uint32_t g_tim6_tick_ms = 0;            /* TIM6 中断累加的时间戳 (ms)，每 1ms 自增 */

/* 调试 PWM 全局控制实例 */
volatile program_debug_pwm_t g_debug_pwm;
/**
 * 初始化程序
 */
void Program_Init(void)
{    
    // 使能 DRV832x
    HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin, GPIO_PIN_RESET);
    // ADC 进行校准
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    program_start_adc2_dma_chain();
}

/* 函数作用：启动 ADC2 DMA + TIM6 触发链，用于 VBUS/NTC 慢变量采样。
 * 输入：无。
 * 输出：无返回值；任一步失败则进入 Error_Handler()。
 * 调用频率：系统启动时只调用一次。
 * 运行内容：先启动 ADC2 DMA，再启动 TIM6 更新中断和 TRGO，让 ADC2 按 1 kHz 节拍采样两路慢变量。 
 */
void program_start_adc2_dma_chain(void)
{
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&g_adc2_dma_buf, ADC2_DMA_LEN);
    // 启动TIM6中断
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * 程序任务
 */
void Program_Task(void)
{
static uint32_t last_ms = 0;
    uint32_t now_ms = g_tim6_tick_ms;
    if (now_ms == last_ms) return;
    last_ms = now_ms;

    /* ── 读 ADC2 DMA 缓冲 ── */
    uint16_t vbus_raw = g_adc2_dma_buf[0];
    uint16_t ntc_raw  = g_adc2_dma_buf[1];

    /* 码值 → 电压 */
    float vbus = (float)vbus_raw * ADC_REF_V / ADC_FULL_SCALE
               * (VBUS_R_UP + VBUS_R_DOWN) / VBUS_R_DOWN;

    /* 验证：断点看 vbus 是否 ~供电电压，ntc_raw 是否变化 */
    (void)vbus;
    (void)ntc_raw;
}

/* ── TIM6 中断回调 ── */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        g_tim6_tick_ms++;
    }
}

/**
 * 启动六路互补 PWM  
 */ 
static void program_start_pwm(void)
{
    // 配置占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2124);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2124);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2124);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

static void program_apply_duty(float da, float db, float dc)
{
    // 获取ccr
    uint32_t ccr_a = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1)+ 1;
    // 设置 da, db, dc 对应的比例 0-1之间
    

}