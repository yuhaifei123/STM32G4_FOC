#include "program.h"
#include <math.h>   // ← 只需要这一个头文件
#include "Common_Utils.h"
#include "ma600a.h"
#include "program_current.h"
#include "foc_core.h"

/* ── 全局对象 ── */
/* ADC2 DMA 循环缓冲：[0]=CH12(PB2), [1]=CH14(PB11) */
volatile uint16_t g_adc2_dma_buf[ADC2_DMA_LEN]; 
// 程序 telemetry 数据 
volatile program_telemetry_t g_program_telemetry;
 /* TIM6 中断累加的时间戳 (ms)，每 1ms 自增 */
static volatile uint32_t g_tim6_tick_ms = 0;           
/* 调试 PWM 全局控制实例 */
volatile program_debug_pwm_t g_debug_pwm;
// MA600A 磁编码器设备对象
ma600a_t g_ma600a;
// FOC 核心对象，持有所有中间变量和输出  
foc_core_t g_foc;

/* ── 前置声明 ── */
/* 启动 ADC2 DMA 链（TIM6 触发） */
static void program_start_adc2_dma_chain(void);      
/* 启动 TIM1 六路互补 PWM */
static void program_start_tim1_pwm_outputs(void);    



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

    program_start_tim1_pwm_outputs();

    /* 默认关闭 debug PWM */
    g_debug_pwm.enable = 0;
    g_debug_pwm.duty_a = 0.30f;
    g_debug_pwm.duty_b = 0.40f;
    g_debug_pwm.duty_c = 0.60f;

    // 初始化
    ma600a_init(&g_ma600a, &hspi1, SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
    // 启动 ADC1 注入采样
    program_start_adc1_injected();
}

/**
 * 程序任务
 */
void program_Task(void)
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
    // TODO 需要测试
    // 打印 vbus ntc_raw
    // com_PrintData((uint8_t*)&vbus, sizeof(vbus));
    // com_PrintData((uint8_t*)&ntc_raw, sizeof(ntc_raw));

    if(g_debug_pwm.enable)
    {
        HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin, GPIO_PIN_SET);
        foc_svpwm_duty_t duty = { .a = g_debug_pwm.duty_a, .b = g_debug_pwm.duty_b, .c = g_debug_pwm.duty_c };
        program_apply_svpwm_to_tim1(&duty);
        return;
    }

    // 获取角度
    ma600a_read_angle(&g_ma600a);
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
static void program_start_tim1_pwm_outputs(void)
{
    // 配置占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2124);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2124);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2124);

    // 启动 PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    // 启动互补 PWM
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void program_apply_svpwm_to_tim1(const foc_svpwm_duty_t *duty)
{
    // 获取 PWM 周期（CCR + 1 近似 ARR+1=4250，用于占空比计算）
    uint32_t period = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1)+ 1;
    // 限幅到 [0, 1]
    float da = program_clamp_f32(duty->a, 0.0f, 1.0f);
    float db = program_clamp_f32(duty->b, 0.0f, 1.0f);
    float dc = program_clamp_f32(duty->c, 0.0f, 1.0f);

    // 写入 TIM1 比较寄存器
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(period * da));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(period * db));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(period * dc));
}

/* 获取 MA600A 数据 */
void get_ma600a(ma600a_t *out)
{
    *out = g_ma600a;   /* 全局实例 → 输出参数 */
}

