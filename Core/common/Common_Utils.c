#include "Common_Utils.h"
#include "adc.h"
#include "tim.h"
#include <string.h>

/* ADC2 DMA 循环缓冲（硬件自动更新） */
static uint32_t adc2_dma_buf[2];  /* [0]=CH12, [1]=CH14 */

/**
 * @brief  打印数据, 通过DMA
 * @param  data: 要打印的数据
 * @param  length: 数据长度
 */
void com_PrintData(uint8_t *data, uint16_t length)
{
    HAL_UART_Transmit(&huart1, data, length, HAL_MAX_DELAY);
}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/**
 * @brief  启动SPI中断
 */
void com_SPI_IT(void)
{
    HAL_SPI_Transmit_IT(&hspi1, NULL, 0);
    HAL_SPI_Receive_IT(&hspi1, NULL, 0);
}

/**
 * @brief  通过SPI写入数据
 * @param  data: 要写入的数据
 * @param  length: 字节数 (自动内部转为16bit传输, 奇数会补齐)
 */
void com_SPI_WriteBytes(uint8_t *data, uint16_t length)
{
    // 拉低 NSS, 开始传输
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    uint16_t size = (length + 1) / 2; // 字节→16bit传输次数 (向上取整)
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
    // 拉高 NSS, 结束传输
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  通过SPI读取数据 (发送0xFF dummy产生时钟)
 * @param  data: 接收缓冲区
 * @param  length: 字节数 (自动内部转为16bit传输, 奇数会补齐)
 */
void com_SPI_ReadBytes(uint8_t *data, uint16_t length)
{
    // 拉低 NSS, 开始传输
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    uint16_t size = (length + 1) / 2; // 字节→16bit传输次数 (向上取整)
    HAL_SPI_TransmitReceive(&hspi1, data, data, size * 2, HAL_MAX_DELAY);
    // 拉高 NSS, 结束传输
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  TIM1 PWM 初始化
 * @note   启动 3 路互补 PWM 输出，设置默认 50% 占空比，死区 ~1us
 */
void com_TIM1_PWM_Init(void)
{
    /* 启动 PWM 通道（含互补输出） */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* 默认 50% 占空比 (ARR=4249) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2124);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2124);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2124);

    // com_PrintData((uint8_t *)"\r\n=== TIM1 PWM Init Done ===\r\n", 30);
    // com_PrintData((uint8_t *)"CH1: PA8/PC13  CH2: PA9/PB0  CH3: PA10/PB1\r\n", 47);
    // com_PrintData((uint8_t *)"Freq:~20kHz  Duty:50%  DeadTime:~1us\r\n\r\n", 42);
}

/**
 * @brief  TIM1 PWM 占空比 + 死区测试
 * @note   每 2 秒自动切换，串口打印状态，示波器观察波形
 *         测试项: 不同占空比、死区时间 0/1us/2us
 */
void com_TIM1_PWM_Test(void)
{
    static uint8_t step = 0;
    static uint32_t last_tick = 0;

    if (HAL_GetTick() - last_tick < 2000)
    {
        return;
    }
    last_tick = HAL_GetTick();

    switch (step)
    {
    case 0:
        /* 25% / 50% / 75%, 死区 ~1us */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1062);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2124);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 3187);
        TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | 85;
     //   com_PrintData((uint8_t *)"CH1=25% CH2=50% CH3=75%  DT=85(~1us)\r\n", 42);
        break;

    case 1:
        /* 互换占空比 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 3187);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1062);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2124);
    //    com_PrintData((uint8_t *)"CH1=75% CH2=25% CH3=50%  DT=85(~1us)\r\n", 42);
        break;

    case 2:
        /* 极限占空比 + 最小死区 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 212);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2124);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4037);
        TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | 0x00;
    //    com_PrintData((uint8_t *)"CH1=5%  CH2=50%  CH3=95%  DT=0(min)\r\n", 41);
        break;

    case 3:
        /* 增大死区 ~2us */
        TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | 170;
    //    com_PrintData((uint8_t *)"CH1=5%  CH2=50%  CH3=95%  DT=170(~2us)\r\n", 43);
        break;

    case 4:
        /* 恢复默认 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2124);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2124);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2124);
        TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | 85;
    //    com_PrintData((uint8_t *)"Restored: 50%/50%/50%  DT=85(~1us)\r\n", 38);
        break;
    }

    //  step++;
    //  if (step > 4) step = 0;
}

/**
 * @brief  ADC1 初始化（校准 + 启动注入转换）
 * @note   注入组由 TIM1_TRGO2 触发（~20kHz），3 通道：CH1/CH3/CH4
 */
void com_ADC_1_Init(void)
{
    static uint8_t inited = 0;
    if (inited) return;
    inited = 1;

    /* ADC 校准 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* 启动注入转换（等待 TIM1_TRGO2 触发，需 TIM1 已启动） */
    HAL_ADCEx_InjectedStart(&hadc1);
}

/**
 * @brief  ADC1 测试：每秒读取 3 路注入通道并打印电压
 * @note   需要 TIM1 PWM 已启动（提供 TRGO2 触发信号）
 */
void com_ADC_1_Test(void)
{
    static uint32_t last_tick = 0;
    char msg[80];

    if (HAL_GetTick() - last_tick < 1000){
        return;
    }
    last_tick = HAL_GetTick();

    // 获取模拟值
    uint32_t raw1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);  
    uint32_t raw3 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);  
    uint32_t raw4 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);  

    uint32_t mv1 = raw1 * 3300 / 4095;
    uint32_t mv3 = raw3 * 3300 / 4095;
    uint32_t mv4 = raw4 * 3300 / 4095;

    // snprintf(msg, sizeof(msg), "ADC: CH1=%lumV CH3=%lumV CH4=%lumV\r\n", mv1, mv3, mv4);
    // com_PrintData((uint8_t*)msg, strlen(msg));
}

/**
 * @brief  ADC2 初始化（校准 + 启动 TIM6 触发 + DMA 传输）
 * @note   常规通道扫描模式，TIM6_TRGO 触发 @1kHz，DMA 循环
 *         CH12(PB2) Rank1 / CH14(PB11) Rank2
 */
void com_ADC_2_Init(void)
{
    static uint8_t inited = 0;
    if (inited) return;
    inited = 1;

    // TODO 通过TIM6 为ADC2提供触发信号 (1kHz)
    HAL_TIM_Base_Start(&htim6);

    /* ADC 校准 */
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    /* 启动 ADC DMA（循环模式，自动搬运 2 通道结果到 adc2_dma_buf） */
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_dma_buf, 2);
}

/**
 * @brief  ADC2 测试：每秒读取 DMA 缓冲并打印电压
 * @note   用于母线电压/温度等慢变量
 */
void com_ADC_2_Test(void)
{
    static uint32_t last_tick = 0;
    char msg[80];

    // 过一段时间,获取ADC值
    if (HAL_GetTick() - last_tick < 1000){
        return;
    }
    last_tick = HAL_GetTick();

    /* 直接读 DMA 缓冲（硬件自动更新） */
    uint32_t raw12 = adc2_dma_buf[0] & 0xFFFF;
    uint32_t raw14 = adc2_dma_buf[1] & 0xFFFF;

    uint32_t mv12 = raw12 * 3300 / 4095;
    uint32_t mv14 = raw14 * 3300 / 4095;

    snprintf(msg, sizeof(msg),"CH12=%lumV CH14=%lumV\r\n", mv12, mv14);
    com_PrintData((uint8_t*)msg, strlen(msg));
}

/**
 * @brief  测试函数，包含 SPI、TIM1 PWM 和 ADC1 测试
 */
void com_test(void)
{
    // SPI 测试
    // com_SPI_WriteBytes((uint8_t*)"Hello, World!\r\n", 14);
    // com_SPI_ReadBytes((uint8_t*)"Hello, World!\r\n", 14);

    // 测试TIM1 PWM
    com_TIM1_PWM_Init();
    com_TIM1_PWM_Test();

    // 测试ADC1
    // com_ADC_1_Init();
    // com_ADC_1_Test();

    // 测试ADC2（母线电压/温度）
    com_ADC_2_Init();
    com_ADC_2_Test();
}

