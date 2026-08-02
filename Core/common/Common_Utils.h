#ifndef Common_Utils_h
#define Common_Utils_h 

#include "stm32g4xx_hal.h"
#include "usart.h"
#include "spi.h"
#include <stdio.h>
#include "main.h"

/**
 * @brief  打印数据, 通过DMA
 * @param  data: 要打印的数据
 * @param  length: 数据长度
 */ 
void com_PrintData(uint8_t *data, uint16_t length);

/**
 * @brief  启动SPI中断
 */
void com_SPI_IT(void);

/**
 * @brief  通过SPI写入数据
 * @param  data: 要写入的数据
 * @param  length: 数据长度
 */ 
void com_SPI_WriteBytes(uint8_t *data, uint16_t length);

/**
 * @brief  通过SPI读取数据
 * @param  data: 要读取的数据
 * @param  length: 数据长度
 */ 
void com_SPI_ReadBytes(uint8_t *data, uint16_t length);

/**
 * @brief  TIM1 PWM 初始化
 */
void com_TIM1_PWM_Init(void);

/**
 * @brief  TIM1 PWM 测试
 */
void com_TIM1_PWM_Test(void);

/**
 * @brief  ADC1 初始化
 */
void com_ADC_1_Init(void);

/**
 * @brief  ADC1 测试 用于下管电流采样
 */
void com_ADC_1_Test(void);

/**
 * @brief  ADC2 初始化
 */
void com_ADC_2_Init(void);

/**
 * @brief  ADC2 测试 用于母线电压/温度等慢变量
 */
void com_ADC_2_Test(void);

/**
 * @brief  测试函数
 */
void com_test(void);

#endif // !
