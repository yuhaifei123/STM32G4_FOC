#ifndef CLI_UART_H
#define CLI_UART_H

#include <stdint.h>

#include "usart.h"

/**
 * 初始化调试串口，绑定 UART 句柄
 * @param huart  UART 句柄指针
 */
void     cli_uart_init(UART_HandleTypeDef *huart);

/**
 * 阻塞发送文本字符串（调试用）
 * @param text  以 '\0' 结尾的字符串
 */
void     cli_uart_send_text(const char *text);

/**
 * 通过 DMA 发送 VOFA JustFloat 波形数据
 * 数据格式：float 二进制数组 + 尾帧 0x00 0x00 0x80 0x7F
 * @param values  float 数组指针
 * @param count   数组元素个数
 * @return 1=发送成功, 0=忙/失败（丢帧）
 */
uint8_t  cli_uart_send_vofa(const float *values, uint8_t count);

/**
 * 查询 DMA 发送是否忙碌
 * @return 1=忙碌中, 0=空闲
 */
uint8_t  cli_uart_is_tx_busy(void);

#endif /* CLI_UART_H */
