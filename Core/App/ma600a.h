#ifndef MA600A_H
#define MA600A_H

#include <stdint.h>

#include "spi.h"

/* MA600A 磁编码器设备对象，通过 SPI1 获取转子绝对机械角度 */
typedef struct
{
    SPI_HandleTypeDef *hspi;        /* SPI 句柄（SPI1） */
    GPIO_TypeDef *cs_port;          /* 片选端口（PA4） */
    uint16_t cs_pin;                /* 片选引脚编号 */
    uint16_t tx_word;               /* 发送命令字 0x0000 */
    uint16_t rx_word;               /* 接收数据字 */
    uint16_t angle_raw;             /* 原始角度码 0~16383 */
    float angle_deg;                /* 角度制 0°~360° */
    float angle_rad;                /* 弧度制 0~2π */
    uint8_t data_valid;             /* 数据有效标志 */
    uint8_t transfer_busy;          /* 传输忙标志 */
    uint8_t consecutive_bad_count;  /* 连续异常计数 */
    uint32_t sample_counter;        /* 累计采样次数 */
    uint32_t reject_count;          /* 跳变拒绝次数 */
    uint32_t comm_error_count;      /* 通信失败次数 */
} ma600a_t;

/* 函数作用：初始化 MA600A 设备对象并绑定底层 SPI 与片选引脚。
 * 输入：sensor 为设备对象指针，hspi 为已完成 CubeMX 初始化的 SPI 句柄，
 *      cs_port/cs_pin 为编码器片选引脚。
 * 输出：无返回值。
 * 运行频率：系统上电初始化时调用 1 次。
 * 运行内容：保存驱动依赖、清零角度缓存，并把片选脚拉高到空闲状态。
 */
void ma600a_init(ma600a_t *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* 函数作用：读取 MA600A 当前机械绝对角度。
 * 输入：sensor 为已完成初始化的 MA600A 设备对象。
 * 输出：返回 1 表示本次 SPI 读取成功，返回 0 表示读取失败。
 * 运行频率：当前建议放在后台慢任务中约 1 kHz 调用，后续也可迁移到控制节拍里。
 * 运行内容：在当前 CubeMX 配置的 16bit SPI 帧下发送 1 个空命令字，
 *      读取返回的 16bit 角度码，并同步换算成角度制与弧度制。
 */
/* Called from the 10 kHz fast-loop ISR to kick the next SPI transfer. */
uint8_t ma600a_read_angle(ma600a_t *sensor);

void ma600a_spi_txrx_cplt_callback(ma600a_t *sensor, SPI_HandleTypeDef *hspi);
void ma600a_spi_error_callback(ma600a_t *sensor, SPI_HandleTypeDef *hspi);

#endif /* MA600A_H */
