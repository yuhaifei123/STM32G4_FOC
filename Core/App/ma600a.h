#ifndef  MA600A_H
#define  MA600A_H
#include "stm32g4xx_hal.h"
#include "program_utils.h"

/*
 * MA600A SPI 协议：16bit 帧，发送全 0 即读角度，返回格式：
 * ┌────────┬────────┬───────────────────────┐
 * │ bit15  │ bit14  │ bit13 ~ bit0          │
 * │ 保留   │ 有效位  │ 14bit 角度码 (0~16383) │
 * └────────┴────────┴───────────────────────┘
 * 精度：360° / 16384 ≈ 0.022°
 */
/* 读取角度的 SPI 命令字：发送 0x0000 即可获取当前角度 */
#define MA600A_CMD_ANGLE 0X0000

/* 提取 bit14 有效标志：0x4000 = 0100_0000_0000_0000 */
/* 若 (rx_data & 0x4000) == 0 → 数据无效，需丢弃 */
#define MA600A_ANGLE_VALID_MASK 0x4000

/* 提取 bit13~0 角度数据：0x3FFF = 0011_1111_1111_1111 */
/* 角度码 = rx_data & 0x3FFF，范围 0 ~ 16383 */
#define MA600A_ANGLE_DATA_MASK   0x3FFF


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

/**
 * 初始化 MA600A 磁编码器设备对象
 * @param ma600a       MA600A 磁编码器设备对象
 * @param hspi         SPI 句柄（SPI1）
 * @param cs_port      片选端口（PA4）
 * @param cs_pin       片选引脚编号
 */
void ma600a_init(ma600a_t *ma600a, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
 * 读取角度数据
 * @param enc          MA600A 磁编码器设备对象
 * @return             数据有效标志
 */
uint8_t ma600a_read_angle(ma600a_t *enc);

#endif // !MA600A_H
