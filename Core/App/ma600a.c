#include "ma600a.h"


/**
 * 初始化 MA600A 磁编码器设备对象
 * @param ma600a       MA600A 磁编码器设备对象
 * @param hspi         SPI 句柄（SPI1）
 * @param cs_port      片选端口（PA4）
 * @param cs_pin       片选引脚编号
 */
void ma600a_init(ma600a_t *ma600a, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    ma600a->hspi = hspi;
    ma600a->cs_port = cs_port;
    ma600a->cs_pin = cs_pin;
    ma600a->tx_word = 0x0000;
    ma600a->rx_word = 0x0000;
    ma600a->angle_raw = 0x0000;
    ma600a->angle_deg = 0.0f;
    ma600a->angle_rad = 0.0f;
    ma600a->data_valid = 0;
    ma600a->transfer_busy = 0;
    ma600a->consecutive_bad_count = 0;
    ma600a->sample_counter = 0;
    ma600a->reject_count = 0;
    ma600a->comm_error_count = 0;
}

/**
 * 读取角度数据
 * @param enc          MA600A 磁编码器设备对象
 * @return             数据有效标志
 */
uint8_t ma600a_read_angle(ma600a_t *enc)
{
    enc->tx_word = MA600A_CMD_ANGLE;
    enc->rx_word = 0;

    // 读取角度数据
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(enc->hspi, (uint8_t*)&enc->tx_word, (uint8_t*)&enc->rx_word, 1, 100);
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);

    enc->sample_counter++;
    if (status != HAL_OK)
    {
        enc->comm_error_count++;
        enc->consecutive_bad_count++;
        enc->data_valid = 0;
        return 0;
    }

    // 判断接收的数据是否正确
    if (enc->rx_word & MA600A_ANGLE_VALID_MASK == 0)
    {
        enc->comm_error_count++;
        enc->consecutive_bad_count++;
        enc->data_valid = 0;
        return 0;  
    }

    enc->consecutive_bad_count = 0;
    enc->angle_raw = enc->rx_word & MA600A_ANGLE_DATA_MASK;
    // 角度制
    enc->angle_deg = (float)enc->angle_raw * 360.0f / 16384.0f;
    // 弧度制
    enc->angle_rad = (float)enc->angle_raw * TWO_PI/ 16384.0f;
    enc->data_valid = 1;    
    return 1;
}
