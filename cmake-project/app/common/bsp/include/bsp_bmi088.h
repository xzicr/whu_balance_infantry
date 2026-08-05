#ifndef BSP_BMI088_H
#define BSP_BMI088_H

#include <stddef.h>
#include <stdint.h>    

#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BSP_BMI088_ACCEL = 0,
    BSP_BMI088_GYRO
} bsp_bmi088_device_t;


/* 将加速度计和陀螺仪的片选引脚恢复为高电平 */
void bsp_bmi088_init(void);

/* 完成一次完整的 SPI 收发，函数内部自动控制片选 */
bsp_status_t bsp_bmi088_transfer(
    bsp_bmi088_device_t device,
    const uint8_t *tx_data,
    uint8_t *rx_data,
    size_t length,
    uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BMI088_H */