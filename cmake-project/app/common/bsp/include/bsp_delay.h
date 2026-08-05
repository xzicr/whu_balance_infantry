#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 毫秒延时，可用于传感器上电和寄存器配置等待 */
void bsp_delay_ms(uint32_t milliseconds);

/* 微秒延时，用于 SPI 时序和寄存器操作间隔 */
void bsp_delay_us(uint32_t microseconds);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DELAY_H */