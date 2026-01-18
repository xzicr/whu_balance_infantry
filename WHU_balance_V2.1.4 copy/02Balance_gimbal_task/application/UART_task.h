#ifndef __UART_TASK_H
#define __UART_TASK_H

#include "main.h"
#include "usart.h"
#include "gimbal_task.h"


typedef struct  {
    uint8_t sof1;
    uint8_t sof2;
} frameHeader;

typedef struct
{
    frameHeader header;
    chassis_data_t send_chassis_data;
}uart_data_t;

void uart_task(void const *pvParameters);

#endif