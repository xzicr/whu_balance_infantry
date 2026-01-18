#ifndef __UART_RECEIVE_H__
#define __UART_RECEIVE_H__

#include "main.h"
#include "usart.h"
#include "CAN_receive.h"

typedef struct  {
    uint8_t sof1;
    uint8_t sof2;
} frameHeader;

typedef struct
{
    frameHeader header;    
    chassis_data_t receive_chassis_data;
}uart_data_t;



void uart_start_task(void const  *pvParameters);

const chassis_data_t *get_Uart_Chassisdata_point();

#endif