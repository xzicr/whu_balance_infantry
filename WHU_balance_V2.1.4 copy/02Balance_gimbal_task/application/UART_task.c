#include "UART_task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"

uart_data_t uart_data;
chassis_data_t *chassis_data_get;
void uart_task(void const *pvParameters)
{
    uart_data.header.sof1=0xAA;
    uart_data.header.sof2=0xFF;
	chassis_data_get=get_chassis_data_point();
    while(1)
    {
        memcpy((uint8_t *)&(uart_data.send_chassis_data),chassis_data_get,sizeof(chassis_data_t));
        HAL_UART_Transmit(&huart1,(uint8_t *)&uart_data,sizeof(uart_data_t),HAL_MAX_DELAY);
        osDelay(10);
    }
}

