#include "Self_aim.h"
#include "main.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "string.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "CAN_receive.h"
uint8_t buffer[sizeof(RECEIVE_DATA)];//用于存储完整的数据包
uint8_t Usart_Receive[1];//用于接收单个字节的数据
//uint8_t Usart_Receive[14];
uint16_t received = 0;//当前接收到的数据长度
uint8_t Count=0;//接收状态标签
InputData inputdata;
SEND_DATA Send_Data;
extern __IO uint32_t uwTick;
uint32_t sendTick =0;
gimbal_control_t *gimbal_data;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t received_byte = Usart_Receive[0]; // 读取接收到的字节
        // buffer_single = received_byte;
        // 读取该字节，检查是否为帧头的sof
        if (Count == 0)
        {
            if (received_byte == FRAME_HEADER_SOF)
            {
                buffer[received++] = received_byte; // 存储帧头起始标志
                Count = 1;                          // 进入帧头接收阶段
            }
            else
            {
                received = 0; // 不是帧头标志则丢弃数据
            }
        }
        else if (Count == 1) // 接收帧头的crc8字段
        {
            buffer[received++] = received_byte;

            // 检查帧头是否接收完成
            if (received == sizeof(FrameHeader))
            {
                FrameHeader *header = (FrameHeader *)buffer;

                // 检查sof和crc8是否符合固定值
                if (header->sof == FRAME_HEADER_SOF && header->crc8 == 0x00)
                {
                    Count = 2; // 帧头校验通过，准备接收数据部分
                }
                else
                {
                    received = 0; // 帧头不匹配，重置接收
                    Count = 0;
                }
            }
        }
        else if (Count == 2) // 接收数据部分以及帧尾
        {
            buffer[received++] = received_byte;

            // 检查数据包是否接收完整
            if (received == sizeof(FrameHeader) + sizeof(InputData) + sizeof(FrameTailer))
            {
                FrameTailer *tail = (FrameTailer *)(buffer + sizeof(FrameHeader) + sizeof(InputData));

                // 检查帧尾
                if (verify_CRC16_check_sum((uint8_t *)buffer, sizeof(RECEIVE_DATA)))
                {

                    // 数据包检验成功，解析数据部分
                    memcpy(&inputdata, buffer + sizeof(FrameHeader), sizeof(InputData));
                    // memset(buffer, 0, sizeof(RECEIVE_DATA));
                }

                // 无论是否校验成功，都重置接收状态
                received = 0;
                Count = 0;
            }
        }
        // 重新开始接收
        HAL_UART_Receive_IT(&huart1, (uint8_t *)Usart_Receive, 1);
    }
}
void data_transition(void)
{
    uint8_t enemy_color;
    get_enemy_color(&enemy_color);
    Send_Data.frame_header.sof = FRAME_HEADER_SOF; // Frame_header
    Send_Data.frame_header.crc8 = FRAME_HEADER_CRC8;
    // Send_Data.frame_tailer.crc16 = FRAME_TAILER_CRC16; //Frame_tail
    gimbal_data = get_gimbal_data();
    Send_Data.output_data.curr_yaw = gimbal_data->gimbal_yaw_motor.absolute_angle / 180 * 3.1415926;
    Send_Data.output_data.curr_pitch = gimbal_data->gimbal_pitch_motor.absolute_angle / 180 * 3.1415926;
    Send_Data.output_data.curr_omega = gimbal_data->gimbal_yaw_motor.motor_gyro; //-0.000006f;
    Send_Data.output_data.state = 0;
    Send_Data.output_data.autoaim = 4;
    Send_Data.output_data.enemy_color = enemy_color;

    uint8_t *data_ptr = (uint8_t *)&Send_Data;                                       // 将Send_Data作为字节数组
    uint32_t data_length = sizeof(SEND_DATA) - sizeof(Send_Data.frame_tailer.crc16); // 计算CRC的长度，排除crc16字段
    Send_Data.frame_tailer.crc16 = get_CRC16_check_sum(data_ptr, data_length, FRAME_TAILER_CRC16_init);
    // uint8_t *data_ptr = (uint8_t *)&Send_Data.output_data;
    // uint32_t data_length = sizeof(Send_Data.output_data);
    // Send_Data.frame_tailer.crc16 = get_CRC16_check_sum(data_ptr, data_length, FRAME_TAILER_CRC16_init);
}
void USART1_SEND(void)
{
    if (uwTick - sendTick < 30)
        return;
    data_transition();
    sendTick = uwTick;
    uint8_t *data_ptr = (uint8_t *)&Send_Data;
    uint16_t data_length = sizeof(SEND_DATA);
    // uint16_t data_length = sizeof(SEND_DATA)+1;
    // data_ptr[sizeof(SEND_DATA)]='\n';
    // 使用 HAL_UART_Transmit 发送整个数据包
    uint8_t newline = '\n';
    HAL_UART_Transmit_IT(&huart1, data_ptr, data_length);
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
        ;
    HAL_UART_Transmit_IT(&huart1, &newline, 1);
    // HAL_Delay(100);
}
void Self_aim_task(void const *pvParameters)
{
	HAL_UART_Receive_IT(&huart1, (uint8_t *)Usart_Receive, 1);
	while(1)
	{
		USART1_SEND();
		osDelay(30);
	}
}
InputData *get_selfaim_data()
{
	return &inputdata;
}