#include "can_bsp.h"
#include "fdcan.h"
#include "dm4310_drv.h"
#include "string.h"
#include "chassis_task.h"

FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

FDCAN_RxHeaderTypeDef RxHeader3;
uint8_t g_Can3RxData[64];

#define get_motor_measure(ptr, data)                                \
{                                                                   \
		(ptr)->last_ecd = (ptr)->ecd;                                   \
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
		(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
		(ptr)->temperate = (data)[6];                                   \
}
#define get_gimbal_measure(ptr, data)                               \
{                                                                   \
		(ptr)->channel_x = (uint16_t)((data)[0] << 8 | (data)[1]);       \
		(ptr)->channel_y = (uint16_t)((data)[2] << 8 | (data)[3]);       \
		(ptr)->channel_w = (uint16_t)((data)[4] << 8 | (data)[5]);       \
		(ptr)->rc_l = data[6]&0x03;																			\
		(ptr)->rc_r = (data[6]&0x0C)>>2;																			\
		(ptr)->revolve = (data[6]&0x10)>>4;																	\
		(ptr)->sp_flag = (data[6]&0x20)>>5;																	\
		(ptr)->turn_flag = (data[6]&0x40)>>6;																	\
}		
#define get_cap_measure(ptr, data)                                    			 \
{                                                                   				 \
		(ptr)->Value_Bat     = (fp32)(((data)[0] << 8 | (data)[1])*0.001f);      \
		(ptr)->Value_Cap     = (fp32)(((data)[2] << 8 | (data)[3])*0.001f);      \
		(ptr)->Power_Charge  = (fp32)(((data)[4] << 8 | (data)[5])*0.002f);  		 \
		(ptr)->Power_Chassis = (fp32)(((data)[6] << 8 | (data)[7])*0.002f);   	 \
}	

motor_measure_t chassis_3508_motor[2];	

static FDCAN_TxHeaderTypeDef  chassis_3508_tx_message;
static uint8_t                chassis_3508_can_send_data[8];

void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID;//扩展ID不接收
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // 
  sFilterConfig.FilterID2 = 0x00000000; // 
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */ 
/* 过滤扩展ID远程帧 */ 
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

	/* 开启RX FIFO0的新数据中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;                 // CAN ID
  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
  if(len<=8)	
	{
	  TxHeader.DataLength = len<<16;     // 发送长度：8byte
	}
	else  if(len==12)	
	{
	   TxHeader.DataLength =FDCAN_DLC_BYTES_12;
	}
	else  if(len==16)	
	{
	  TxHeader.DataLength =FDCAN_DLC_BYTES_16;
	
	}
  else  if(len==20)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_20;
	}		
	else  if(len==24)	
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_24;	
	}else  if(len==48)
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_48;
	}else  if(len==64)
   {
		 TxHeader.DataLength =FDCAN_DLC_BYTES_64;
	 }
											
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//比特率切换关闭，不适用于经典CAN
  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // CANFD
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
  TxHeader.MessageMarker = 0;//消息标记

   // 发送CAN指令
	 HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
	 return 0;
}


extern chassis_t chassis_move;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN1)
    {
      /* Retrieve Rx messages from RX FIFO0 */
			memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//接收前先清空数组	
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
			
			switch(RxHeader1.Identifier)
			{
        case 0x11 :dm4310_fbdata(&chassis_move.joint_motor[0], g_Can1RxData,RxHeader1.DataLength);break;
        case 0x12 :dm4310_fbdata(&chassis_move.joint_motor[1], g_Can1RxData,RxHeader1.DataLength);break;
				case CAN_3508_L_ID:get_motor_measure(&chassis_3508_motor[0], g_Can1RxData);break;
				case CAN_3508_R_ID:get_motor_measure(&chassis_3508_motor[1], g_Can1RxData);break;				
				default: break;
			}			
	  }		
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN2)
    {
      /* Retrieve Rx messages from RX FIFO0 */
			memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
			switch(RxHeader2.Identifier)
			{
        case 0x11 :dm4310_fbdata(&chassis_move.joint_motor[2], g_Can2RxData,RxHeader2.DataLength);break;
        case 0x12 :dm4310_fbdata(&chassis_move.joint_motor[3], g_Can2RxData,RxHeader2.DataLength);break;	         	
				default: break;
			}	
    }
  }
}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor_l: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor_r: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis_All3508(int16_t motor_l,int16_t motor_r)
{
    uint32_t send_mail_box;
	
    // 配置FDCAN的TxHeader
    chassis_3508_tx_message.Identifier          =   CAN_CHASSIS_ALL3508_ID; // 标准标识符
    chassis_3508_tx_message.IdType              =   FDCAN_STANDARD_ID;      // 标准帧格式
    chassis_3508_tx_message.TxFrameType         =   FDCAN_DATA_FRAME;       // 数据帧
    chassis_3508_tx_message.DataLength          =   FDCAN_DLC_BYTES_8;      // 数据长度码，8字节
    chassis_3508_tx_message.ErrorStateIndicator =   FDCAN_ESI_ACTIVE; 		  // 错误状态指示
    chassis_3508_tx_message.BitRateSwitch       =   FDCAN_BRS_OFF; 					// 关闭位速率切换
    chassis_3508_tx_message.FDFormat            =   FDCAN_CLASSIC_CAN; 			// 使用经典CAN模式
    chassis_3508_tx_message.TxEventFifoControl  =   FDCAN_NO_TX_EVENTS;		  // 不使用Tx事件FIFO
    chassis_3508_tx_message.MessageMarker       =   0; 											// 消息标记
	
	  // 填充数据
	  chassis_3508_can_send_data[0] = (motor_l >> 8);
    chassis_3508_can_send_data[1] = motor_l;
    chassis_3508_can_send_data[2] = (motor_r >> 8);
    chassis_3508_can_send_data[3] = motor_r;
 		chassis_3508_can_send_data[4] = (0 >> 8);
	  chassis_3508_can_send_data[5] = 0;
    chassis_3508_can_send_data[6] = (0 >> 8);
    chassis_3508_can_send_data[7] = 0;

    // 发送数据
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &chassis_3508_tx_message, chassis_3508_can_send_data);
}

/**
  * @brief          返回3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_3508motor_measure_point(uint8_t i)
{
    return &chassis_3508_motor[i];
}

