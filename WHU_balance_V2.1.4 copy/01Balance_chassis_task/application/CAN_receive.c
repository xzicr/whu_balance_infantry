/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  *  V2.0.0     Nov-12-2025     xzicr              1. support hal lib
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
// motor data read
#define get_motor_measure(ptr, data)                                 \
  {                                                                  \
    (ptr)->last_ecd = (ptr)->ecd;                                    \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);             \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);       \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);   \
    (ptr)->temperate = (data)[6];                                    \
    if ((ptr)->ecd - (ptr)->last_ecd > 4096)                         \
    {                                                                \
      (ptr)->ecd_count--;                                            \
    }                                                                \
    else if ((ptr)->ecd - (ptr)->last_ecd < -4096)                   \
    {                                                                \
      (ptr)->ecd_count++;                                            \
    }                                                                \
    (ptr)->angle = (ptr)->ecd_count * 360 + (ptr)->ecd * 360 / 8192; \
  }

  #define get_lkmotor_measure(ptr, data)                                  \
    {                                                                     \
      (ptr)->temp = (int8_t)data[1];                                      \
      (ptr)->iq = (int16_t)(data[3] << 8 | data[2]);                      \
      (ptr)->speed = (int16_t)(data[5] << 8 | data[4]);                   \
      (ptr)->encoder = (uint16_t)(data[7] << 8 | data[6]);                \
      (ptr)->last_encoder = (ptr)->encoder;                               \
      (ptr)->angle = (float)(((float)(ptr)->last_encoder / 65536) * 360); \
    }

#define get_HT_motor_measure(ptr, data)                                                                \
  {                                                                                                    \
    (ptr)->last_ecd = (ptr)->ecd;                                                                      \
    (ptr)->ecd = uint_to_float((uint16_t)((data)[1] << 8 | (data)[2]), P_MIN, P_MAX, 16)*60.0f;        \
	if((ptr)->ecd>180){(ptr)->ecd-=360;}                                                                    \
	if((ptr)->ecd<-180){(ptr)->ecd+=360;}                                                                    \
  (ptr)->speed_rpm = uint_to_float((uint16_t)(data[3] << 4) | (data[4] >> 4), V_MIN, V_MAX, 12);     \
  (ptr)->real_torque = uint_to_float((uint16_t)(((data[4] & 0x0f) << 8) | (data)[5]), -18, +18, 12); \
}

#define get_supercap_data(temp, data)       \
  {                                         \
    uint16_t *temp = (uint16_t *)data;      \
    Power_data[0] = (float)temp[0] / 100.f; \
    Power_data[1] = (float)temp[1] / 100.f; \
    Power_data[2] = (float)temp[2] / 100.f; \
    Power_data[3] = (float)temp[3] / 100.f; \
  }

	
	
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;

  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef referee_tx_message;
static uint8_t referee_can_send_data[1];

motor_measure_t motor_chassis[7];
lkmotor_measure_t lkmotor_data[2];
HTmotor_measure_t htmotor_data[4];
//static chassis_data_t chassis_data;
/* 超级电容反馈数据 */
float Power_data[4];
uint16_t power_data_temp[4];
uint32_t cnt;




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  if (hcan == &hcan1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
     switch(rx_data[0])
    {
      case CAN_HT_MOTOR_ID1:
      case CAN_HT_MOTOR_ID2:
      case CAN_HT_MOTOR_ID3:
      case CAN_HT_MOTOR_ID4:
      {
        static uint8_t i = 0;
        i = rx_data[0] - CAN_HT_MOTOR_ID1;
        get_HT_motor_measure(&htmotor_data[i], rx_data);
		  break;
      }
      default:
      {
        break;
      }
    }
    switch (rx_header.StdId)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
      static uint8_t i = 0;
      i = rx_header.StdId - CAN_3508_M1_ID;
      get_motor_measure(&motor_chassis[i], rx_data);
      detect_hook(CHASSIS_MOTOR1_TOE + i);
      break;
    }

    case CAN_SUPER_CAP_ID:
    {
      get_supercap_data(power_data_temp, rx_data);
      break;
    }
    default:
    {
      break;
    }
    }
  }
  else if (hcan == &hcan2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
      case CAN_LK_MOTOR_ID1:
      {
        get_lkmotor_measure(&lkmotor_data[0], rx_data);
        break;
      }
      case CAN_LK_MOTOR_ID2:
      {
        get_lkmotor_measure(&lkmotor_data[1], rx_data);
        break;
      }
      case CAN_YAW_MOTOR_ID:
      {
        static uint8_t i = 0;
        i = rx_header.StdId - CAN_3508_M1_ID;
        get_motor_measure(&motor_chassis[i], rx_data)
      }
      case CAN_TRIGGER_MOTOR_ID:
      {
        static uint8_t i = 0;
        i = rx_header.StdId - CAN_3508_M1_ID;
        get_motor_measure(&motor_chassis[i], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + i);
        break;
      }      
      default:
      {
        break;
      }
    }
  }
}

void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&REFEREE_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_cmd_referee_data(uint8_t color)
{
  uint32_t send_mail_box;
  referee_tx_message.StdId = CAN_referee_data;
  referee_tx_message.IDE = CAN_ID_STD;
  referee_tx_message.RTR = CAN_RTR_DATA;
  referee_tx_message.DLC = 0x01;
  referee_can_send_data[0] = color;
  HAL_CAN_AddTxMessage(&REFEREE_CAN, &referee_tx_message, referee_can_send_data, &send_mail_box);
}
void CAN_INIT_STATUS(uint8_t status)
{
  uint32_t send_mail_box;
  referee_tx_message.StdId = CAN_chassis_gimbal_ID;
  referee_tx_message.IDE = CAN_ID_STD;
  referee_tx_message.RTR = CAN_RTR_DATA;
  referee_tx_message.DLC = 0x01;
  referee_can_send_data[0] = status;
  HAL_CAN_AddTxMessage(&REFEREE_CAN, &referee_tx_message, referee_can_send_data, &send_mail_box);
}
//6020y电机协议
void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = motor1 >> 8;
  gimbal_can_send_data[1] = motor1;
  gimbal_can_send_data[2] = motor2 >> 8;
  gimbal_can_send_data[3] = motor2;
  gimbal_can_send_data[4] = motor3 >> 8;
  gimbal_can_send_data[5] = motor3;
  gimbal_can_send_data[6] = motor4 >> 8;
  gimbal_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}



/* ------------------------海泰电机------------------------- */
void CAN_HT_CMD(uint8_t id, fp32 f_t)
{
  uint32_t canTxMailbox = CAN_TX_MAILBOX0;

  fp32 f_p = 0.0f, f_v = 0.0f, f_kp = 0.0f, f_kd = 0.0f;
  uint16_t p, v, kp, kd, t;
  uint8_t buf[8];
  LIMIT_MIN_MAX(f_p, P_MIN, P_MAX);
  LIMIT_MIN_MAX(f_v, V_MIN, V_MAX);
  LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
  LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
  LIMIT_MIN_MAX(f_t, T_MIN, T_MAX);

  p = float_to_uint(f_p, P_MIN, P_MAX, 16);
  v = float_to_uint(f_v, V_MIN, V_MAX, 12);
  kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
  kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
  t = float_to_uint(f_t, T_MIN, T_MAX, 12);

  buf[0] = p >> 8;
  buf[1] = p & 0xFF;
  buf[2] = v >> 4;
  buf[3] = ((v & 0xF) << 4) | (kp >> 8);
  buf[4] = kp & 0xFF;
  buf[5] = kd >> 4;
  buf[6] = ((kd & 0xF) << 4) | (t >> 8);
  buf[7] = t & 0xff;

  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;

  if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)
  {
    canTxMailbox = CAN_TX_MAILBOX0;
  }
  else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
  {
    canTxMailbox = CAN_TX_MAILBOX1;
  }
  else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
  {
    canTxMailbox = CAN_TX_MAILBOX2;
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, buf, (uint32_t *)&canTxMailbox) == HAL_OK)
  {
  };
}
void CAN_CMD_HT_Enable(uint8_t id, uint8_t unterleib_motor_send_data[8])
{
  uint32_t canTxMailbox = CAN_TX_MAILBOX0;

  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;

  if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)
  {
    canTxMailbox = CAN_TX_MAILBOX0;
  }
  else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
  {
    canTxMailbox = CAN_TX_MAILBOX1;
  }
  else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
  {
    canTxMailbox = CAN_TX_MAILBOX2;
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, unterleib_motor_send_data, (uint32_t *)&canTxMailbox) == HAL_OK)
  {
  };
}
/* ------------------------瓴控电机------------------------- */
void CAN_LK_START_control(uint16_t id)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0x88;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_LK_CLOSE_control(uint16_t id)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0x80;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_read_lkmotor_state(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_LK_MOTOR_ID1;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0x9C;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = 0x00;
  chassis_can_send_data[5] = 0x00;
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_LK_POSITION_Control(int32_t angleControl)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_LK_MOTOR_ID1;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0xA3;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = *((uint8_t *)(&angleControl));
  chassis_can_send_data[5] = *((uint8_t *)(&angleControl) + 1);
  chassis_can_send_data[6] = *((uint8_t *)(&angleControl) + 2);
  chassis_can_send_data[7] = *((uint8_t *)(&angleControl) + 3);
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_LK_SPEED_Control(int16_t iqControl, int32_t speedControl)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_LK_MOTOR_ID1;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0xA2;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = *((uint8_t *)(&iqControl));
  chassis_can_send_data[3] = *((uint8_t *)(&iqControl) + 1);
  chassis_can_send_data[4] = *((uint8_t *)(&speedControl));
  chassis_can_send_data[5] = *((uint8_t *)(&speedControl) + 1);
  chassis_can_send_data[6] = *((uint8_t *)(&speedControl) + 2);
  chassis_can_send_data[7] = *((uint8_t *)(&speedControl) + 3);
  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_LK_Torque_Control(uint16_t id,int16_t iqControl)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = id;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0xA1;
  chassis_can_send_data[1] = 0x00;
  chassis_can_send_data[2] = 0x00;
  chassis_can_send_data[3] = 0x00;
  chassis_can_send_data[4] = *((uint8_t *)(&iqControl));
  chassis_can_send_data[5] = *((uint8_t *)(&iqControl) + 1);
  chassis_can_send_data[6] = 0x00;
  chassis_can_send_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_LK_Boradcast_Control(int16_t iqControl_1,int16_t iqControl_2,int16_t iqControl_3,int16_t iqControl_4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x280;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = *(uint8_t *)(&iqControl_1);
  chassis_can_send_data[1] = *((uint8_t *)(&iqControl_1)+1);
  chassis_can_send_data[2] = *(uint8_t *)(&iqControl_2);
  chassis_can_send_data[3] = *((uint8_t *)(&iqControl_2)+1);
  chassis_can_send_data[4] = *(uint8_t *)(&iqControl_3);
  chassis_can_send_data[5] = *((uint8_t *)(&iqControl_3) + 1);
  chassis_can_send_data[6] = *(uint8_t *)(&iqControl_4);
  chassis_can_send_data[7] = *((uint8_t *)(&iqControl_4)+1);
  HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
/* -----------------超级电容功率控制----------------- */
void CAN_Send_Setpower(uint16_t setPower)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_SUPER_CAP_SET_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x02;
  chassis_can_send_data[0] = setPower >> 8;
  chassis_can_send_data[1] = setPower;
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/* ---------------电机数据反馈函数------------------ */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis[4];
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis[5];
}

const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis[6];
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

HTmotor_measure_t *get_HT_motor_measure_point(uint8_t i)
{
  return &htmotor_data[i];
}

lkmotor_measure_t *get_LK_motor_measure_point(uint8_t i)
{
  return &lkmotor_data[(i&0x001)];
}
