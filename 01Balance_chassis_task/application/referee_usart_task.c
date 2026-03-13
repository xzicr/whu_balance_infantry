/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "uart_receive.h"
#include "ui.h"
#include "chassis_task.h"
#include "shoot.h"


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

uint32_t ui_fpsTick; // 刷新计数
uint32_t ui_period = 100;
uint32_t ui_initTick_1; // 重新初始化绘制计数（解决丢包导致静态图案丢失）
uint32_t ui_initTick_2;
uint32_t ui_initTick_3;
extern __IO uint32_t uwTick; // 系统时钟

//动态数据指针外部声明
extern ui_interface_round_t *ui_default_DynamicBottomGroup_FricRound;
extern ui_interface_round_t *ui_default_DynamicBottomGroup_AimRound;
extern ui_interface_arc_t *ui_default_DynamicLeftGroup_ShootHeatArc;
extern ui_interface_arc_t *ui_default_DynamicLeftGroup_PowerArc;
extern ui_interface_arc_t *ui_default_DynamicRightGroup_DynamicPitchArc;
extern ui_interface_arc_t *ui_default_DynamicRightGroup_ChassisArc;
extern ui_interface_number_t *ui_frame1_DynamicNumberGroup_NewNumber;
//裁判系统
extern robot_status_t robot_state; //机器人状态
extern power_heat_data_t power_heat_data;  //机器人功率与发射热量
extern projectile_allowance_t bullet_remaining;  //剩余弹量
extern game_status_t game_state;   //比赛信息
extern int ui_self_id;  //机器人ID

//云台数据
extern uart_data_t uart_data;
//拨弹盘结构体数据
extern shoot_control_t shoot_control;
//暂存变量

void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
		osDelay(10);
	
    ui_init_default_DynamicBottomGroup();
    ui_init_default_DynamicHightGroup();
    ui_init_default_DynamicLeftGroup();
    ui_init_default_DynamicRightGroup();
    ui_init_default_StaticLeftGroup();
    ui_init_default_StaticGroup();
    ui_init_default_StaticMiddleGroup();
    ui_init_frame1_DynamicNumberGroup();
    ui_init_frame1_StaticNumberGroup();
    ui_init_frame1_StaticTextGroup();

    while(1)
    {
        ui_self_id =robot_state.robot_id;        
        referee_unpack_fifo_data();
        osDelay(10);

        //定时初始化防止图形未进行初始化
        if(uwTick - ui_initTick_1 >= 3*ui_period)
        {
          ui_initTick_1 =uwTick;
          ui_init_default_DynamicBottomGroup();
          ui_init_default_DynamicHightGroup();
          ui_init_default_DynamicLeftGroup();
        }
        if(uwTick - ui_initTick_2 >= 4*ui_period)
        {
          ui_initTick_2 =uwTick;
          ui_init_default_DynamicRightGroup();
          ui_init_default_StaticLeftGroup();
          ui_init_default_StaticGroup();
        }
        if(uwTick - ui_initTick_3 >= 3*ui_period)
        {
          ui_initTick_3 =uwTick;
          ui_init_default_StaticMiddleGroup();
          ui_init_frame1_DynamicNumberGroup();
          ui_init_frame1_StaticNumberGroup();
          ui_init_frame1_StaticTextGroup();
        }
        //设置按键触发重新初始化
        if(uart_data.receive_chassis_data.ui_init_flag==1)
        {
          ui_init_default_DynamicBottomGroup();
          ui_init_default_DynamicHightGroup();
          ui_init_default_DynamicLeftGroup();
          osDelay(ui_period);
          ui_init_default_DynamicRightGroup();
          ui_init_default_StaticLeftGroup();
          ui_init_default_StaticGroup();
          osDelay(ui_period);
          ui_init_default_StaticMiddleGroup();
          ui_init_frame1_DynamicNumberGroup();
          ui_init_frame1_StaticNumberGroup();
          ui_init_frame1_StaticTextGroup();
        }
        //开始伟大的编程吧
        //自瞄和摩擦轮
        if(uart_data.receive_chassis_data.fric_flag==1)
        {
          ui_default_DynamicBottomGroup_FricRound->color = UI_Color_Pink;
        }
        else
        {
          ui_default_DynamicBottomGroup_FricRound->color = UI_Color_White;
        }
        if(uart_data.receive_chassis_data.auto_flag==1)
        {
          ui_default_DynamicBottomGroup_AimRound->color = UI_Color_Pink;
        }
        else
        {
          ui_default_DynamicBottomGroup_AimRound->color = UI_Color_White;
        }

        ui_update_default_DynamicHightGroup();
        ui_update_default_DynamicBottomGroup();
        // osDelay(ui_period);
        // ui_update_default_DynamicBottomGroup();
        //底盘缓冲能量和枪口热量
        // ui_default_DynamicLeftGroup_ShootHeatArc->start_angle = 
        // 290;//-fp32_constrain( power_heat_data.shooter_17mm_1_barrel_heat/robot_state.shooter_barrel_heat_limit*20.0f,0,20);
        // ui_default_DynamicLeftGroup_PowerArc->start_angle = 
        // 290;//+fp32_constrain( power_heat_data.buffer_energy,0,50);
        // ui_default_DynamicLeftGroup_ShootHeatArc->start_angle = 
        // 270.0-fp32_constrain( uart_data.receive_chassis_data.vx_set,0,20);
        // ui_default_DynamicLeftGroup_PowerArc->start_angle = 
        // 225.0+fp32_constrain( uart_data.receive_chassis_data.vy_set,0,55);
        ui_update_default_DynamicLeftGroup();

        ui_default_DynamicRightGroup_ChassisArc->start_angle = 200-chassis_move.chassis_posture_info.yaw_angle_total*180/PI;
        ui_default_DynamicRightGroup_ChassisArc->end_angle = 160-chassis_move.chassis_posture_info.yaw_angle_total*180/PI;
        ui_default_DynamicRightGroup_DynamicPitchArc->start_angle = 90-chassis_move.chassis_posture_info.pitch_angle*180/PI;
        ui_default_DynamicRightGroup_DynamicPitchArc->end_angle = 91-chassis_move.chassis_posture_info.pitch_angle*180/PI;

        ui_update_default_DynamicRightGroup();
        // osDelay(ui_period);
        // ui_update_default_DynamicRightGroup();
        ui_frame1_DynamicNumberGroup_NewNumber->number = shoot_control.shoot_motor_measure->speed_rpm;
        if( shoot_control.shoot_motor_measure->speed_rpm<200)
        {
          ui_frame1_DynamicNumberGroup_NewNumber->color = UI_Color_Purplish_red;
        }
        else
        {
          ui_frame1_DynamicNumberGroup_NewNumber->color = UI_Color_Green;
        }
        ui_update_frame1_DynamicNumberGroup();

    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


void USART6_IRQHandler(void)
{
   static volatile uint8_t res;
   if(USART6->SR & UART_FLAG_IDLE)
   {
       __HAL_UART_CLEAR_PEFLAG(&huart6);

       static uint16_t this_time_rx_len = 0;

       if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
       {
           __HAL_DMA_DISABLE(huart6.hdmarx);
           this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
           __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
           huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
           __HAL_DMA_ENABLE(huart6.hdmarx);
           fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
           detect_hook(REFEREE_TOE);
       }
       else
       {
           __HAL_DMA_DISABLE(huart6.hdmarx);
           this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
           __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
           huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
           __HAL_DMA_ENABLE(huart6.hdmarx);
           fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
           detect_hook(REFEREE_TOE);
       }
   }
}


