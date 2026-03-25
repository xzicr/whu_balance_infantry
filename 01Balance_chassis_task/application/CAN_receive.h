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
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define REFEREE_CAN hcan2

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define P_MIN -95.5f// Radians
#define P_MAX 95.5f
#define V_MIN -45.0f// Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f// N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f// N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
/* CAN send and receive ID */
typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_3508_M1_ID = 0x201,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,

  CAN_YAW_MOTOR_ID = 0x205,
  CAN_PIT_MOTOR_ID = 0x206,
  CAN_TRIGGER_MOTOR_ID = 0x207,
  CAN_GIMBAL_ALL_ID = 0x1FF,

  CAN_gmbial_chassis_data1 = 0x01,
  CAN_gmbial_chassis_data2 = 0x02,
  CAN_gmbial_chassis_data3 = 0x03,
  CAN_gmbial_chassis_data4 = 0x04,
  CAN_chassis_gimbal_ID = 0x123,
  CAN_referee_data = 0x05,

  CAN_HT_MOTOR_ID1 = 0x01,
  CAN_HT_MOTOR_ID2 = 0x02,
  CAN_HT_MOTOR_ID3 = 0x03,
  CAN_HT_MOTOR_ID4 = 0x04,

  CAN_LK_MOTOR_ID1 = 0x141,
  CAN_LK_MOTOR_ID2 = 0x142,
  CAN_LK_MOTOR_ID3 = 0x143,

  /* 超级电容ID */
  CAN_SUPER_CAP_ID = 0x211,
  /* 超级电容设置ID */
  CAN_SUPER_CAP_SET_ID = 0x210,

} can_msg_id_e;




//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	fp32 angle;
    int32_t ecd_count;
} motor_measure_t;
typedef struct
{
	int8_t temp;
	int16_t iq;
	int16_t speed;
	uint16_t encoder;
	uint16_t last_encoder;
	float angle;
  uint32_t last_update_time;
}lkmotor_measure_t;

typedef struct 
{
  float last_ecd;
  float ecd;
  float speed_rpm;
  float real_torque;
}HTmotor_measure_t;

typedef union
{
	float float_t;
	uint8_t uint8_t[4];
} send_float_typedef;
typedef enum
{
  CHASSIS_MODE_OFF=0,
  CHASSIS_MOVE_ON,
  CHASSIS_MODE_DEBUG,
  CHASSIS_MODE_INIT,
}RC_chassis_mode_e;

typedef enum
{
    SHOOT_STOP = 0,   
    SHOOT_READY_FRIC,  
    SHOOT_READY_BULLET,

    SHOOT_SINGLE,
    SHOOT_CONTINUE,
    SHOOT_READY,       
    SHOOT_BULLET,      
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,            
} shoot_mode_e;

typedef struct
{
	float vx_set;//底盘x轴方向设定的速度控制量；
	float vy_set;//底盘y轴方向设定的速度控制量
	float wz_set;//底盘自旋时 设定的速度控制量；
  float high_set;//高度控制量
	float yaw_angle_set;//设置yaw轴角度
	float yaw_angle;//yaw轴实时角度
	float yaw_gyro;//yaw轴角速度
  float pitch_angle;//pitch轴实时角度
	RC_chassis_mode_e chassis_mode;//底盘模式
	shoot_mode_e shoot_mode_rc;//射击模式
  uint8_t spin_flag;//小陀螺标志位
  uint8_t tk_flag,jump_flag,cap_flag,sit_flag,high_flag,fric_flag,auto_flag,ui_init_flag,reset_flag;
}chassis_data_t;	



/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


extern void CAN_cmd_referee_data(uint8_t color);
extern void CAN_INIT_STATUS(uint8_t status);
/* -----------------------HT_func------------------------- */
void CAN_HT_CMD( uint8_t id, fp32 f_t );
void CAN_CMD_HT_Enable(uint8_t id, uint8_t unterleib_motor_send_data[8] );
/* -----------------------LK_func------------------------ */
extern void CAN_read_lkmotor_state(void);
extern void CAN_LK_START_control(uint16_t id);
extern void CAN_LK_CLOSE_control(uint16_t id);
extern void CAN_LK_POSITION_Control(int32_t angleControl);
extern void CAN_LK_SPEED_Control(int16_t iqControl,int32_t speedControl);
extern void CAN_LK_Torque_Control(uint16_t id,int16_t iqControl);
void CAN_LK_Boradcast_Control(int16_t iqControl_1,int16_t iqControl_2,int16_t iqControl_3,int16_t iqControl_4);

/* -----------------------Setpower------------------------ */
extern void CAN_Send_Setpower(uint16_t setPower);

/* -----------------------return motor measure----------- */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

HTmotor_measure_t *get_HT_motor_measure_point(uint8_t i);
lkmotor_measure_t *get_LK_motor_measure_point(uint8_t i);
#endif
