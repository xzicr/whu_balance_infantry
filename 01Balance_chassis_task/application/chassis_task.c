/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "vofa.h"
#include "Chassis_power_control.h"
#include "LQR.h"


#define square(x) ((x)*(x))
#define SIGN(x) ((x)>0 ? 1 : ((x)<0? -1: 0))
#define min(a,b) ((a)<(b)?(a):(b))

#define LimitMax(input, max)   \
{                              \
	if ((input) > max)       \
	{                      \
			input = max;   \
	}                      \
	else if ((input) < -max) \
	{                      \
			input = -max;  \
	}                      \
}
#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }
  
#define LimitOutput(input, min, max )  \
{ 						\
	if( (input) < min ) \
		input = min; \
 else if( (input) > max ) \
 input = max; \
}

fp32 angle_change(float angle)
{
  float angle1;
  if (angle >= 0 && angle < 180)
  {
    angle1 = angle;
  }
  else if (angle >= 0 && angle > 180)
  {
    angle1 = angle - 360;
  }
  else if (angle < 0 && angle > -180)
  {
    angle1 = angle;
  }
  else if (angle < 0 && angle < -180)
  {
    angle1 = angle + 360;
  }
  return angle1;
}
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
	int32_t relative_ecd = ecd - offset_ecd;
	if (relative_ecd > HALF_ECD_RANGE)
	{
		relative_ecd -= ECD_RANGE;
	}
	else if (relative_ecd < -HALF_ECD_RANGE)
	{
		relative_ecd += ECD_RANGE;
	}
	
  return relative_ecd * MOTOR_ECD_TO_RAD;
}

// 底盘运动数据
chassis_move_t chassis_move;
feed_type_def feed;
float data[2];

float suspend_foot_speed_p=10.0f;

fp32 TK_x_p = -10.0f, TK_y_p = 10.0f, TK_y_d = 3.0f, reducing_p = 120.0f;

fp32 suspend_LQR[2][6] = {	
	20.0f,5.0f,	0.0f,	0.0f,	0.0f,	0.0f,
	0,	0,  0,  0,	  0,	0
};
/* ------------------------PID info------------------------ */
fp32 roll_PD[2]        = {100.0, 20.0}; // 200, 45
fp32 coordinate_PD[2]  = { 12.0f, 1.0f }; //10.0f,0.5f    //15.0f,1.0f
fp32 yaw_PD_test[2]    = { 20.0f, 180.0f };
fp32 stand_PD[2]       = {152.0f,2.0f};//{ 200.0f, 50.0f };
fp32 jump_stand_PD[2]       = { 10000.0f, 150.0f };
fp32 suspend_stand_PD[2] = { 100.0f, 30.0f };

/* ------------------------平步数据------------------------ */
float alpha_dx = 0.95f;

fp32 IDEAL_PREPARING_STAND_JUMPING_ANGLE = 0.6f;

fp32 stablize_foot_speed_threshold = 1.2f, stablize_yaw_speed_threshold = 1.5f,rotate_move_threshold = 45.0f;
uint8_t robot_level = 1;
fp32 rotate_speed_list[11] = {0.0, 5.0, 5.3, 5.6, 6.0, 6.0, 7.0, 8.0, 9.0, 10.0, 12.0}; 
fp32 move_scale_list[11] =     { 0.0, 1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
fp32 rotate_move_scale_list[11] = {0.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8};

uint8_t no_follow_flag, follow_angle;

int target_speed_sign = 1;

fp32 Moving_High_Offset = 0.0f;
fp32 HIGH_HIGH = 0.35f;
fp32 SIT_HIGH = 0.101f;
fp32 NORMAL_HIGH = 0.12f; 
fp32 High_Offset = 0.04f;

fp32 rollP, rollD, roll_angle_deadband = 0.0f, roll_gyro_deadband = 0.0f, leg_dlength_deadband = 0.0f;
fp32 offset_k = 0.31f;
fp32 rc_sign = 1.0f,fake_sign = 1.0f;
fp32 rc_angle, rc_angle_temp,X_speed, Y_speed,temp_max_spd,rotate_move_offset,delta_theta,delta_theta_temp,acc_step = 0.3f;
fp32 stepp = 0.2;
fp32 normal_move_scale=0.01f;
fp32 normalized_speed;
/* ----------------------function definition ------------------------ */
void Joint_Motor_to_Init_Pos(void);
void Motor_Zero_CMD_Send(void);
void HT_Motor_zero_set(void);
void Forward_kinematic_solution(chassis_move_t *feedback_update,fp32 Q1,fp32 S1,fp32 Q4,fp32 S4, uint8_t ce);

void chassis_init(chassis_move_t *chassis_move_init);
void chassis_feedback_update(chassis_move_t *chassis_move_update);
void Chassis_Status_Detect( chassis_move_t *detect );
void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
void Target_Value_Set( chassis_move_t *target_value_set );
void Chassis_Torque_Calculation(chassis_move_t *bl_ctrl);
void Chassis_Torque_Combine(chassis_move_t *bl_ctrl);
void Motor_CMD_Send(chassis_move_t *CMD_Send);

void chassis_set_contorl(chassis_move_t *chassis_move_control);

void chassis_task(void const *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);
	
	while (1)
	{
		
		chassis_feedback_update(&chassis_move);
		Chassis_Status_Detect(&chassis_move);
		chassis_set_mode(&chassis_move);
		chassis_mode_change_control_transit(&chassis_move);
		Target_Value_Set(&chassis_move);
		Chassis_Torque_Calculation(&chassis_move);
		Chassis_Torque_Combine(&chassis_move);
    // chassis data update
    // 底盘数据更新
    // set chassis control set-point
    // 底盘控制量设置


    Motor_CMD_Send(&chassis_move);

    vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
    chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}


static void chassis_init(chassis_move_t *chassis_move_init)
{
  if (chassis_move_init == NULL)
  {
    return;
  }

	/*----------------------- Set HT Zero Point ---------------------- */

	vTaskDelay(100);
	HT_Motor_zero_set();
	Motor_Zero_CMD_Send();
	vTaskDelay(1);
	CAN_LK_START_control(0x143);
	CAN_LK_START_control(0x142);
  /* -------------------------param get---------------------------- */
	chassis_move_init->joint_motor_1.motor_measure = get_HT_motor_measure_point(0);
	chassis_move_init->joint_motor_2.motor_measure = get_HT_motor_measure_point(1);
	chassis_move_init->joint_motor_3.motor_measure = get_HT_motor_measure_point(2);
	chassis_move_init->joint_motor_4.motor_measure = get_HT_motor_measure_point(3);
	chassis_move_init->foot_motor_L.motor_measure = get_LK_motor_measure_point(0);
	chassis_move_init->foot_motor_R.motor_measure = get_LK_motor_measure_point(1);
  
  chassis_move_init->chassis_INS_angle = get_INS_angle_point();
  chassis_move_init->chassis_INS_gyro = get_gyro_data_point();
  chassis_move_init->chassis_data_ = get_Chassisdata_point();
  chassis_move_init->gimbal_lkmotor_measure = get_yaw_gimbal_lkmotor_measure_point();
  
  
  /* ----------------------------Mode set --------------------------- */
//   chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;

  	chassis_move_init->mode.chassis_mode = chassis_move_init->mode.last_chassis_mode = DISABLE_CHASSIS;
	chassis_move_init->mode.chassis_balancing_mode = chassis_move_init->mode.last_chassis_balancing_mode = NO_FORCE;
	chassis_move_init->mode.sport_mode = chassis_move_init->mode.last_sport_mode = NORMAL_MOVING_MODE;
	chassis_move_init->joint_motor_1.motor_mode = chassis_move_init->joint_motor_1.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->joint_motor_2.motor_mode = chassis_move_init->joint_motor_2.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->joint_motor_3.motor_mode = chassis_move_init->joint_motor_3.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->joint_motor_4.motor_mode = chassis_move_init->joint_motor_4.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->foot_motor_L.motor_mode = chassis_move_init->foot_motor_L.last_motor_mode = MOTOR_NO_FORCE;
	chassis_move_init->foot_motor_R.motor_mode = chassis_move_init->foot_motor_R.last_motor_mode = MOTOR_NO_FORCE;

	/* ----------------------------------INIT HT/LK MOTOR-------------------------------- */
	chassis_move_init->joint_motor_1.position_offset = chassis_move_init->joint_motor_1.motor_measure->ecd;
	chassis_move_init->joint_motor_2.position_offset = chassis_move_init->joint_motor_2.motor_measure->ecd;
	chassis_move_init->joint_motor_3.position_offset = chassis_move_init->joint_motor_3.motor_measure->ecd;
	chassis_move_init->joint_motor_4.position_offset = chassis_move_init->joint_motor_4.motor_measure->ecd;
	chassis_move_init->joint_motor_1.position = (chassis_move_init->joint_motor_1.motor_measure->ecd -chassis_move_init->joint_motor_1.position_offset) + LEG_OFFSET;
	chassis_move_init->joint_motor_2.position = (chassis_move_init->joint_motor_2.motor_measure->ecd -chassis_move_init->joint_motor_2.position_offset) - LEG_OFFSET;
	chassis_move_init->joint_motor_3.position = (chassis_move_init->joint_motor_3.motor_measure->ecd -chassis_move_init->joint_motor_3.position_offset) + LEG_OFFSET;
	chassis_move_init->joint_motor_4.position = (chassis_move_init->joint_motor_4.motor_measure->ecd -chassis_move_init->joint_motor_4.position_offset) - LEG_OFFSET;
  	chassis_move_init->foot_motor_L.distance_offset = ( chassis_move_init->foot_motor_L.position/360.0f ) * WHEEL_PERIMETER;
	chassis_move_init->foot_motor_R.distance_offset = ( (360.0f-chassis_move_init->foot_motor_R.position/360.0f) ) * WHEEL_PERIMETER;
	
	/* ----------------------------------VMC J-------------------------------- */
	 // N11 系数
    chassis_move_init->InverseJacobianCoefficient.N11.c0 = 0.1226f;//-0.0046f;
    chassis_move_init->InverseJacobianCoefficient.N11.c1 = -1.824f;//0.0074f;
    chassis_move_init->InverseJacobianCoefficient.N11.c2 = -0.08976f;//-0.0027f;
    chassis_move_init->InverseJacobianCoefficient.N11.c3 = 3.55f;//0.0076f;
    chassis_move_init->InverseJacobianCoefficient.N11.c4 = -0.2468f;//0.0133f;
    chassis_move_init->InverseJacobianCoefficient.N11.c5 = 0.0434;//0.0006f;
		
		// N12 系数
    chassis_move_init->InverseJacobianCoefficient.N12.c0 = 0.05869f;//0.0614f;
    chassis_move_init->InverseJacobianCoefficient.N12.c1 = 2.473f;//-0.2012f;
    chassis_move_init->InverseJacobianCoefficient.N12.c2 = -0.6447f;//0.0776f;
    chassis_move_init->InverseJacobianCoefficient.N12.c3 = -4.797f;//0.4127f;
    chassis_move_init->InverseJacobianCoefficient.N12.c4 = 2.438f;//-0.2497f;
    chassis_move_init->InverseJacobianCoefficient.N12.c5 = -0.003233f;//-0.0021f;
    
    // N21 系数
    chassis_move_init->InverseJacobianCoefficient.N21.c0 = 0.1226f;//-1.2671f;
    chassis_move_init->InverseJacobianCoefficient.N21.c1 = -1.824f;//7.6106f;
    chassis_move_init->InverseJacobianCoefficient.N21.c2 = -0.08976f;//-0.1929f;
    chassis_move_init->InverseJacobianCoefficient.N21.c3 = 3.55f;//-11.8850f;
    chassis_move_init->InverseJacobianCoefficient.N21.c4 = 0.2468f;//0.9325f;
    chassis_move_init->InverseJacobianCoefficient.N21.c5 = 0.0434f;//0.0488f;
    
    // N22 系数
    chassis_move_init->InverseJacobianCoefficient.N22.c0 = -0.05869f;//0.0038f;
    chassis_move_init->InverseJacobianCoefficient.N22.c1 = -2.473f;//-0.0392f;
    chassis_move_init->InverseJacobianCoefficient.N22.c2 = -0.6447f;//-0.0004f;
    chassis_move_init->InverseJacobianCoefficient.N22.c3 = 4.797f;//0.0767f;
		chassis_move_init->InverseJacobianCoefficient.N22.c4 = 2.438f;//0.0013f;
		chassis_move_init->InverseJacobianCoefficient.N22.c5 = 0.003233f;//0;


    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
  
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  //  chassis_move_init

  // initialize angle PID
  // 初始化角度PID
  PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
  // 用一阶滤波代替斜波函数生成
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

  chassis_move_init->flag_info.init_flag = 1;
  chassis_feedback_update(chassis_move_init);
  chassis_move_init->flag_info.init_flag = 0;
}


static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }
  /* --------------------------------set chassis mode -------------------------------- */
	if( chassis_move_mode->chassis_data_->chassis_mode == CHASSIS_MODE_OFF )
		chassis_move_mode->mode.chassis_mode = DISABLE_CHASSIS;
	else 
		chassis_move_mode->mode.chassis_mode = ENABLE_CHASSIS;
	
	if( chassis_move_mode->mode.chassis_mode == ENABLE_CHASSIS )
	{
		chassis_move_mode->joint_motor_1.motor_mode = MOTOR_FORCE;
		chassis_move_mode->joint_motor_2.motor_mode = MOTOR_FORCE;
		chassis_move_mode->joint_motor_3.motor_mode = MOTOR_FORCE;
		chassis_move_mode->joint_motor_4.motor_mode = MOTOR_FORCE;
		chassis_move_mode->foot_motor_L.motor_mode  = MOTOR_FORCE;
		chassis_move_mode->foot_motor_R.motor_mode  = MOTOR_FORCE;
	}
	else
	{
		if( chassis_move_mode->mode.chassis_balancing_mode == JOINT_REDUCING )
		{
			chassis_move_mode->joint_motor_1.motor_mode = MOTOR_FORCE;
			chassis_move_mode->joint_motor_2.motor_mode = MOTOR_FORCE;
			chassis_move_mode->joint_motor_3.motor_mode = MOTOR_FORCE;
			chassis_move_mode->joint_motor_4.motor_mode = MOTOR_FORCE;
			chassis_move_mode->foot_motor_L.motor_mode  = MOTOR_NO_FORCE;
			chassis_move_mode->foot_motor_R.motor_mode  = MOTOR_NO_FORCE;
		}
		else
		{
			chassis_move_mode->joint_motor_1.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->joint_motor_2.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->joint_motor_3.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->joint_motor_4.motor_mode = MOTOR_NO_FORCE;
			chassis_move_mode->foot_motor_L.motor_mode  = MOTOR_NO_FORCE;
			chassis_move_mode->foot_motor_R.motor_mode  = MOTOR_NO_FORCE;
		}

	}

	/*-------------------------- Sport Mode Update ----------------------------------*/ 
	if( chassis_move_mode->mode.chassis_balancing_mode == BALANCING_READY )
	{
    /* 之后再考虑添加 */
		if( chassis_move_mode->chassis_data_->tk_flag )
			chassis_move_mode->mode.sport_mode = TK_MODE;
		else if( chassis_move_mode->flag_info.abnormal_flag )
			chassis_move_mode->mode.sport_mode = ABNORMAL_MOVING_MODE;
		else if( chassis_move_mode->mode.sport_mode == JUMPING_MODE && chassis_move_mode->mode.jumping_stage != FINISHED )
			chassis_move_mode->mode.sport_mode = JUMPING_MODE;
		else if( chassis_move_mode->chassis_data_->jump_flag )
			chassis_move_mode->mode.sport_mode = JUMPING_MODE;
		else if( chassis_move_mode->chassis_data_->cap_flag )
			chassis_move_mode->mode.sport_mode = CAP_MODE;
		else if( chassis_move_mode->chassis_data_->fly_flag )
			chassis_move_mode->mode.sport_mode = FLY_MODE;
		else 
			chassis_move_mode->mode.sport_mode = NORMAL_MOVING_MODE;
	}
	else
		chassis_move_mode->mode.sport_mode = NONE;

	/* ----------------- Rotation Flag --------------------*/
	static uint8_t last_rotation_flag = 0;
	last_rotation_flag = chassis_move_mode->flag_info.rotation_flag;
	chassis_move_mode->flag_info.rotation_flag = chassis_move_mode->chassis_data_->chassis_mode == CHASSIS_MODE_ROTATE;////4==/Rotate
	if (!last_rotation_flag && chassis_move_mode->flag_info.rotation_flag){
		for (int i = 0; i < 11; ++i)
			rotate_speed_list[i] = - rotate_speed_list[i];
	}

	/* ----------------- No Follow Flag ----------------------*/
	if (chassis_move_mode->chassis_data_->chassis_mode==CHASSIS_MODE_NO_FOLLOW) no_follow_flag = 1;
	else if(chassis_move_mode->chassis_data_->chassis_mode==CHASSIS_MODE_FOLLOW)no_follow_flag = 0; //No Follow
	
	/*---------------- Moving Flag ----------------------------*/
	if( chassis_move_mode->chassis_data_->vx_set != 0.0f )
	{
		chassis_move_mode->flag_info.controlling_flag = 1;
		chassis_move_mode->flag_info.set_pos_after_moving = 1;
	}	
	else 
		chassis_move_mode->flag_info.controlling_flag = 0;
		
	if( fabs(chassis_move_mode->chassis_posture_info.foot_speed_K) <= 0.05f &&
		!chassis_move_mode->flag_info.controlling_flag &&
		chassis_move_mode->flag_info.set_pos_after_moving )  					// maybe bug
		{
			chassis_move_mode->flag_info.moving_flag = 0;
			chassis_move_mode->flag_info.set_pos_after_moving = 0;
		}
	else 
		chassis_move_mode->flag_info.moving_flag = 1;


}

uint8_t reduce_flag = 0;
fp32 reduce_high, high_offset = 0.2f;
fp32 debug_1 = 0.995;
static void chassis_mode_change_control_transit(chassis_move_t *chassis_mode_change)
{
  if (chassis_mode_change == NULL)
  {
    return;
  }
  /* --------------------------------使能/失能 后判断是否 进入/退出 平衡模式--------------------------------  */
	if( chassis_mode_change->mode.chassis_mode == ENABLE_CHASSIS && chassis_mode_change->mode.last_chassis_mode == DISABLE_CHASSIS )
		chassis_mode_change->mode.chassis_balancing_mode = FOOT_LAUNCHING;

	if( chassis_mode_change->mode.chassis_balancing_mode == FOOT_LAUNCHING && 
		fabs(chassis_mode_change->chassis_posture_info.pitch_angle) < EXIT_PITCH_ANGLE )
		chassis_mode_change->mode.chassis_balancing_mode = JOINT_LAUNCHING;
	else if( chassis_mode_change->mode.chassis_balancing_mode == JOINT_LAUNCHING )//&& (
		// (NORMAL_HIGH - chassis_mode_change->chassis_posture_info.leg_length_L) < 0.03f ||
		// (NORMAL_HIGH - chassis_mode_change->chassis_posture_info.leg_length_R) < 0.03f) )
		chassis_mode_change->mode.chassis_balancing_mode = BALANCING_READY;
	else if( chassis_mode_change->mode.chassis_balancing_mode == BALANCING_READY &&
		chassis_mode_change->mode.chassis_mode == DISABLE_CHASSIS ){
			if (!reduce_flag) {
				reduce_high = chassis_mode_change->chassis_posture_info.ideal_high + high_offset;
				reduce_flag = 1;
			}
			chassis_mode_change->mode.chassis_balancing_mode = JOINT_REDUCING;
		}
	else if( chassis_mode_change->mode.chassis_balancing_mode == JOINT_REDUCING && (
		fabs( chassis_mode_change->chassis_posture_info.leg_length_L - SIT_HIGH ) < 0.001f ||
		fabs( chassis_mode_change->chassis_posture_info.leg_length_R - SIT_HIGH ) < 0.001f ) ){
			chassis_mode_change->mode.chassis_balancing_mode = NO_FORCE;
			reduce_flag = 0;
		}
  /* ---------------------目前尚未开发运动模式，不存在判断跳跃状态---------------------------- */
	if( chassis_mode_change->mode.sport_mode == JUMPING_MODE && chassis_mode_change->mode.last_sport_mode != JUMPING_MODE )
	{
		chassis_mode_change->mode.jumping_mode = STANDING_JUMP;
	}
	else if( chassis_mode_change->mode.sport_mode != JUMPING_MODE )
		chassis_mode_change->mode.jumping_mode = NOT_DEFINE;
	

	if( chassis_mode_change->mode.jumping_mode == MOVING_JUMP )
	{
		if( chassis_mode_change->mode.jumping_stage == READY_TO_JUMP )
			chassis_mode_change->mode.jumping_stage = CONSTACTING_LEGS;
		else if( chassis_mode_change->mode.jumping_stage == CONSTACTING_LEGS &&
			chassis_mode_change->chassis_posture_info.leg_length_L <= 0.13f )
				chassis_mode_change->mode.jumping_stage = EXTENDING_LEGS;
		else if( chassis_mode_change->mode.jumping_stage == EXTENDING_LEGS  && 
			chassis_mode_change->chassis_posture_info.leg_length_L >= 0.30f )
				chassis_mode_change->mode.jumping_stage = CONSTACTING_LEGS_2;
		else if( chassis_mode_change->mode.jumping_stage == CONSTACTING_LEGS_2  && 
			chassis_mode_change->chassis_posture_info.leg_length_L <= 0.13f )
				chassis_mode_change->mode.jumping_stage = PREPARING_LANDING;
		else if( chassis_mode_change->mode.jumping_stage == PREPARING_LANDING && 
			chassis_mode_change->flag_info.suspend_flag_R == ON_GROUND &&
			chassis_mode_change->flag_info.suspend_flag_L == ON_GROUND )
				chassis_mode_change->mode.jumping_stage = FINISHED;
		else if( chassis_mode_change->mode.jumping_stage == FINISHED )
				chassis_mode_change->mode.jumping_stage = READY_TO_JUMP;
	}
	else if( chassis_mode_change->mode.jumping_mode == STANDING_JUMP )
	{
		if( chassis_mode_change->mode.jumping_stage == READY_TO_JUMP )
			chassis_mode_change->mode.jumping_stage = PREPARING_STAND_JUMPING;
		else if(chassis_mode_change->mode.jumping_stage == PREPARING_STAND_JUMPING &&
			fabs(chassis_mode_change->chassis_posture_info.leg_angle_L + IDEAL_PREPARING_STAND_JUMPING_ANGLE ) < stepp )
			chassis_mode_change->mode.jumping_stage = EXTENDING_LEGS;
		else if( chassis_mode_change->mode.jumping_stage == EXTENDING_LEGS  && 
			chassis_mode_change->chassis_posture_info.leg_length_L >= 0.30f )
				chassis_mode_change->mode.jumping_stage = CONSTACTING_LEGS_2;
		else if( chassis_mode_change->mode.jumping_stage == CONSTACTING_LEGS_2  && 
			chassis_mode_change->chassis_posture_info.leg_length_L <= 0.13f )
				chassis_mode_change->mode.jumping_stage = PREPARING_LANDING;
		else if( chassis_mode_change->mode.jumping_stage == PREPARING_LANDING && 
			chassis_mode_change->flag_info.suspend_flag_R == ON_GROUND &&
			chassis_mode_change->flag_info.suspend_flag_L == ON_GROUND )
				chassis_mode_change->mode.jumping_stage = FINISHED;
		else if( chassis_mode_change->mode.jumping_stage == FINISHED )
				chassis_mode_change->mode.jumping_stage = READY_TO_JUMP;
	}
	else
		chassis_mode_change->mode.jumping_stage = FINISHED; 


	if( chassis_mode_change->flag_info.stablize_high_flag == 1 &&
		Moving_High_Offset >= 0.2 &&
		chassis_mode_change->chassis_posture_info.yaw_gyro <= ( stablize_yaw_speed_threshold - 0.5f ) )
			chassis_mode_change->flag_info.stablize_high_flag = 0;
}

fp32 HIGH_SWITCH = 36.0f;

void Target_Value_Set( chassis_move_t *target_value_set )
{
	/*------------------------------------定腿高判断----------------------------------*/
	if( target_value_set->flag_info.stablize_high_flag == 1 )
		if( Moving_High_Offset < 0.2 )
			Moving_High_Offset += 0.001f;

	if( target_value_set->flag_info.stablize_high_flag == 0 )
		Moving_High_Offset = 0.0f;

	/* ----------------------------------- X Speed Set -----------------------------------*/
	if( target_value_set->mode.sport_mode             != NONE                 && 
		target_value_set->flag_info.suspend_flag_L     == ON_GROUND &&
		target_value_set->flag_info.suspend_flag_R     == ON_GROUND )
	{
		if (!target_value_set->flag_info.rotation_flag) {
			if(toe_is_error(REFEREE_TOE)){
				target_value_set->chassis_posture_info.foot_speed_set = target_value_set->chassis_data_->vx_set * normal_move_scale ;

			} else {
				// target_speed_sign = (fake_sign * target_value_set->chassis_rc_ctrl->X_speed * move_scale_list[robot_level]) > 0 ? 1: -1;
				target_speed_sign = SIGN(target_value_set->chassis_data_->vx_set * move_scale_list[robot_level]);					
				target_value_set->chassis_posture_info.foot_speed_set = target_value_set->chassis_posture_info.foot_speed_K + target_speed_sign * acc_step;

				if (target_value_set->mode.sport_mode == FLY_MODE) {}
					// target_value_set->chassis_posture_info.foot_speed_set = target_speed_sign * min(ABS(target_value_set->chassis_rc_ctrl->X_speed * fly_speed),ABS(target_value_set->chassis_posture_info.foot_speed_set));
				else if (target_value_set->mode.sport_mode == CAP_MODE){}
					// target_value_set->chassis_posture_info.foot_speed_set = target_speed_sign * min(ABS(target_value_set->chassis_rc_ctrl->X_speed * cap_move_scale_list[robot_level]),ABS(target_value_set->chassis_posture_info.foot_speed_set));
				else{}
			  		// target_value_set->chassis_posture_info.foot_speed_set = target_speed_sign * min(ABS(target_value_set->chassis_rc_ctrl->X_speed * move_scale_list[robot_level]),ABS(target_value_set->chassis_posture_info.foot_speed_set));

				// if( target_value_set->chassis_posture_info.leg_length_L > 0.22f || target_value_set->chassis_posture_info.leg_length_R > 0.22f )
				// {
				// 	target_value_set->chassis_posture_info.foot_speed_set /= 2.0f;
				// 	debug_2++;
				// }	
			}
		
		}
		else // Rotate and move
		{
			delta_theta_temp = target_value_set->chassis_data_->wz_set - rc_angle;
			if (delta_theta_temp>PI/2) delta_theta_temp -= PI;
			else if (delta_theta_temp<-PI/2) delta_theta_temp += PI;
			delta_theta = fabs(delta_theta_temp);
			if (delta_theta<rotate_move_threshold)
				target_value_set->chassis_posture_info.foot_speed_set
				= ((rotate_move_threshold-delta_theta)/rotate_move_threshold)
				 * rc_sign * fake_sign * normalized_speed * rotate_move_scale_list[robot_level];
			else target_value_set->chassis_posture_info.foot_speed_set = 0;
		}
	}
	else
		target_value_set->chassis_posture_info.foot_speed_set = 0;

	// --------- Distance Set ---------
	if( target_value_set->mode.chassis_balancing_mode == NO_FORCE || target_value_set->mode.sport_mode == TK_MODE)
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	else if( target_value_set->mode.sport_mode == ABNORMAL_MOVING_MODE )
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_set;
	else if( target_value_set->flag_info.suspend_flag_R == OFF_GROUND ||
		target_value_set->flag_info.suspend_flag_L == OFF_GROUND )
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	else if( target_value_set->flag_info.controlling_flag )
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	else if( !target_value_set->flag_info.moving_flag && 
			target_value_set->mode.chassis_balancing_mode == BALANCING_READY &&
			target_value_set->flag_info.last_moving_flag )
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	

	// --------- Y Speed & Angle Set --------- 
	if( target_value_set->mode.sport_mode             != NONE    && 
		target_value_set->flag_info.suspend_flag_L      == ON_GROUND &&
		target_value_set->flag_info.suspend_flag_R      == ON_GROUND )
	{
	 	if( target_value_set->flag_info.rotation_flag == 1 )
	 	{
	 	 	target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle;
	 	 	target_value_set->chassis_posture_info.yaw_gyro_set = rotate_speed_list[robot_level];
	 	} 
	 	else
	 	{
			if (target_value_set->chassis_data_->wz_set>-CHASSIS_RC_DEADLINE) 
				target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle + (target_value_set->chassis_data_->wz_set*0.001f) ;
			else if (target_value_set->chassis_data_->wz_set<CHASSIS_RC_DEADLINE) 
				target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle + (target_value_set->chassis_data_->wz_set*0.001f) ;
			else 
				target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle;
			target_value_set->chassis_posture_info.yaw_gyro_set = 0.0f;
		}
	}
	else
	{
	 	target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle;
	 	target_value_set->chassis_posture_info.yaw_gyro_set = target_value_set->chassis_posture_info.yaw_gyro;
	}



	// --------- Side Angle Set ---------
	// if( target_value_set->mode.sport_mode         != NONE                 &&
	// 	target_value_set->mode.sport_mode         != ABNORMAL_MOVING_MODE && 
	// 	target_value_set->flag_info.suspend_flag_R   == ON_GROUND &&
	// 	target_value_set->flag_info.suspend_flag_L   == ON_GROUND )
	// {
	// 	if( target_value_set->mode.sport_mode == SIDE_MODE )
	// 		target_value_set->chassis_posture_info.roll_angle_set = (fp32)(5.0 / 180.0 * PI) * 50.0f;
	// 	// else if( target_value_set->chassis_rc_ctrl->side_flag == -1 )
	// 	// 	target_value_set->chassis_posture_info.roll_angle_set = -(fp32)(5.0 / 180.0 * PI) * 50.0f;
	// 	else	
	// 		target_value_set->chassis_posture_info.roll_angle_set = 0.0f;
	// }
	// else
		target_value_set->chassis_posture_info.roll_angle_set = 0.0f;

	// ---------------- Leg Angle Set ----------
	if( target_value_set->mode.jumping_stage == PREPARING_STAND_JUMPING )
	{
		target_value_set->chassis_posture_info.leg_angle_L_set = -IDEAL_PREPARING_STAND_JUMPING_ANGLE;
		target_value_set->chassis_posture_info.leg_angle_R_set = -IDEAL_PREPARING_STAND_JUMPING_ANGLE;
	}
	else
	{
		target_value_set->chassis_posture_info.leg_angle_L_set = 0.0f;
		target_value_set->chassis_posture_info.leg_angle_R_set = 0.0f;
	}
	

	// ----------------- Chassis High Mode Update ----------------- 
	if( target_value_set->mode.sport_mode == ABNORMAL_MOVING_MODE || target_value_set->mode.sport_mode == TK_MODE)
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;
	else if( target_value_set->mode.chassis_balancing_mode == FOOT_LAUNCHING )
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if( target_value_set->mode.chassis_balancing_mode == JOINT_LAUNCHING )
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;
	else if( target_value_set->mode.chassis_balancing_mode == JOINT_REDUCING )
		target_value_set->mode.chassis_high_mode = CHANGING_HIGH;
	else if( target_value_set->chassis_data_->sit_flag )
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if( target_value_set->chassis_data_->high_flag )
		target_value_set->mode.chassis_high_mode = HIGH_MODE;
	else if( target_value_set->mode.jumping_stage == CONSTACTING_LEGS )
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if( target_value_set->mode.jumping_stage == EXTENDING_LEGS )
		target_value_set->mode.chassis_high_mode = HIGH_MODE;
	else if( target_value_set->mode.jumping_stage == CONSTACTING_LEGS_2 )
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if( target_value_set->mode.jumping_stage == PREPARING_LANDING )
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;
	else
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;

	// --------- Leg Length Set --------- 
	if( target_value_set->mode.chassis_high_mode == SIT_MODE )
		target_value_set->chassis_posture_info.ideal_high = SIT_HIGH;
	else if( target_value_set->mode.chassis_high_mode == NORMAL_MODE )
		target_value_set->chassis_posture_info.ideal_high = NORMAL_HIGH + High_Offset ;//- Moving_High_Offset;
	else if( target_value_set->mode.chassis_high_mode == HIGH_MODE )
		target_value_set->chassis_posture_info.ideal_high = HIGH_HIGH + High_Offset - Moving_High_Offset;
	else if( target_value_set->mode.chassis_high_mode == CHANGING_HIGH )
	{
		reduce_high = reduce_high * debug_1;
		target_value_set->chassis_posture_info.ideal_high = min(reduce_high,target_value_set->chassis_posture_info.ideal_high);
			// target_value_set->chassis_posture_info.ideal_high = 
			// 	debug_1 * target_value_set->chassis_posture_info.ideal_high;
	}

	if( target_value_set->mode.sport_mode == ABNORMAL_MOVING_MODE ||
		( target_value_set->flag_info.suspend_flag_L == 1 && target_value_set->flag_info.suspend_flag_R == 1 ) ||
		target_value_set->chassis_posture_info.ideal_high == SIT_HIGH ||
		target_value_set->mode.chassis_balancing_mode == JOINT_REDUCING )
	{
		target_value_set->chassis_posture_info.leg_length_L_set = target_value_set->chassis_posture_info.ideal_high;
		target_value_set->chassis_posture_info.leg_length_R_set = target_value_set->chassis_posture_info.ideal_high;
	}
	else
	{
		target_value_set->chassis_posture_info.foot_roll_angle = 
			target_value_set->chassis_posture_info.roll_angle +
			atan(( target_value_set->chassis_posture_info.leg_length_L - target_value_set->chassis_posture_info.leg_length_R ) / 0.50f );
		
		target_value_set->chassis_posture_info.leg_length_L_set = 
			target_value_set->chassis_posture_info .ideal_high ;
			//+ 0.25f * arm_sin_f32( target_value_set->chassis_posture_info .foot_roll_angle ) / arm_cos_f32( target_value_set->chassis_posture_info .foot_roll_angle );
		target_value_set->chassis_posture_info.leg_length_R_set = 
			target_value_set->chassis_posture_info .ideal_high;
			//- 0.25f * arm_sin_f32( target_value_set->chassis_posture_info .foot_roll_angle ) / arm_cos_f32( target_value_set->chassis_posture_info .foot_roll_angle );
	}
		
}
void Chassis_Torque_Calculation(chassis_move_t *bl_ctrl)
{

	LQR_Data_Update(bl_ctrl);
	/*----------------------------Roll Balance PD-------------------------*/
	if( bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1 ||
		bl_ctrl->mode.chassis_high_mode == SIT_MODE )
	{
		bl_ctrl->torque_info.joint_roll_torque_R = 0.0f;
		bl_ctrl->torque_info.joint_roll_torque_L = 0.0f;
	}
	else
	{
		if (bl_ctrl->chassis_posture_info.roll_angle < -roll_angle_deadband) {
			rollP = roll_PD[0] * (bl_ctrl->chassis_posture_info.roll_angle_set - (bl_ctrl->chassis_posture_info.roll_angle+roll_angle_deadband));
		} 
		else if (bl_ctrl->chassis_posture_info.roll_angle > roll_angle_deadband){
			rollP = roll_PD[0] * (bl_ctrl->chassis_posture_info.roll_angle_set - (bl_ctrl->chassis_posture_info.roll_angle-roll_angle_deadband));
		}
		else {
			rollP = 0.0f;
		}

		if (bl_ctrl->chassis_posture_info.roll_gyro < -roll_gyro_deadband) {
			rollD = roll_PD[1] * - (bl_ctrl->chassis_posture_info.roll_gyro + roll_gyro_deadband);
		} 
		else if (bl_ctrl->chassis_posture_info.roll_gyro > roll_gyro_deadband){
			rollD = roll_PD[1] * - (bl_ctrl->chassis_posture_info.roll_gyro - roll_gyro_deadband);
		}
		else {
			rollD = 0.0f;
		}
		// bl_ctrl->torque_info.joint_roll_torque_R = rollP + rollD;
		// bl_ctrl->torque_info.joint_roll_torque_L = -bl_ctrl->torque_info.joint_roll_torque_R;
	}
	

	/*----------------------During turns: prevent displacement of two legs   PD-------------*/ 
	bl_ctrl->torque_info.joint_prevent_splits_torque_L = 
		coordinate_PD[0] * (bl_ctrl->chassis_posture_info.leg_angle_L - bl_ctrl->chassis_posture_info.leg_angle_R)
		+ coordinate_PD[1] * (bl_ctrl->chassis_posture_info.leg_gyro_L - bl_ctrl->chassis_posture_info.leg_gyro_R);

	bl_ctrl->torque_info.joint_prevent_splits_torque_R = -	bl_ctrl->torque_info.joint_prevent_splits_torque_L;

	// if( bl_ctrl->mode.jumping_stage == EXTENDING_LEGS ||
	// 	bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2 )
	// {
	// 	bl_ctrl->torque_info.joint_stand_torque_L = 
	// 		+ jump_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
	// 		+ jump_stand_PD[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_L );
	
	// 	bl_ctrl->torque_info.joint_stand_torque_R = 
	// 		+ jump_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
	// 		+ jump_stand_PD[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_R );
	// }
	// else if( bl_ctrl->mode.sport_mode == ABNORMAL_MOVING_MODE )
	// {
	// 	bl_ctrl->torque_info.joint_stand_torque_L = 
	// 			+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
	// 			+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_L );
	// 	bl_ctrl->torque_info.joint_stand_torque_R = 
	// 			+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
	// 			+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_R );
	// }
	// else if( bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1 )
	// {
	// 	if( bl_ctrl->flag_info.suspend_flag_L == 1 )
	// 	{
	// 		bl_ctrl->torque_info.joint_stand_torque_L = 
	// 			+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
	// 			+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_L );
	// 	}
	// 	else{
	// 		bl_ctrl->torque_info.joint_stand_torque_L = 
	// 			FEED_f
	// 			+ stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
	// 			+ stand_PD[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_L );
	// 	}

	// 	if( bl_ctrl->flag_info.suspend_flag_R == 1 )
	// 	{
	// 		bl_ctrl->torque_info.joint_stand_torque_R = 
	// 			+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
	// 			+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_R );
	// 	} else {
	// 		bl_ctrl->torque_info.joint_stand_torque_R = 
	// 			FEED_f
	// 			+ stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
	// 			+ stand_PD[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_R );
	// 	}
	// }
	// else if( bl_ctrl->mode.chassis_balancing_mode == JOINT_REDUCING )
	// {

	// 	bl_ctrl->torque_info.joint_stand_torque_L = 
	// 		+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
	// 		+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_L );
	
	// 	bl_ctrl->torque_info.joint_stand_torque_R = 
	// 		+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
	// 		+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_R );
	// }
	
	
		bl_ctrl->torque_info.joint_stand_torque_L = FEED_f;
		bl_ctrl->torque_info.joint_stand_torque_L += stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L );
		if (bl_ctrl->chassis_posture_info.leg_dlength_L > leg_dlength_deadband){
			bl_ctrl->torque_info.joint_stand_torque_L += stand_PD[1] * ( 0 - (bl_ctrl->chassis_posture_info.leg_dlength_L-leg_dlength_deadband) );
		}
		else if (bl_ctrl->chassis_posture_info.leg_dlength_L < -leg_dlength_deadband){
			bl_ctrl->torque_info.joint_stand_torque_L += stand_PD[1] * ( 0 - (bl_ctrl->chassis_posture_info.leg_dlength_L+leg_dlength_deadband));
		}
		else {
			bl_ctrl->torque_info.joint_stand_torque_L += 0.0f;
		}
		bl_ctrl->torque_info.joint_stand_torque_R =  FEED_f;
		bl_ctrl->torque_info.joint_stand_torque_R += stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R );
		if (bl_ctrl->chassis_posture_info.leg_dlength_R > leg_dlength_deadband){
			bl_ctrl->torque_info.joint_stand_torque_R += stand_PD[1] * ( 0 - (bl_ctrl->chassis_posture_info.leg_dlength_R-leg_dlength_deadband) );
		}
		else if (bl_ctrl->chassis_posture_info.leg_dlength_R < -leg_dlength_deadband){
			bl_ctrl->torque_info.joint_stand_torque_R += stand_PD[1] * ( 0 - (bl_ctrl->chassis_posture_info.leg_dlength_R+leg_dlength_deadband));
		}
		else {
			bl_ctrl->torque_info.joint_stand_torque_R += 0.0f;
		}	
	
	/*-------------------------- joint motor calucaltion -----------------------------*/


	// TODDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDO
	if( bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2 )
	{
		bl_ctrl->torque_info.joint_balancing_torque_L = 
			-suspend_LQR[0][0] * ( 0 - bl_ctrl->chassis_posture_info.leg_angle_L ) 
			-suspend_LQR[0][1] * ( 0 - bl_ctrl->chassis_posture_info.leg_gyro_L );
		bl_ctrl->torque_info.joint_balancing_torque_R = 0.0f;
			// -suspend_LQR[0][0] * ( 0 - bl_ctrl->chassis_posture_info.leg_angle_R )
			// -suspend_LQR[0][1] * ( 0 - bl_ctrl->chassis_posture_info.leg_gyro_R );

		bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
		bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
	}
	else 
	{
		if( bl_ctrl->mode.sport_mode == TK_MODE )//bl_ctrl->mode.sport_mode == ABNORMAL_MOVING_MODE || 
		{
			bl_ctrl->torque_info.joint_balancing_torque_L = 
			bl_ctrl->torque_info.joint_moving_torque_L    =
			bl_ctrl->torque_info.joint_balancing_torque_R =
			bl_ctrl->torque_info.joint_moving_torque_R    = 0;
		}
		else 
		{
			if( bl_ctrl->mode.chassis_high_mode == SIT_MODE ||
				bl_ctrl->mode.chassis_balancing_mode == JOINT_REDUCING )
			{
				bl_ctrl->torque_info.joint_balancing_torque_L = 
				bl_ctrl->torque_info.joint_moving_torque_L    =
				bl_ctrl->torque_info.joint_balancing_torque_R = 
				bl_ctrl->torque_info.joint_moving_torque_R    = 0;
			}
			else
			{
				
				bl_ctrl->torque_info.joint_balancing_torque_L = (
					+LQR[2][4] * ( bl_ctrl->chassis_posture_info.leg_angle_L_set - bl_ctrl->chassis_posture_info.leg_angle_L )
					-LQR[2][5] * (                  0.0f                         - bl_ctrl->chassis_posture_info.leg_gyro_L )
					-LQR[2][8] * ( bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle ) 
					-LQR[2][9] * ( bl_ctrl->chassis_posture_info.pitch_gyro_set  - bl_ctrl->chassis_posture_info.pitch_gyro )
				);
				// bl_ctrl->torque_info.joint_moving_torque_L    = ( 
				// 	+LQR[2][0] * ( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K + NORMAL_MODE_WEIGHT_DISTANCE_OFFSET)
				// 	+LQR[2][1] * ( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.foot_speed_K )
				// 	+LQR[2][2] * ( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle )
				// 	+LQR[2][3] * ( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
				// 	);

				bl_ctrl->torque_info.joint_balancing_torque_R = -(
					LQR[3][6] * ( bl_ctrl->chassis_posture_info.leg_angle_R_set - bl_ctrl->chassis_posture_info.leg_angle_R ) 
					-LQR[3][7] * (                  0.0f                         - bl_ctrl->chassis_posture_info.leg_gyro_R )
					-LQR[3][8] * ( bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle ) 
					-LQR[3][9] * ( bl_ctrl->chassis_posture_info.pitch_gyro_set  - bl_ctrl->chassis_posture_info.pitch_gyro )
				);
				// bl_ctrl->torque_info.joint_moving_torque_R    = ( 
				// 	+LQR[3][0] * ( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K + NORMAL_MODE_WEIGHT_DISTANCE_OFFSET)
				// 	+LQR[3][1] * ( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.foot_speed_K )
				// 	+LQR[3][2] * ( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle )
				// 	+LQR[3][3] * ( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
				// 	);
			}
		}
	}
	// --------------------- Foot motor LQR ---------------------
	if ( bl_ctrl->mode.sport_mode == TK_MODE ) {
		bl_ctrl->torque_info.foot_balancing_torque_L = 0.0f;
		bl_ctrl->torque_info.foot_balancing_torque_R = 0.0f;
		bl_ctrl->torque_info.foot_moving_torque_L = (int) (
			-TK_x_p*( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_speed_K ) 
			-TK_y_p*( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.yaw_angle )
			-TK_y_d*( 0.0f		                                      - bl_ctrl->chassis_posture_info.yaw_gyro )
		)* TORQ_K;
		bl_ctrl->torque_info.foot_moving_torque_R = (int)-(
			-TK_x_p*( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_speed_K ) 
			+TK_y_p*( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.yaw_angle )
			+TK_y_d*( 0.0f		                                      - bl_ctrl->chassis_posture_info.yaw_gyro )
		)* TORQ_K;
	}
	else {
		bl_ctrl->torque_info.foot_balancing_torque_L = (int) ( 
			-LQR[0][4]*( bl_ctrl->chassis_posture_info.leg_angle_L_set - bl_ctrl->chassis_posture_info.leg_angle_L) 
			+LQR[0][5]*(                  0.0f                         - bl_ctrl->chassis_posture_info.leg_gyro_L)
			-LQR[0][8]*( bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle) 
			+LQR[0][9]*( bl_ctrl->chassis_posture_info.pitch_gyro_set - bl_ctrl->chassis_posture_info.pitch_gyro) 
		)* TORQ_K;
		// bl_ctrl->torque_info.foot_moving_torque_L = (int) ( 
		// 	-LQR[0][0]*( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K + NORMAL_MODE_WEIGHT_DISTANCE_OFFSET)*10.0f
		// 	+LQR[0][1]*( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.foot_speed_K) 
		// 	// -LQR[0][2]*( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle)
		// 	// -LQR[0][3]*( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
		// )* TORQ_K;
		bl_ctrl->torque_info.foot_balancing_torque_R = (int)-( 
			-LQR[1][6]*( bl_ctrl->chassis_posture_info.leg_angle_R_set  - bl_ctrl->chassis_posture_info.leg_angle_R) 
			+LQR[1][7]*(                  0.0f                         - bl_ctrl->chassis_posture_info.leg_gyro_R)
			-LQR[1][8]*( bl_ctrl->chassis_posture_info.pitch_angle_set  - bl_ctrl->chassis_posture_info.pitch_angle)
			+LQR[1][9]*( bl_ctrl->chassis_posture_info.pitch_gyro_set - bl_ctrl->chassis_posture_info.pitch_gyro)
		)* TORQ_K;
		// bl_ctrl->torque_info.foot_moving_torque_R =(int) -( 
		// 	-LQR[1][0]*( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K + NORMAL_MODE_WEIGHT_DISTANCE_OFFSET)*10.0f
		//     +LQR[1][1]*( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.foot_speed_K) 			
		// 	// -LQR[1][2]*( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle)
		// 	// +LQR[1][3]*( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
		// )* TORQ_K;
		
	}

	if( bl_ctrl->flag_info.suspend_flag_R == 1 )
	{
		bl_ctrl->torque_info.joint_balancing_torque_R = 0.0f;
			// -suspend_LQR[0][0] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_angle_R )
			// -suspend_LQR[0][1] * ( 0 - bl_ctrl->chassis_posture_info.leg_gyro_R );

		bl_ctrl->torque_info.foot_moving_torque_R = 
			-suspend_foot_speed_p * ( 0.0f - bl_ctrl->foot_motor_R.speed );	

		bl_ctrl->torque_info.joint_moving_torque_R   = 0.0f;
		bl_ctrl->torque_info.foot_balancing_torque_R = 0.0f;
	}
	if( bl_ctrl->flag_info.suspend_flag_L == 1 )
	{
		bl_ctrl->torque_info.joint_balancing_torque_L = 
			-suspend_LQR[0][0] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_angle_L ) 
			-suspend_LQR[0][1] * ( 0 - bl_ctrl->chassis_posture_info.leg_gyro_L );
		bl_ctrl->torque_info.foot_moving_torque_L = 
			+suspend_foot_speed_p * ( 0.0f - bl_ctrl->foot_motor_L.speed );

		bl_ctrl->torque_info.joint_moving_torque_L   = 0.0f;
		bl_ctrl->torque_info.foot_balancing_torque_L = 0.0f;
	}


	// LimitMax( bl_ctrl->torque_info.foot_moving_torque_L,  MAX_ACCL );
	// LimitMax( bl_ctrl->torque_info.foot_moving_torque_R,  MAX_ACCL );
	LimitMax( bl_ctrl->torque_info.joint_moving_torque_L, MAX_ACCL_JOINT );
	LimitMax( bl_ctrl->torque_info.joint_moving_torque_R, MAX_ACCL_JOINT );

}

void Chassis_Torque_Combine(chassis_move_t *bl_ctrl)
{
	bl_ctrl->mapping_info .J1_L = get_jacobian_element(bl_ctrl,bl_ctrl->chassis_posture_info .leg_length_L ,bl_ctrl->chassis_posture_info .leg_angle_L,1);//N11
	bl_ctrl->mapping_info .J2_L = get_jacobian_element(bl_ctrl,bl_ctrl->chassis_posture_info .leg_length_L ,bl_ctrl->chassis_posture_info .leg_angle_L,3);//N21
	bl_ctrl->mapping_info .J3_L = get_jacobian_element(bl_ctrl,bl_ctrl->chassis_posture_info .leg_length_L ,bl_ctrl->chassis_posture_info .leg_angle_L,2);//N12
	bl_ctrl->mapping_info .J4_L = get_jacobian_element(bl_ctrl,bl_ctrl->chassis_posture_info .leg_length_L ,bl_ctrl->chassis_posture_info .leg_angle_L,4);//N22
	bl_ctrl->mapping_info .J1_R = get_jacobian_element(bl_ctrl,bl_ctrl->chassis_posture_info .leg_length_R ,bl_ctrl->chassis_posture_info .leg_angle_R,1);
	bl_ctrl->mapping_info .J2_R = get_jacobian_element(bl_ctrl,bl_ctrl->chassis_posture_info .leg_length_R ,bl_ctrl->chassis_posture_info .leg_angle_R,3); 
	bl_ctrl->mapping_info .J3_R = get_jacobian_element(bl_ctrl,bl_ctrl->chassis_posture_info .leg_length_R ,bl_ctrl->chassis_posture_info .leg_angle_R,2);
	bl_ctrl->mapping_info .J4_R = get_jacobian_element(bl_ctrl,bl_ctrl->chassis_posture_info .leg_length_R ,bl_ctrl->chassis_posture_info .leg_angle_R,4);

	bl_ctrl->torque_info.foot_horizontal_torque_L = 
		bl_ctrl->torque_info.foot_balancing_torque_L + bl_ctrl->torque_info.foot_moving_torque_L;
	bl_ctrl->torque_info.foot_horizontal_torque_R = 
		bl_ctrl->torque_info.foot_balancing_torque_R + bl_ctrl->torque_info.foot_moving_torque_R;//足端轮子水平力矩

	bl_ctrl->foot_motor_L.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_L;
	bl_ctrl->foot_motor_R.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_R;

	LimitMax(bl_ctrl->foot_motor_L.torque_out,16383);
	LimitMax(bl_ctrl->foot_motor_R.torque_out,16383);
	/* -----------------------首先尝试平衡力矩调试-------------------------	 */
	bl_ctrl->torque_info.joint_horizontal_torque_L = 
		 bl_ctrl->torque_info.joint_prevent_splits_torque_L+bl_ctrl->torque_info.joint_moving_torque_L+bl_ctrl->torque_info.joint_balancing_torque_L ;
	bl_ctrl->torque_info.joint_horizontal_torque_R =
		bl_ctrl->torque_info.joint_prevent_splits_torque_R+ bl_ctrl->torque_info.joint_moving_torque_R +bl_ctrl->torque_info.joint_balancing_torque_R ;

	bl_ctrl->torque_info.joint_vertical_torque_L = 
		bl_ctrl->torque_info.joint_stand_torque_L + bl_ctrl->torque_info.joint_roll_torque_L;
	bl_ctrl->torque_info.joint_vertical_torque_R = 
		bl_ctrl->torque_info.joint_stand_torque_R + bl_ctrl->torque_info.joint_roll_torque_R;

	bl_ctrl->torque_info.joint_horizontal_torque_temp1_L = 
		(bl_ctrl->torque_info.joint_horizontal_torque_L) * (-bl_ctrl->mapping_info .J3_L) ;
	bl_ctrl->torque_info.joint_horizontal_torque_temp2_L = 
		(bl_ctrl->torque_info.joint_horizontal_torque_L) * bl_ctrl->mapping_info .J4_L ;
	bl_ctrl->torque_info.joint_horizontal_torque_temp1_R = 
		(bl_ctrl->torque_info.joint_horizontal_torque_R) * bl_ctrl->mapping_info .J3_R ;
	bl_ctrl->torque_info.joint_horizontal_torque_temp2_R = 
		(bl_ctrl->torque_info.joint_horizontal_torque_R) * (-bl_ctrl->mapping_info .J4_R) ;
	
	bl_ctrl->torque_info.joint_vertical_torque_temp1_L = 
		(bl_ctrl->torque_info.joint_vertical_torque_L) * bl_ctrl->mapping_info .J1_L;
	bl_ctrl->torque_info.joint_vertical_torque_temp2_L = 
		(bl_ctrl->torque_info.joint_vertical_torque_L) * (-bl_ctrl->mapping_info .J2_L);
	bl_ctrl->torque_info.joint_vertical_torque_temp1_R = 
		(bl_ctrl->torque_info.joint_vertical_torque_R) * (-bl_ctrl->mapping_info .J1_R);
	bl_ctrl->torque_info.joint_vertical_torque_temp2_R = 
		(bl_ctrl->torque_info.joint_vertical_torque_R) * bl_ctrl->mapping_info .J2_R;

	/****************************************/

	fp32 MAX_balance = 2000.0f;

	LimitMax(bl_ctrl->torque_info.joint_horizontal_torque_temp1_L,MAX_balance);
	LimitMax(bl_ctrl->torque_info.joint_horizontal_torque_temp2_L,MAX_balance);
	LimitMax(bl_ctrl->torque_info.joint_horizontal_torque_temp1_R,MAX_balance);
	LimitMax(bl_ctrl->torque_info.joint_horizontal_torque_temp2_R,MAX_balance);
	LimitMax(bl_ctrl->torque_info.joint_vertical_torque_temp1_L,15);
	LimitMax(bl_ctrl->torque_info.joint_vertical_torque_temp2_L,15);
	LimitMax(bl_ctrl->torque_info.joint_vertical_torque_temp1_R,15);
	LimitMax(bl_ctrl->torque_info.joint_vertical_torque_temp2_R,15);


	bl_ctrl->joint_motor_1.torque_out = + bl_ctrl->torque_info.joint_horizontal_torque_temp1_L + bl_ctrl->torque_info.joint_vertical_torque_temp1_L;
	bl_ctrl->joint_motor_2.torque_out = + bl_ctrl->torque_info.joint_horizontal_torque_temp2_L + bl_ctrl->torque_info.joint_vertical_torque_temp2_L;
	bl_ctrl->joint_motor_3.torque_out = - bl_ctrl->torque_info.joint_horizontal_torque_temp1_R + bl_ctrl->torque_info.joint_vertical_torque_temp1_R;
	bl_ctrl->joint_motor_4.torque_out = - bl_ctrl->torque_info.joint_horizontal_torque_temp2_R + bl_ctrl->torque_info.joint_vertical_torque_temp2_R;
	/* ----------------------------调试阶段可以先不加----------------------------------------- */
	if( fabs(bl_ctrl->joint_motor_1.motor_measure->ecd) >= MOTOR_POS_UPPER_BOUND )
	{ 
		bl_ctrl->joint_motor_1.max_torque = LIMITED_TORQUE;
		bl_ctrl->joint_motor_1.min_torque = -1.0f * UNLIMITED_TORQUE;
	}
	else if( fabs(bl_ctrl->joint_motor_1.motor_measure->ecd) <= -MOTOR_POS_LOWER_BOUND )
	{
		bl_ctrl->joint_motor_1.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_1.min_torque = -1.0f * LIMITED_TORQUE;
	}
	else
	{
		bl_ctrl->joint_motor_1.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_1.min_torque = -1.0f * UNLIMITED_TORQUE;
	}
	
	if( fabs(bl_ctrl->joint_motor_3.motor_measure->ecd) <= -MOTOR_POS_UPPER_BOUND )
	{
		bl_ctrl->joint_motor_3.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_3.min_torque = -1.0f * UNLIMITED_TORQUE;LIMITED_TORQUE;
	}
	else if( fabs(bl_ctrl->joint_motor_3.motor_measure->ecd) >= MOTOR_POS_LOWER_BOUND )
	{
		bl_ctrl->joint_motor_3.max_torque = LIMITED_TORQUE;
		bl_ctrl->joint_motor_3.min_torque = -1.0f *UNLIMITED_TORQUE;
	}
	else
	{
		bl_ctrl->joint_motor_3.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_3.min_torque = -1.0f * UNLIMITED_TORQUE;
	}

	if( fabs(bl_ctrl->joint_motor_2.motor_measure->ecd) <= -MOTOR_POS_UPPER_BOUND )
	{
		bl_ctrl->joint_motor_2.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_2.min_torque = -1.0f * LIMITED_TORQUE;
	}
	else if( fabs(bl_ctrl->joint_motor_2.motor_measure->ecd) >= MOTOR_POS_LOWER_BOUND )
	{
		bl_ctrl->joint_motor_2.max_torque = LIMITED_TORQUE;
		bl_ctrl->joint_motor_2.min_torque = -1.0f * UNLIMITED_TORQUE;
	}
	else
	{
		bl_ctrl->joint_motor_2.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_2.min_torque = -1.0f * UNLIMITED_TORQUE;
	}

	if( fabs(bl_ctrl->joint_motor_4.motor_measure->ecd) >= MOTOR_POS_UPPER_BOUND )
	{
		bl_ctrl->joint_motor_4.max_torque = LIMITED_TORQUE;
		bl_ctrl->joint_motor_4.min_torque = -1.0f * UNLIMITED_TORQUE;
	}
	else if( fabs(bl_ctrl->joint_motor_4.motor_measure->ecd) >= MOTOR_POS_LOWER_BOUND )
	{
		bl_ctrl->joint_motor_4.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_4.min_torque = -1.0f * LIMITED_TORQUE;
	}
	else
	{
		bl_ctrl->joint_motor_4.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_4.min_torque = -1.0f * UNLIMITED_TORQUE;
	}

	LimitOutput( bl_ctrl->joint_motor_1.torque_out, -200.0f,200.0f);//bl_ctrl->joint_motor_1.min_torque, bl_ctrl->joint_motor_1.max_torque);
	LimitOutput( bl_ctrl->joint_motor_2.torque_out, -200.0f,200.0f);//bl_ctrl->joint_motor_2.min_torque, bl_ctrl->joint_motor_2.max_torque);
	LimitOutput( bl_ctrl->joint_motor_3.torque_out, -200.0f,200.0f);//bl_ctrl->joint_motor_3.min_torque, bl_ctrl->joint_motor_3.max_torque);
	LimitOutput( bl_ctrl->joint_motor_4.torque_out, -200.0f,200.0f);//bl_ctrl->joint_motor_4.min_torque, bl_ctrl->joint_motor_4.max_torque);
}


void 
chassis_feedback_update(chassis_move_t *fdb)
{
  if (fdb == NULL)
  {
    return;
  }
  /*------------------------------- Update mode info ----------------------------*/ 
	fdb->mode.last_chassis_mode                = fdb->mode.chassis_mode;
	fdb->mode.last_chassis_balancing_mode      = fdb->mode.chassis_balancing_mode;
	fdb->mode.last_sport_mode                  = fdb->mode.sport_mode;
	fdb->mode.last_jumping_mode                = fdb->mode.jumping_mode;
	fdb->mode.last_jumping_stage               = fdb->mode.jumping_stage;
	fdb->mode.last_chassis_high_mode           = fdb->mode.chassis_high_mode;

	fdb->flag_info.last_static_flag            = fdb->flag_info.static_flag;
	fdb->flag_info.last_moving_flag            = fdb->flag_info.moving_flag;
	fdb->flag_info.last_overpower_warning_flag = fdb->flag_info.overpower_warning_flag;
	fdb->flag_info.last_stablize_high_flag     = fdb->flag_info.stablize_high_flag;

	fdb->flag_info.last_suspend_flag_L         = fdb->flag_info.suspend_flag_L;
	fdb->flag_info.last_suspend_flag_R         = fdb->flag_info.suspend_flag_R;

	fdb->joint_motor_1.last_motor_mode = fdb->joint_motor_1.motor_mode;
	fdb->joint_motor_2.last_motor_mode = fdb->joint_motor_2.motor_mode;
	fdb->joint_motor_3.last_motor_mode = fdb->joint_motor_3.motor_mode;
	fdb->joint_motor_4.last_motor_mode = fdb->joint_motor_4.motor_mode;
	fdb->foot_motor_L.last_motor_mode  = fdb->foot_motor_L.motor_mode;
	fdb->foot_motor_R.last_motor_mode  = fdb->foot_motor_R.motor_mode;


	/*------------------------------- Update HT Motor info ------------------------------ */
	fdb->joint_motor_1.position = (fdb->joint_motor_1.motor_measure->ecd - fdb->joint_motor_1.position_offset) + LEG_OFFSET;
	fdb->joint_motor_2.position = (fdb->joint_motor_2.motor_measure->ecd - fdb->joint_motor_2.position_offset) - LEG_OFFSET;
	fdb->joint_motor_3.position = (fdb->joint_motor_3.motor_measure->ecd - fdb->joint_motor_3.position_offset) - LEG_OFFSET;
	fdb->joint_motor_4.position = (fdb->joint_motor_4.motor_measure->ecd - fdb->joint_motor_4.position_offset) + LEG_OFFSET;

	fdb->joint_motor_1.velocity = fdb->joint_motor_1.motor_measure->speed_rpm;
	fdb->joint_motor_2.velocity = fdb->joint_motor_2.motor_measure->speed_rpm;
	fdb->joint_motor_3.velocity = fdb->joint_motor_3.motor_measure->speed_rpm;
	fdb->joint_motor_4.velocity = fdb->joint_motor_4.motor_measure->speed_rpm;

	// fdb->joint_motor_1.torque_get =  fdb->joint_motor_1.motor_measure->real_torque;
	// fdb->joint_motor_2.torque_get =  fdb->joint_motor_2.motor_measure->real_torque;
	// fdb->joint_motor_3.torque_get =  fdb->joint_motor_3.motor_measure->real_torque;
	// fdb->joint_motor_4.torque_get =  fdb->joint_motor_4.motor_measure->real_torque;

	fdb->joint_motor_1.torque_get = 0.95f * fdb->joint_motor_1.torque_get + 0.05f * fdb->joint_motor_1.motor_measure->real_torque;
	fdb->joint_motor_2.torque_get = 0.95f * fdb->joint_motor_2.torque_get + 0.05f * fdb->joint_motor_2.motor_measure->real_torque;
	fdb->joint_motor_3.torque_get = 0.95f * fdb->joint_motor_3.torque_get + 0.05f * fdb->joint_motor_3.motor_measure->real_torque;
	fdb->joint_motor_4.torque_get = 0.95f * fdb->joint_motor_4.torque_get + 0.05f * fdb->joint_motor_4.motor_measure->real_torque;

  /*------------------------------------ Update LK Motor info -------------------------------*/ 
	fdb->foot_motor_L.last_position = fdb->foot_motor_L.position;
	fdb->foot_motor_R.last_position = fdb->foot_motor_R.position;
	fdb->foot_motor_L.position = fdb->foot_motor_L.motor_measure->angle;
	fdb->foot_motor_R.position = fdb->foot_motor_R.motor_measure->angle;
	if( fdb->flag_info.init_flag != 1 ) 
	{
		if( (fdb->foot_motor_L.last_position - fdb->foot_motor_L.position ) > HALF_POSITION_RANGE )
			fdb->foot_motor_L.turns++;		
		else if( (fdb->foot_motor_L.last_position - fdb->foot_motor_L.position ) < -HALF_POSITION_RANGE )
			fdb->foot_motor_L.turns--;	
		if( ( fdb->foot_motor_R.last_position - fdb->foot_motor_R.position ) > HALF_POSITION_RANGE )
			fdb->foot_motor_R.turns--;
		else if( ( fdb->foot_motor_R.last_position - fdb->foot_motor_R.position ) < -HALF_POSITION_RANGE )
			fdb->foot_motor_R.turns++;
	}
	fdb->foot_motor_L.distance	= ( fdb->foot_motor_L.position/360.0f + fdb->foot_motor_L.turns ) * WHEEL_PERIMETER - fdb->foot_motor_L.distance_offset;
	fdb->foot_motor_R.distance  = ( (360.0f-fdb->foot_motor_R.position/360.0f) + fdb->foot_motor_R.turns ) * WHEEL_PERIMETER - fdb->foot_motor_R.distance_offset;
	fdb->chassis_posture_info.foot_distance = ( fdb->foot_motor_L.distance + fdb->foot_motor_R.distance ) /2.0f;
	
	fdb->foot_motor_L.speed = -fdb->foot_motor_L.motor_measure->speed * PI2 * WHEEL_RADIUS / 10.0f / 60.0f; // rpm -> m/s
	fdb->foot_motor_R.speed = fdb->foot_motor_R.motor_measure->speed * PI2 * WHEEL_RADIUS / 10.0f / 60.0f;
	fdb->chassis_posture_info.foot_speed  = ( fdb->foot_motor_L.speed + fdb->foot_motor_R.speed ) / 2.0f;
	/* ------------------------------------卡尔曼滤波------------------------------------------- */
	/* 目前还不会 */
	fdb->chassis_posture_info.foot_speed_K=fdb->chassis_posture_info.foot_speed;
	fdb->chassis_posture_info.foot_distance_K =fdb->chassis_posture_info.foot_distance;
	/*---------------------------------------- Five_Bars_to_Pendulum -----------------------------*/ 
	Forward_kinematic_solution(fdb,fdb->joint_motor_1.position,fdb->joint_motor_1.velocity,
		fdb->joint_motor_2.position,fdb->joint_motor_2.velocity,1);
	Forward_kinematic_solution(fdb,fdb->joint_motor_4.position,fdb->joint_motor_4.velocity,
		fdb->joint_motor_3.position, fdb->joint_motor_3.velocity,0);

	/* ----------------------------------------- INS angle ----------------------------------------- */
	fdb->chassis_posture_info.yaw_gyro = rad_format(*(fdb->chassis_INS_gyro + INS_GYRO_YAW_ADDRESS_OFFSET));     // - chassis_move_update->chassis_yaw_motor->relative_angle);
	fdb->chassis_posture_info.pitch_gyro = rad_format(*(fdb->chassis_INS_gyro + INS_GYRO_PITCH_ADDRESS_OFFSET)); //- chassis_move_update->chassis_pitch_motor->relative_angle);
	fdb->chassis_posture_info.roll_gyro = *(fdb->chassis_INS_gyro + INS_GYRO_ROLL_ADDRESS_OFFSET);

	fdb->chassis_posture_info.yaw_angle = rad_format(*(fdb->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));     // - chassis_move_update->chassis_yaw_motor->relative_angle);
	fdb->chassis_posture_info.pitch_angle = rad_format(*(fdb->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET)); //- chassis_move_update->chassis_pitch_motor->relative_angle);
	fdb->chassis_posture_info.roll_angle = *(fdb->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
	fdb->chassis_relative_angle = fdb->gimbal_lkmotor_measure->angle;//(rad_format((chassis_move_update->chassis_data_->yaw_angle) * 3.1415926535 / 180) - rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET))) * 180 / 3.1415926535; // angle_change((chassis_move_update->gimbal_yaw_motor.gimbal_motor_measure->ecd-2)*360/8192);

  	/*------------------------------------- foot/leg angle/lenght information update -------------------------------------------- */
	fdb->chassis_posture_info.leg_angle_L -= fdb->chassis_posture_info.pitch_angle;
	// fdb->chassis_posture_info.leg_gyro_L -= fdb->chassis_posture_info.pitch_gyro;

	// fp32 temp_v_L = ( fdb->chassis_posture_info.leg_angle_L - fdb->chassis_posture_info.last_leg_angle_L ) / CHASSIS_CONTROL_TIME;
	// fdb->chassis_posture_info.leg_gyro_L = alpha_dx * temp_v_L + (1-alpha_dx) * fdb->chassis_posture_info.leg_gyro_L;
	fdb->chassis_posture_info .last_leg_angle_L = fdb->chassis_posture_info .leg_angle_L;

	fdb->chassis_posture_info.leg_angle_R -= fdb->chassis_posture_info.pitch_angle;
	// fdb->chassis_posture_info.leg_gyro_R  -= fdb->chassis_posture_info.pitch_gyro;

	// fp32 temp_v_R = ( fdb->chassis_posture_info.leg_angle_R - fdb->chassis_posture_info.last_leg_angle_R ) / CHASSIS_CONTROL_TIME;
	// fdb->chassis_posture_info.leg_gyro_R = alpha_dx * temp_v_R + (1-alpha_dx) * fdb->chassis_posture_info.leg_gyro_R;
	fdb->chassis_posture_info.last_leg_angle_R = fdb->chassis_posture_info .leg_angle_R;

	fp32 temp_v_L = ( fdb->chassis_posture_info.leg_length_L - fdb->chassis_posture_info.last_leg_length_L ) / CHASSIS_CONTROL_TIME;
	fdb->chassis_posture_info.leg_dlength_L = alpha_dx * temp_v_L + (1-alpha_dx) * fdb->chassis_posture_info.leg_dlength_L;
	fdb->chassis_posture_info.last_leg_length_L = fdb->chassis_posture_info.leg_length_L;

	fp32 temp_v_R = ( fdb->chassis_posture_info.leg_length_R - fdb->chassis_posture_info.last_leg_length_R ) / CHASSIS_CONTROL_TIME;
	fdb->chassis_posture_info.leg_dlength_R = alpha_dx * temp_v_R + (1-alpha_dx) * fdb->chassis_posture_info.leg_dlength_R;
	fdb->chassis_posture_info.last_leg_length_R = fdb->chassis_posture_info.leg_length_R;

	/*---------------------------- Rotate and Move Info update ------------------------------*/
	rotate_move_offset = offset_k * rotate_speed_list[robot_level];
	rc_sign = 1.0f;
	X_speed = fdb->chassis_data_->vx_set;
	Y_speed = fdb->chassis_data_->vy_set;
	// Original rc angle
	if (X_speed == 0){
		if (Y_speed > 0)
			rc_angle_temp = PI/2;
		else if (Y_speed < 0)
			rc_angle_temp = -PI/2;
		else
			rc_angle_temp = 0;
	}
	else{
		Y_speed = fdb->chassis_data_->vy_set;
		rc_angle_temp = atan(Y_speed/X_speed);
	}
	// Normalized speed
	if (fabs(X_speed) < 0.1 ||fabs(Y_speed) <0.1)
		temp_max_spd = 1.0f;
	else if (X_speed<Y_speed){
		temp_max_spd = sqrt(square((1/Y_speed)*X_speed)+1);
	}
	else{
		temp_max_spd = sqrt(square((1/X_speed)*Y_speed)+1);
	}
	normalized_speed = sqrt(X_speed*X_speed+Y_speed*Y_speed) / temp_max_spd;
	// Ref angle and sign
	if (X_speed<0) rc_sign *= -1.0f;
	rc_angle_temp += rotate_move_offset;
	if (rc_angle_temp>PI/2||rc_angle_temp<-PI/2){
		rc_sign *= -1.0f;
	}
	while (rc_angle_temp>PI/2) {
		rc_angle_temp -= PI;
	}
	while (rc_angle_temp<-PI/2){
		rc_angle_temp += PI;
	} 
	// if (X_speed < 0) rc_sign = -1.0f; 	
	rc_angle = rc_angle_temp * 180.0f / PI;
	// if (fdb->chassis_rc_ctrl->fake_flag == 0) fake_sign = -1.0f;
	// else fake_sign = 1.0f;


}

void Chassis_Status_Detect( chassis_move_t *detect )
{
	/*--------------------------- Off Ground Detect --------------------------*/


	if( detect->mode.chassis_balancing_mode == BALANCING_READY && detect->mode.sport_mode!=TK_MODE)
	{
		if( fabs(detect->chassis_posture_info.pitch_angle) >= DANGER_PITCH_ANGLE )
			detect->flag_info.abnormal_flag = 1;
		else if( fabs(detect->chassis_posture_info.foot_speed_K) < MOVE_LOWER_BOUND && 
			fabs(detect->chassis_posture_info.pitch_angle)        < 0.1f &&
				detect->flag_info.abnormal_flag )
				detect->flag_info.abnormal_flag = 0;
	}
	// if (abnormal_debug){
	// 	detect->flag_info.abnormal_flag = 0;
	// }

//	Supportive_Force_Cal(detect,  detect->joint_motor_1.position, detect->joint_motor_2.position, 1.0f );	
//	Supportive_Force_Cal(detect,  detect->joint_motor_3.position,  detect->joint_motor_4.position, 0.0f );
	
	if( detect->mode.jumping_stage == CONSTACTING_LEGS )
		detect->flag_info.suspend_flag_L = detect->flag_info.suspend_flag_R = ON_GROUND;
	else
	{
		if( detect->mode.sport_mode == JUMPING_MODE )
		{
			if( detect->chassis_posture_info.supportive_force_R <= LOWER_SUPPORT_FORCE_FOR_JUMP &&
				detect->chassis_posture_info.leg_length_R > 0.13f )
				detect->flag_info.suspend_flag_R = OFF_GROUND;
			else
				detect->flag_info.suspend_flag_L = ON_GROUND;
			if( detect->chassis_posture_info.supportive_force_L <= LOWER_SUPPORT_FORCE_FOR_JUMP &&
				detect->chassis_posture_info.leg_length_L > 0.13f )
				detect->flag_info.suspend_flag_L = OFF_GROUND;
			else 
				detect->flag_info.suspend_flag_R = ON_GROUND;
		}
		// else
		// {
		// 	if( detect->chassis_posture_info.supportive_force_R <= LOWER_SUPPORT_FORCE &&
		// 		detect->chassis_posture_info.leg_length_R > 0.13f )
		// 		detect->flag_info.suspend_flag_R = OFF_GROUND;
		// 	else
		// 		detect->flag_info.suspend_flag_L = ON_GROUND;
		// 	if( detect->chassis_posture_info.supportive_force_L <= LOWER_SUPPORT_FORCE &&
		// 		detect->chassis_posture_info.leg_length_L > 0.13f )
		// 		detect->flag_info.suspend_flag_L = OFF_GROUND;
		// 	else 
		// 		detect->flag_info.suspend_flag_R = ON_GROUND;
		// }
	} 


	if( detect->flag_info.abnormal_flag == 1 &&
		( detect->flag_info.last_suspend_flag_L == ON_GROUND || detect->flag_info.last_suspend_flag_R == ON_GROUND ) &&
		( detect->flag_info.suspend_flag_L == OFF_GROUND || detect->flag_info.suspend_flag_R  == OFF_GROUND ) )
			detect->flag_info.Ignore_Off_Ground = 1;
	else if( detect->flag_info.abnormal_flag != 1 )
		detect->flag_info.Ignore_Off_Ground = 0;
	if( detect->flag_info.Ignore_Off_Ground )
	{
		detect->flag_info.suspend_flag_R = ON_GROUND;
		detect->flag_info.suspend_flag_L = ON_GROUND;
	}
	// detect->flag_info.suspend_flag_L = detect->flag_info.suspend_flag_R = ON_GROUND;
	//Moving_High_Offset

	if( fabs(detect->chassis_posture_info.foot_speed_K) > stablize_foot_speed_threshold &&
		fabs(detect->chassis_posture_info.yaw_gyro ) > stablize_yaw_speed_threshold )
		detect->flag_info.stablize_high_flag = 1;

}

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel, vy_channel;
  fp32 vx_set_channel, vy_set_channel;
  // deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
  // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

  // keyboard set speed set-point
  // 键盘控制
  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
  }
  else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
  }

  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
  }
  else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
  }

  // first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
  // 一阶低通滤波代替斜波作为底盘速度输入
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
  // stop command, need not slow change, set zero derectly
  // 停止信号，不需要缓慢加速，直接减速到零
  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
  *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

/**
 * @brief          four mecanum wheels speed is calculated by three param.
 * @param[in]      vx_set: vertial speed
 * @param[in]      vy_set: horizontal speed
 * @param[in]      wz_set: rotation speed
 * @param[out]     wheel_speed: four mecanum wheels speed
 * @retval         none
 */
/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
  // because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
  // 旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
  wheel_speed[0] = (vx_set + vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set) * 1.41421356f;
  wheel_speed[1] = (-vx_set + vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set) * 1.41421356f;
  wheel_speed[2] = (-vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set) * 1.41421356f;
  wheel_speed[3] = (vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set) * 1.41421356f;
}


void Motor_CMD_Send(chassis_move_t *CMD_Send)
{
	if( CMD_Send->joint_motor_1.motor_mode == MOTOR_FORCE )
		CAN_HT_CMD( 0x01, CMD_Send->joint_motor_1.torque_out );
	else 
		CAN_HT_CMD( 0x01, 0.0 );
	if( CMD_Send->joint_motor_2.motor_mode == MOTOR_FORCE )
		CAN_HT_CMD( 0x02, CMD_Send->joint_motor_2.torque_out );
	else 
		CAN_HT_CMD( 0x02, 0.0 );


	if( CMD_Send->foot_motor_R.motor_mode != MOTOR_FORCE )
		CMD_Send->foot_motor_R .torque_out = 0.0f;
	if( CMD_Send->foot_motor_L.motor_mode != MOTOR_FORCE )
		CMD_Send->foot_motor_L .torque_out = 0.0f;
	vTaskDelay(10);

	CAN_LK_Torque_Control( 0x141,-CMD_Send->foot_motor_L.torque_out );
	vTaskDelay(10);
	CAN_LK_Torque_Control( 0x142,-CMD_Send->foot_motor_R.torque_out );
	vTaskDelay(10);
	if( CMD_Send->joint_motor_3.motor_mode == MOTOR_FORCE )
		CAN_HT_CMD( 0x03, CMD_Send->joint_motor_3.torque_out );
	else 
		CAN_HT_CMD( 0x03, 0.0 );
	

	if( CMD_Send->joint_motor_4.motor_mode == MOTOR_FORCE )
		CAN_HT_CMD( 0x04, CMD_Send->joint_motor_4.torque_out );
	else 
		CAN_HT_CMD( 0x04, 0.0 );
}

void Joint_Motor_to_Init_Pos()
{
	static int Init_Time = 0;
	while( Init_Time < 200 )
	{
		CAN_HT_CMD( 0x01, 0.8 );
		vTaskDelay(2);
		CAN_HT_CMD( 0x02, -0.8 );
		vTaskDelay(2);
		CAN_HT_CMD( 0x03, -0.8 );
		vTaskDelay(2);
		CAN_HT_CMD( 0x04, 0.8 );
		vTaskDelay(2);
		Init_Time++;
	}
}
void HT_Motor_zero_set(void)
{
	uint8_t tx_buff[8];
	for( int i = 0; i < 7; i++ )
		tx_buff[i] = 0xFF;
	tx_buff[7] = 0xfc;

	CAN_CMD_HT_Enable( 0x01, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x02, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x03, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x04, tx_buff );
	vTaskDelay(50);

	Joint_Motor_to_Init_Pos();
	// Set zero init point
	tx_buff[7] = 0xfe;

	CAN_CMD_HT_Enable( 0x01, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x02, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x03, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x04, tx_buff );

	vTaskDelay(50);

}

void Motor_Zero_CMD_Send(void)
{
	CAN_HT_CMD( 0x01, 0.0 );
	vTaskDelay(1);
	CAN_HT_CMD( 0x02, 0.0 );
	vTaskDelay(1);
	CAN_HT_CMD( 0x03, 0.0 );
	vTaskDelay(1);
	CAN_HT_CMD( 0x04, 0.0 );
	vTaskDelay(1);

}

void Forward_kinematic_solution(chassis_move_t *feedback_update,
											fp32 Q1,fp32 S1,fp32 Q4,fp32 S4, uint8_t ce)
{
  fp32 dL0=0,L0=0,Q0=0,S0=0;
  fp32 xb,xd,yb,yd,Lbd;
  fp32 A0,B0,C0,Q2,Q3,S2,S3;
  fp32 vxb,vxd,vyb,vyd,vxc,vyc;
  fp32 sin_Q2,cos_Q2,sin_Q3,cos_Q3;
  fp32 axb,ayb,axd,ayd,a2,axc;
  fp32 cos_Q1,cos_Q4,sin_Q1,sin_Q4;
  fp32 xc,yc;	
  /******************************/
	Q1 = ((180.0f+Q1)*PI)/180.0f;
	Q4 = ((180.0f-Q4)*PI)/180.0f;

	cos_Q1 = cos(Q1);
	sin_Q1 = sin(Q1);
	cos_Q4 = cos(Q4);
	sin_Q4 = sin(Q4);
	xb = -L5/2.0f + L1*cos_Q1;
	xd =  L5/2.0f - L4*cos_Q4;
	yb = L1*sin_Q1;
	yd = L4*sin_Q4;

	Lbd=(xd-xb)*(xd-xb)+(yd-yb)*(yd-yb);
	A0 = 2.0f*L2*(xd-xb);
	B0 = 2.0f*L2*(yd-yb);
	C0 = L2*L2+Lbd-L3*L3;
	Q2 = 2.0f *atan((B0+sqrt(A0*A0 + B0*B0 -C0*C0))/(A0+C0));
	
	xc = xb + cos(Q2)*L2;
	yc = yb + sin(Q2)*L2;

	L0=sqrt( xc*xc + yc*yc );
	Q0 = atan(xc/yc);
	
	vxb = -S1*L1*sin_Q1;
	vyb = S1*L1*cos_Q1;
	vxd = -S4*L4*sin_Q4;
	vyd = S4*L4*cos_Q4;
	Q3 = atan((yc-yd)/(xc-xd));
	S2 = ((vxd-vxb)*cos(Q3) + (vyd-vyb)*sin(Q3))/(L2*sin(Q3-Q2)); 
	S3 = ((vxd-vxb)*cos(Q2) + (vyd-vyb)*sin(Q2))/(L2*sin(Q3-Q2)); 
	vxc = vxb - S2*L2*sin(Q2);
  	vyc = vyb + S2*L2*cos(Q2);
	S0 = 3*(-sin(fabs(Q0))*vxc-cos(Q0)*vyc);

	// if( Q0 < 0 )
	// 	Q0 = -Q0;
	/*******************************/
	if (ce)
	{
		feedback_update->chassis_posture_info .leg_length_L = L0;
		feedback_update->chassis_posture_info .leg_angle_L  = Q0;
		feedback_update->chassis_posture_info .xc  = xc;
    feedback_update->chassis_posture_info .yc  = yc;
		feedback_update->chassis_posture_info .xb  = xb;
    feedback_update->chassis_posture_info .yb  = yb;
    feedback_update->chassis_posture_info .Q2  = Q2;

		feedback_update->chassis_posture_info .leg_gyro_L   		= S0;
	}
	else 
	{
		feedback_update->chassis_posture_info .leg_length_R = L0;
		feedback_update->chassis_posture_info .leg_angle_R  = -Q0;
		feedback_update->chassis_posture_info .leg_gyro_R   		= -S0;
	}
}

// 计算多项式值
float evaluate_polynomial(float L0, float Q0, PolynomialCoefficients coeffs) {
    return coeffs.c0 + 
           coeffs.c1 * L0 + 
           coeffs.c2 * Q0 + 
           coeffs.c3 * L0 * L0 + 
           coeffs.c4 * L0 * Q0 + 
           coeffs.c5 * Q0 * Q0;
}

// 计算雅可比矩阵
float get_jacobian_element(chassis_move_t *VMCJ, float L0, float Q0, uint8_t element_type) {
    switch(element_type) {
        case 1: // N11
            return evaluate_polynomial(L0, Q0, VMCJ->InverseJacobianCoefficient.N11);
        case 2: // N12
            return evaluate_polynomial(L0, Q0, VMCJ->InverseJacobianCoefficient.N12);
        case 3: // N21
            return evaluate_polynomial(L0, Q0, VMCJ->InverseJacobianCoefficient.N21);
        case 4: // N22
            return evaluate_polynomial(L0, Q0, VMCJ->InverseJacobianCoefficient.N22);
        default:
            return 0.0f; // 或者返回错误值
    }
}