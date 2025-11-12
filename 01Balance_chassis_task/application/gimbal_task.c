#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "pid.h"
#include "gimbal_task.h"
#include "shoot.h"
#include "arm_math.h"
#include "user_lib.h"
#include "math.h"
#include "chassis_task.h"
#include "uart_receive.h"

gimbal_control_t gimbal_control;
extern chassis_move_t chassis_move;
uint8_t start;
uint8_t close;
float yaw_angle_limit_func(float input) 
{
    if (input >= 180) {
        return input - 360;
    } else if (input <= -180) {
        return input + 360;
    } else {
        return input;
    }
}	
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}
void gimbal_Init(gimbal_control_t *gimbal_control)
{

	gimbal_control->yaw_ctrl_data=get_Uart_Chassisdata_point();
	gimbal_control->gimbal_yaw_motor.gimbal_lkmotor_measure=get_yaw_gimbal_lkmotor_measure_point();
	gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->gimbal_yaw_motor.absolute_angle;
	gimbal_control->gimbal_yaw_motor.absolute_angle_set1=rad_format(gimbal_control->gimbal_yaw_motor.absolute_angle_set);
	gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_OFF;
	gimbal_PID_init(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid,YAW_ANGLE_PID_MAX_OUT,YAW_ANGLE_PID_MAX_IOUT,YAW_ANGLE_PID_KP,YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD);
}
void gimbal_set_mode(gimbal_control_t *gimbal_control)
{
	if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_INIT)
	{
		if(fabs(gimbal_control->gimbal_yaw_motor.relative_angle-gimbal_control->gimbal_yaw_motor.relative_angle_set)>1)
		{
			return;
		}
		else
		{
			gimbal_control->YAW_INIT_FLAG=1;
		}
	}
	if(gimbal_control->yaw_ctrl_data->chassis_mode==CHASSIS_MODE_OFF)
	{
		gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_OFF;
	}
	else if(gimbal_control->yaw_ctrl_data->chassis_mode==CHASSIS_MODE_FOLLOW||gimbal_control->yaw_ctrl_data->chassis_mode==CHASSIS_MODE_NO_FOLLOW)
	{
		gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	if(gimbal_control->yaw_ctrl_data->chassis_mode==CHASSIS_MODE_INIT)
	{
		gimbal_control->gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_INIT;
	}
}
void gimbal_feedback_update(gimbal_control_t *gimbal_control)
{
//	CAN_read_lkmotor_state();
	gimbal_control->gimbal_yaw_motor.absolute_angle=gimbal_control->yaw_ctrl_data->yaw_angle;
	gimbal_control->gimbal_yaw_motor.motor_gyro=gimbal_control->yaw_ctrl_data->yaw_gyro;
	gimbal_control->gimbal_yaw_motor.relative_angle= gimbal_control->gimbal_yaw_motor.gimbal_lkmotor_measure->angle; //(rad_format((gimbal_control->yaw_ctrl_data->yaw_angle)*3.1415926535/180)-rad_format(chassis_move->chassis_yaw))*180/3.1415926535;

	
}
void gimbal_set_control(gimbal_control_t *gimbal_control)
{
	// const static fp32 yaw_angle_set_order_filter[1] = {YAW_SET_NUM};
	if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_OFF)
	{
		gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->gimbal_yaw_motor.absolute_angle;
	}
	else  if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		gimbal_control->gimbal_yaw_motor.absolute_angle_set=gimbal_control->yaw_ctrl_data->yaw_angle_set;
	}
	else  if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_INIT)
	{
		gimbal_control->gimbal_yaw_motor.relative_angle_set=gimbal_control->gimbal_yaw_motor.relative_angle;//0.0f;
	}	
}
void gimbal_control_loop(gimbal_control_t *gimbal_control)
{
	if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_OFF)
	{
		if(close==0)
		{
			// CAN_LK_CLOSE_control();
			close=1;
		}
		else if(close==1)
		{
			start = 0;
		}
	}
	else if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		gimbal_PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid,gimbal_control->gimbal_yaw_motor.absolute_angle,gimbal_control->gimbal_yaw_motor.absolute_angle_set,gimbal_control->gimbal_yaw_motor.motor_gyro);
		gimbal_control->gimbal_yaw_motor.yaw_given_current= gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid.out;
	}
	else if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_ROTATE)
	{
		gimbal_PID_calc(&gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid,gimbal_control->gimbal_yaw_motor.absolute_angle,gimbal_control->gimbal_yaw_motor.absolute_angle_set,gimbal_control->gimbal_yaw_motor.motor_gyro);
		gimbal_control->gimbal_yaw_motor.yaw_given_current=gimbal_control->gimbal_yaw_motor.gimbal_motor_angle1_pid.out;
	}
	else if(gimbal_control->gimbal_yaw_motor.gimbal_motor_mode==GIMBAL_MOTOR_INIT)
	{
		if(start==0)
		{
//			CAN_LK_START_control();
			close=0;
			start=1;
		}
		if(start==1)
		{
			//CAN_LK_POSITION_Control(0);
		}
	}
}
void gimbal_task()
{
	gimbal_Init(&gimbal_control);
	shoot_init();
	while(1)
	{
		gimbal_set_mode(&gimbal_control);
		gimbal_feedback_update(&gimbal_control);
		gimbal_set_control(&gimbal_control);
		gimbal_control_loop(&gimbal_control);
		gimbal_control.shoot=shoot();
		if(gimbal_control.gimbal_yaw_motor.gimbal_motor_mode!=GIMBAL_MOTOR_OFF)
		{
//			CAN_cmd_gimbal(0,0,gimbal_control.shoot->given_current,0);
		}
		if(gimbal_control.gimbal_yaw_motor.gimbal_motor_mode!=GIMBAL_MOTOR_INIT)
		{
			// CAN_LK_SPEED_Control(1000,gimbal_control.gimbal_yaw_motor.gimbal_motor_angle1_pid.out);
		}
		vTaskDelay(GIMBAL_CONTROL_TIME);
	}
}
