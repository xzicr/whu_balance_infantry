#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "shoot.h"
#include "pid.h"
#include "referee.h"
#include "detect_task.h"
#include "math.h"

shoot_control_t shoot_control;
uint16_t shootflag=0;
uint16_t anti_jamming_flag=0;
//float data[2];

void tigger_motor_turn_back(void)
{
	if(shoot_control.block_time<BLOCK_TIME)
		shoot_control.speed_set=5000;
	else
	{
		shoot_control.speed_set=-5000;
	}
	if(shoot_control.shoot_motor_measure->speed_rpm<BLOCK_TRIGGER_SPEED&&shoot_control.block_time<BLOCK_TIME)
	{
		shoot_control.block_time++;
		shoot_control.reverse_time=0;
	}
	else if(shoot_control.block_time==BLOCK_TIME&&shoot_control.reverse_time<REVERSE_TIME)
	{
		shoot_control.reverse_time++;
	}
	else
	{
		shoot_control.block_time=0;
	}
}
static void shoot_feedback_update(void)
{
	shoot_control.angle=shoot_control.shoot_motor_measure->angle;
	shoot_control.last_press_l=shoot_control.press_l;
	shoot_control.press_l=(shoot_control.shoot_control_data->shoot_mode>>1) & 0x01;
	shoot_control.shoot_friction_mode=shoot_control.shoot_control_data->shoot_mode&0x04;
	get_power_shooter_output(&shoot_control.shooter_output);
}
void shoot_init()
{
	static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
	static const fp32 Trigger_angle_pid[3]={TRIGGER_ANGLE_PID_KP ,TRIGGER_ANGLE_PID_KI ,TRIGGER_ANGLE_PID_KD };
	shoot_control.shoot_motor_measure=get_trigger_motor_measure_point();
	shoot_control.shoot_control_data=get_Chassisdata_point();
	PID_init(&shoot_control.trigger_motor_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT, TRIGGER_BULLET_PID_MAX_IOUT);
	PID_init(&shoot_control.trigger_motor_angle_pid,PID_POSITION,Trigger_angle_pid,TRIGGER_ANGLE_PID_MAX_OUT ,TRIGGER_ANGLE_PID_MAX_IOUT);
	shoot_control.angle =shoot_control.shoot_motor_measure->angle;
	shoot_control.set_angle = shoot_control.angle;
	shoot_control.speed = 0.0f;
    shoot_control.speed_set=0.0f ;
	
}

static void shoot_set_mode(void)
{
	if(shoot_control.shoot_control_data->shoot_mode==4)
	{
		shoot_control.shoot_mode=SHOOT_BULLET;
		
	}
//	else if(shoot_control.shoot_control_data->shoot_mode==3)
//	{
//		shoot_control.shoot_mode=SHOOT_STOP;
//		
//	}
}
void shoot_control_set()
{

//	if(fabs(shoot_control.set_angle-shoot_control.angle)>1000)
//	{
//		anti_jamming_flag=1;
//	}
//	else
//	{
//		anti_jamming_flag=0;
//	}
	if(shoot_control.shoot_friction_mode==4&&!toe_is_error(TRIGGER_MOTOR_TOE))//&&anti_jamming_flag==0)
	{
//		shoot_control.set_angle+=1140;
		tigger_motor_turn_back();
	}
	else
	{
		shoot_control.speed_set=0;
	}
	//当射击标志位和拨弹电机都在线时

}
void shoot_control_loop()
{
//	PID_calc(&shoot_control.trigger_motor_angle_pid,shoot_control.angle,shoot_control.set_angle);
	PID_calc(&shoot_control.trigger_motor_speed_pid, shoot_control.shoot_motor_measure->speed_rpm, shoot_control.speed_set);
	shoot_control.given_current=(int16_t)(shoot_control.trigger_motor_speed_pid.out);
}
shoot_control_t *shoot()
{
		
		shoot_feedback_update();
		shoot_set_mode();
		shoot_control_set();
		shoot_control_loop();
		return &shoot_control;
	
}