#include "main.h"
#include "shoot.h"
#include "bsp_laser.h"
#include <math.h>
float data1[2];
shoot_control_t shoot_control;
int shoot_flag=0;

/* 初始化的低通滤波器 */
void low_pass_filter_init(low_pass_filter_t *filter,fp32 alpha)
{
	filter->alpha=alpha;
	filter->output=0;
}
fp32 low_pass_filter_calc(low_pass_filter_t *filter,fp32 input)
{
	filter->output=filter->alpha*input+(1-filter->alpha)*filter->output;
	return filter->output;
}

void shoot_Init()
{
	static const fp32 friction_speed_pid[3]={FRICTION_SPEED_PID_KP ,FRICTION_SPEED_PID_KI,FRICTION_SPEED_PID_KD};
	PID_init(&shoot_control.friction_left_motor_speed_pid,PID_POSITION,friction_speed_pid,FRICTION_SPEED_PID_MAX_OUT,FRICTION_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.friction_right_motor_speed_pid,PID_POSITION,friction_speed_pid,FRICTION_SPEED_PID_MAX_OUT,FRICTION_SPEED_PID_MAX_IOUT);

	shoot_control.shoot_mode=SHOOT_STOP;
	shoot_control.shoot_rc = get_remote_control_point();
	shoot_control.friction_motor_measure[0]=get_chassis_motor_measure_point(0);
	shoot_control.friction_motor_measure[1]=get_chassis_motor_measure_point(1);
//	low_pass_filter_init(&shoot_control.left_speed_filter,0.7f);
//	low_pass_filter_init(&shoot_control.right_speed_filter,0.7f);

}
void shoot_feedback_update()
{
	shoot_control.last_press_l=shoot_control.press_l;
	shoot_control.last_press_r=shoot_control.press_r;
	shoot_control.press_l=shoot_control.shoot_rc->mouse.press_l;
	shoot_control.press_r=shoot_control.shoot_rc->mouse.press_r;
}
void shoot_set_mode()
{

	if(shoot_control.press_r&&shoot_control.last_press_r==0)
	{
		shoot_flag=!shoot_flag;
	}
	if(shoot_flag==1||shoot_control.shoot_rc->rc.s[0]==1)
	{
		shoot_control.shoot_mode=SHOOT_BULLET;
	}
	else if(shoot_flag==0||shoot_control.shoot_rc->rc.s[0]==2)
	{
		shoot_control.shoot_mode=SHOOT_STOP;
	}

	
}
void friction_control_set()
{
	if(shoot_control.shoot_mode==SHOOT_BULLET)
	{
		shoot_control.friction_left_speed_set=-6000;  
		shoot_control.friction_right_speed_set=6000;                              
		laser_on();
		
	}
	else 
	{
		shoot_control.friction_left_speed_set=0;  
		shoot_control.friction_right_speed_set=0;                              
		laser_off();
	}
}

void shoot_speed_filter()
{
	shoot_control.friction_left_last_speed=shoot_control.friction_left_speed;
	shoot_control.friction_right_last_speed=shoot_control.friction_right_speed;
	shoot_control.friction_left_speed=1.0f*shoot_control.friction_motor_measure[0]->speed_rpm;
	shoot_control.friction_right_speed=1.0f*shoot_control.friction_motor_measure[1]->speed_rpm;

	if(fabs(shoot_control.friction_left_speed)>10000)
	{
		shoot_control.friction_left_speed=shoot_control.friction_left_last_speed;
	}
	if(fabs(shoot_control.friction_right_speed)>10000)
	{
		shoot_control.friction_right_speed=shoot_control.friction_right_last_speed;
	}
//	shoot_control.friction_left_speed=low_pass_filter_calc(&shoot_control.left_speed_filter,shoot_control.friction_left_speed);
//	shoot_control.friction_right_speed=low_pass_filter_calc(&shoot_control.right_speed_filter,shoot_control.friction_right_speed);
}
fp32 apply_deadzone(fp32 value,fp32 deadzone)
{
	if(fabs(value)<deadzone)
	{
		return 0.0f;
	}	
	return value;
}
const shoot_control_t *shoot_control_loop()
{
	shoot_feedback_update(); /* 获取鼠标发射按键数据 */
	shoot_set_mode();		 /* up->bullet    down->stop */
	friction_control_set();
	shoot_speed_filter();
	
	shoot_control.shoot_left_given_current=PID_calc(&shoot_control.friction_left_motor_speed_pid, shoot_control.friction_left_speed, shoot_control.friction_left_speed_set);

	shoot_control.shoot_right_given_current=PID_calc(&shoot_control.friction_right_motor_speed_pid, shoot_control.friction_right_speed, shoot_control.friction_right_speed_set);
//	shoot_control.shoot_right_given_current = PID_calc(&shoot_control.friction_right_motor_current_pid, shoot_control.friction_motor_measure[1]->given_current,100);//shoot_control.friction_right_motor_speed_pid.out);
//	shoot_control.shoot_right_given_current=apply_deadzone(shoot_control.shoot_right_given_current,10.0f);
	return &shoot_control;
}