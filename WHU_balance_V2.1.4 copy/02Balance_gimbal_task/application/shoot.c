#include "main.h"
#include "shoot.h"
#include "bsp_laser.h"
#include <math.h>

//射击数据结构体
shoot_control_t shoot_control;

//时间计数
int32_t time_cnt;

//连发标志位
int8_t continue_flag;

//射击摩擦轮PID参数
static const fp32 friction_speed_pid[3]={FRICTION_SPEED_PID_KP ,FRICTION_SPEED_PID_KI,FRICTION_SPEED_PID_KD};
static const fp32 friction_current_pid[3]={FRICTION_CURRENT_PID_KP ,FRICTION_CURRENT_PID_KI,FRICTION_CURRENT_PID_KD};


/**
 * @brief 更新鼠标数据
 * @param[in] none
 * @param[out] none
 * @note 
 */
void shoot_feedback_update(void);

/**
 * @brief 速度滤波处理
 * @param[in] none
 * @param[out] none
 * @note 
 */
void shoot_speed_filter(void);

/**
 * @brief 设置发单模式
 * @param[in] none
 * @param[out] none
 * @note 
 */
void shoot_set_mode(void);

/**
 * @brief 摩擦轮速度设置
 * @param[in] none
 * @param[out] none
 * @note 
 */
void friction_control_set(void);



void shoot_Init()
{
	//摩擦轮电机PID参数初始化
	PID_init(&shoot_control.friction_left_motor_speed_pid,PID_POSITION,friction_speed_pid,FRICTION_SPEED_PID_MAX_OUT,FRICTION_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.friction_right_motor_speed_pid,PID_POSITION,friction_speed_pid,FRICTION_SPEED_PID_MAX_OUT,FRICTION_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.friction_left_motor_current_pid,PID_POSITION,friction_current_pid,FRICTION_CURRENT_PID_MAX_OUT,FRICTION_CURRENT_PID_MAX_IOUT);
	PID_init(&shoot_control.friction_right_motor_current_pid,PID_POSITION,friction_current_pid,FRICTION_CURRENT_PID_MAX_OUT,FRICTION_CURRENT_PID_MAX_IOUT);
	
	//初始化模式设置
	shoot_control.shoot_mode=SHOOT_STOP;

	//获取遥控器数据 和 摩擦轮电机数据
	shoot_control.shoot_rc = get_remote_control_point();
	shoot_control.friction_motor_measure[0]=get_chassis_motor_measure_point(0);
	shoot_control.friction_motor_measure[1]=get_chassis_motor_measure_point(1);

}

const shoot_control_t *shoot_control_loop()
{
	//获取鼠标数据
	shoot_feedback_update(); 

	//速度滤波处理
	shoot_speed_filter();

	//设置发射模式
	shoot_set_mode();		
	
	//摩擦轮速度设置
	friction_control_set();

	//摩擦轮速度环电流环PID控制
	PID_calc(&shoot_control.friction_left_motor_speed_pid, shoot_control.friction_left_speed, shoot_control.friction_left_speed_set);
	PID_calc(&shoot_control.friction_right_motor_speed_pid, shoot_control.friction_right_speed, shoot_control.friction_right_speed_set);
	shoot_control.shoot_left_given_current = shoot_control.friction_left_motor_speed_pid.out;//PID_calc(&shoot_control.friction_left_motor_current_pid, shoot_control.friction_motor_measure[0]->given_current,shoot_control.friction_left_motor_speed_pid.out);//shoot_control.friction_right_motor_speed_pid.out);
	shoot_control.shoot_right_given_current= shoot_control.friction_right_motor_speed_pid.out;//PID_calc(&shoot_control.friction_right_motor_current_pid, shoot_control.friction_motor_measure[1]->given_current,shoot_control.friction_right_motor_speed_pid.out);

	return &shoot_control;
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

	//根据遥控器设置发射模式
	if(shoot_control.shoot_rc->rc.s[0]==2)
	{
		shoot_control.shoot_mode=SHOOT_STOP;
	}
	else if(shoot_control.shoot_rc->rc.s[0]==3)
	{
		shoot_control.shoot_mode=SHOOT_READY_FRIC;
	}
	if(((shoot_control.press_r&&!shoot_control.last_press_r)||shoot_control.shoot_rc->rc.s[0]==1)&&shoot_control.shoot_mode!=SHOOT_CONTINUE)
	{
		shoot_control.shoot_mode=SHOOT_SINGLE;
	}
	if(shoot_control.shoot_rc->rc.s[0]==1)
	{
		time_cnt++;
		if(time_cnt>400)
		{
			continue_flag=1;
			time_cnt=0;
		}
	}

	if((shoot_control.press_l&&shoot_control.last_press_l)||continue_flag==1)
	{
		continue_flag=0;
		shoot_control.shoot_mode=SHOOT_CONTINUE;
	}
}

void friction_control_set()
{
	if(shoot_control.shoot_rc->rc.s[0]==3||shoot_control.shoot_rc->rc.s[0]==1)
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

	//一阶低通滤波
	shoot_control.friction_left_speed=0.5f*shoot_control.friction_motor_measure[0]->speed_rpm+0.5f*shoot_control.friction_left_last_speed;
	shoot_control.friction_right_speed=0.5f*shoot_control.friction_motor_measure[1]->speed_rpm+0.5f*shoot_control.friction_right_last_speed;

	//设置速度上限
	fp32_constrain(shoot_control.friction_left_speed,-10000,10000);
	fp32_constrain(shoot_control.friction_right_speed,-10000,10000);
}

