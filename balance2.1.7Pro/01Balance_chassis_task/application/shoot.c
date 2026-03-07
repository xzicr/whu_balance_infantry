#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "shoot.h"
#include "pid.h"
#include "referee.h"
#include "detect_task.h"
#include "math.h"
#include "user_lib.h"
#include "uart_receive.h"
shoot_control_t shoot_control;
const chassis_data_t *shoot_enable;

uint8_t single_shoot_state = 0;			//单发模式状态位，用于显示单发动作的执行情况
uint8_t single_shoot_cnt = 0;			//??1表示本次单发动作完成，防止连续单??

uint16_t shoot_state;
uint8_t process_state = 0;
int8_t shoot_mode;

void ballet_feq_calc(uint8_t fequence)
{
	shoot_control.speed_set = (fequence/8)*60*36;
}

/*重写的播弹盘电机函数*/
void push_motor_on(void)
{
	shoot_control.speed_set=7000;
}

void trigger_motor_turn_back(void)
{
	/* ---------1 rpm <-> 0.10471975511 rad/s------*/
	/* ----------2006减速比36??1------------------- */
	//7000rpm  对应输出20.36217460rad/s
	if(shoot_control.block_time<BLOCK_TIME)
		shoot_control.speed_set=7000;
	else
	{
		shoot_control.speed_set=-7000;
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
uint16_t shoot_single_control(void)
{
    if (single_shoot_state == 0)
    {
        shoot_control.set_angle = shoot_control.angle + PI_TEN;
        single_shoot_state = 1;
        return 0;  // 动作开??
    }
    else if(single_shoot_state == 1)
    {
        if (fabs(shoot_control.set_angle - shoot_control.angle) > 2.0f)
        {
			/* ------位置环PID-------- */
            PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.angle, shoot_control.set_angle);
            PID_calc(&shoot_control.trigger_position_mode_speed_pid, shoot_control.shoot_motor_measure->speed_rpm, shoot_control.trigger_motor_angle_pid.out);
            shoot_control.given_current=(int16_t)(shoot_control.trigger_position_mode_speed_pid.out);
            return 0;  // 动作进行??
        }
        else 
        {
            single_shoot_state = 0;
            single_shoot_cnt = 1;
            return 1;  // 动作完成
        }
    }
    return 0;  // 默认返回??
}

static void shoot_feedback_update(void)
{
	shoot_control.speed = shoot_control.shoot_motor_measure->speed_rpm ;
	shoot_control.angle=shoot_control.shoot_motor_measure->angle;
	shoot_control.last_press_l=shoot_control.press_l;
	shoot_control.press_l=(shoot_control.shoot_control_data->shoot_mode_rc >>4) & 0x01;
	shoot_mode = shoot_control.shoot_control_data->shoot_mode_rc&0x0F;
	// get_power_shooter_output(&shoot_control.shooter_output);
}
void shoot_init()
{
	static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
	static const fp32 Trigger_angle_pid[3]={TRIGGER_ANGLE_PID_KP ,TRIGGER_ANGLE_PID_KI ,TRIGGER_ANGLE_PID_KD };

	shoot_control.shoot_motor_measure=get_trigger_motor_measure_point();
	shoot_control.shoot_control_data=get_Uart_Chassisdata_point();

	PID_init(&shoot_control.trigger_position_mode_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT, TRIGGER_BULLET_PID_MAX_IOUT);
	PID_init(&shoot_control.trigger_speed_mode_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT, TRIGGER_BULLET_PID_MAX_IOUT);
	PID_init(&shoot_control.trigger_motor_angle_pid,PID_POSITION,Trigger_angle_pid,TRIGGER_ANGLE_PID_MAX_OUT ,TRIGGER_ANGLE_PID_MAX_IOUT);
	
	shoot_control.angle =shoot_control.shoot_motor_measure->angle;
	shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed_set=0.0f;
	shoot_control.speed = 0.0f;
	shoot_state = SHOOT_FINISH;
	
}

static void shoot_set_mode(void)
{
	if(shoot_mode==SHOOT_SINGLE)
	{
		shoot_control.shoot_mode=SHOOT_SINGLE;
	}
	else if(shoot_mode==SHOOT_CONTINUE)
	{
		shoot_control.shoot_mode=SHOOT_CONTINUE;
	}
	else if(shoot_mode==SHOOT_STOP||shoot_mode==SHOOT_READY_FRIC)
	{
		shoot_control.shoot_mode=SHOOT_STOP;
	}

}
void shoot_control_set()
{
	if(shoot_control.shoot_mode==SHOOT_SINGLE  &&shoot_state == SHOOT_FINISH&& single_shoot_cnt == 0)
	{
		shoot_state = SHOOT_START_SINGLE;
	}
	else if(shoot_state == SHOOT_START_SINGLE)
	{
		process_state = shoot_single_control();
		if (process_state==1)
		{
			process_state=0;
			shoot_control.given_current=0;
			shoot_state = SHOOT_FINISH;
		}
	}

	if(shoot_control.shoot_mode==SHOOT_CONTINUE && shoot_state == SHOOT_FINISH)
	{
		shoot_state = SHOOT_START_CONTINUE;
	}
	else if(shoot_state == SHOOT_START_CONTINUE)
	{
		ballet_feq_calc(11);
		single_shoot_cnt = 0;	//单发的这个标志位置0

		/* -------计算PID速度??-------- */
		PID_calc(&shoot_control.trigger_speed_mode_speed_pid, shoot_control.shoot_motor_measure->speed_rpm, shoot_control.speed_set);
		shoot_control.given_current=(int16_t)(shoot_control.trigger_speed_mode_speed_pid.out);

		if(shoot_control.shoot_mode!=SHOOT_CONTINUE)
		{
			shoot_state = SHOOT_FINISH;
		}
	}

	if((shoot_control.shoot_mode!=SHOOT_CONTINUE)&&(shoot_control.shoot_mode!=SHOOT_SINGLE)&&shoot_state == SHOOT_FINISH)
	{	//停拨弹盘
		shoot_control.speed_set=0;
		PID_calc(&shoot_control.trigger_speed_mode_speed_pid, shoot_control.shoot_motor_measure->speed_rpm, shoot_control.speed_set);
		shoot_control.given_current=(int16_t)(shoot_control.trigger_speed_mode_speed_pid.out);
		single_shoot_cnt = 0;	
	}

	if(shoot_control.shoot_mode==SHOOT_SINGLE && shoot_state == SHOOT_FINISH && single_shoot_cnt == 1)
	{
		shoot_control.speed_set=0;    
		PID_calc(&shoot_control.trigger_speed_mode_speed_pid, shoot_control.shoot_motor_measure->speed_rpm, shoot_control.speed_set);
		shoot_control.given_current=(int16_t)(shoot_control.trigger_speed_mode_speed_pid.out);
	}

}
shoot_control_t *shoot()
{
		shoot_feedback_update();
		shoot_set_mode();
		shoot_control_set();
		return &shoot_control;
	
}

