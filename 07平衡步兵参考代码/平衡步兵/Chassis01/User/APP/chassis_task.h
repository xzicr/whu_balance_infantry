#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"

#define LEG_OFFSET 0.578f

typedef struct
{
	float Mb;
	float g;
	float Ml;
	float Rl;
	float ksi_l;
}gc_t;
extern gc_t gc;

typedef enum
{
    CHASSIS_ZERO_FORCE = 0,
	  CHASSIS_PURE,
	  CHASSIS_PURE_SIT,
	  CHASSIS_PURE_HIGH,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_NO_FOLLOW,
    CHASSIS_SIT,
} chassis_behaviour_e;

typedef struct
{
  Joint_Motor_t joint_motor[4];
  Wheel_Motor_t wheel_motor[2];
	
	float v_set;//�����ٶȣ���λ��m/s
	float target_v;
	float x_set;//����λ�ã���λ��m
	float turn_set;//����yaw�ỡ��
	float target_turn;
	float leg_set;//�����ȳ�����λ��m
	float leg_lx_set;
	float target_leg_lx_set;
	float leg_left_set;
	float leg_right_set;
	float last_leg_set;
	float last_leg_left_set;
	float last_leg_right_set;
	float roll_set;
	float roll_target;
	float now_roll_set;

	float v_filter;//�˲���ĳ����ٶȣ���λ��m/s
	float x_filter;//�˲���ĳ���λ�ã���λ��m

	float myPith;
	float myPithGyro;
	float roll;
	float total_yaw;
	float theta_err;//���ȼн����
		
	float turn_T;//yaw�Ჹ��
	float leg_tp;//�����油��
	
	float F0;
	float Tp;
	
	uint8_t start_flag;//������־
	uint8_t last_start_flag;
	
	uint8_t recover_flag;//һ������µĵ��������־
	
	uint32_t count_key;
	uint8_t jump_flag;
	float jump_leg;
	uint32_t jump_time_r;
	uint32_t jump_time_l;
	uint8_t jump_status_r;
	uint8_t jump_status_l;
} chassis_t;
extern chassis_t chassis_move;		
extern vmc_leg_t balance_L;			
extern vmc_leg_t balance_R;		

extern void chassis_task(void);
extern void chassis_init(chassis_t *chassis,vmc_leg_t *vmc_l,vmc_leg_t *vmc_r,gc_t *gc);
extern void mySaturate(float *in,float min,float max);
extern void chassis_feedback_update(chassis_t *chassis,vmc_leg_t *vmc_l,vmc_leg_t *vmc_r,INS_t *ins);
extern void chassis_control_loop(chassis_t *chassis,gc_t *gc,vmc_leg_t *vmcl,vmc_leg_t *vmcr,INS_t *ins);
extern void slope_following(float *target,float *set,float acc);
extern fp32 power_control(chassis_t *power_control);
#endif



