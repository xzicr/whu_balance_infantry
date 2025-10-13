/**
  *********************************************************************
  * @file      chassis_task.c/h
  * @brief     该任务控制左半部分的电机，分别是四个DM8009和两个M3508改，这六个电机挂载在can2总线上
	*						 从底盘上往下看，左上角的DM8009发送id为8、接收id为4，
	*						 左下角的DM8009发送id为6、接收id为3，
	*						 左边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassis_task.h"
#include "fdcan.h"
#include "VMC_calc.h"
#include "user_lib.h"

#include "INS_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "uart_bsp.h"

#define LimitMax(input, max)   \
{                          \
		if (input > max)       \
		{                      \
				input = max;       \
		}                      \
		else if (input < -max) \
		{                      \
				input = -max;      \
		}                      \
}

float LQR_K_START[12]={ 
   15.3788,   1.8496,   0,   0,   14.7706,   2.3751,
   0,   0,     0,     0,  0,   0};	             	

float LQR_K[4][10]={ //调试可站 调试中 0.15
    1.6404,    2.8061,    4.2202,    1.3884,   10.4381,    0.6518,    5.6533,    0.3986,   20.5323,    1.1015,
    1.6404,    2.8061,   -4.2202,   -1.3884,    5.6533,    0.3986,   10.4381,    0.6518,   20.5323,    1.1015,
    4.8052,    8.1150,    4.6801,    1.5313,   43.0124,    2.3802,   -9.6527,   -0.1742,  -74.8202,   -2.5287,
    4.8052,    8.1150,   -4.6801,   -1.5313,   -9.6527,   -0.1742,   43.0124,    2.3802,  -74.8202,   -2.5287};	 
	 
fp32 roll_PD[2] = {200,15}; 							//	 500,30		  
fp32 stand_PD[2] = {1500,100};						//  1500,100 
fp32 jump_PD[2] = {3000,100};
/*********************************************************************************/
static chassis_behaviour_e  chassis_behaviour      = CHASSIS_ZERO_FORCE;//行为层模式设置
static chassis_behaviour_e  chassis_behaviour_last = CHASSIS_ZERO_FORCE;//行为层模式设置
/*********************************************************************************/
chassis_t chassis_move;	
gc_t gc;	 
vmc_leg_t balance_L;			
vmc_leg_t balance_R;			
Ordinary_Least_Squares_t OLS_S0_L;
Ordinary_Least_Squares_t OLS_S0_R;
Ordinary_Least_Squares_t OLS_angle_L;
Ordinary_Least_Squares_t OLS_angle_R;		

extern INS_t INS;		
uint32_t CHASS_TIME=1;	
void chassis_task(void)
{
  while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}	
	
	chassis_init(&chassis_move,&balance_L,&balance_R,&gc);
	while(1)
	{	
		chassis_feedback_update(&chassis_move,&balance_L,&balance_R,&INS);//更新数据
		
		chassis_control_loop(&chassis_move,&gc,&balance_L,&balance_R,&INS);//控制计算

    	if(chassis_move.start_flag==2) {
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,balance_R.torque_set[1]);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,-balance_L.torque_set[1]);
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,balance_R.torque_set[0]);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,-balance_L.torque_set[0]);
			osDelay(CHASS_TIME);
			CAN_cmd_chassis_All3508(-chassis_move.wheel_motor[0].given_current,chassis_move.wheel_motor[1].given_current);		
			osDelay(CHASS_TIME);	
		}
    	else if(chassis_move.start_flag==1) {
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,-1.0);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,1.0);
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,1.0);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,-1.0);
			osDelay(CHASS_TIME);
			CAN_cmd_chassis_All3508(-chassis_move.wheel_motor[0].given_current,chassis_move.wheel_motor[1].given_current);		
			osDelay(CHASS_TIME);
		}		
		else if(chassis_move.start_flag==0) {
		  mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
			osDelay(CHASS_TIME);
			CAN_cmd_chassis_All3508(-chassis_move.wheel_motor[0].given_current,chassis_move.wheel_motor[1].given_current);			
			osDelay(CHASS_TIME);			
		}	
	}
}

void chassis_init(chassis_t *chassis,vmc_leg_t *vmc_l,vmc_leg_t *vmc_r,gc_t *gc)
{
	//左腿电机初始化
	joint_motor_init(&chassis->joint_motor[2],0x01,MIT_MODE);//发送id为0x01
	joint_motor_init(&chassis->joint_motor[3],0x02,MIT_MODE);//发送id为0x02	
	//右腿电机初始化
	joint_motor_init(&chassis->joint_motor[0],0x01,MIT_MODE);//发送id为0x01
	joint_motor_init(&chassis->joint_motor[1],0x02,MIT_MODE);//发送id为0x02	
	
	//给杆长赋值
	VMC_init(vmc_l);
	VMC_init(vmc_r);
	
	//物理量赋值
	gc->g=9.8f;
	gc->Mb=5.0f;
	gc->Ml=1.3f;
	gc->ksi_l=0.1f;
	gc->Rl=0.24f;
	
	OLS_Init(&OLS_S0_L,2);
	OLS_Init(&OLS_S0_R,2);
	OLS_Init(&OLS_angle_L,2);
	OLS_Init(&OLS_angle_R,2);	
	
	for(int j=0;j<10;j++){
	  enable_motor_mode(&hfdcan2,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++){
	  enable_motor_mode(&hfdcan2,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++){
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++){
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	  osDelay(1);
	}	

	if(!chassis->joint_motor[0].para.state) 
		enable_motor_mode(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	if(!chassis->joint_motor[1].para.state) 
		enable_motor_mode(&hfdcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);	
	if(!chassis->joint_motor[2].para.state) 
		enable_motor_mode(&hfdcan2,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);	
	if(!chassis->joint_motor[3].para.state) 
		enable_motor_mode(&hfdcan2,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);	
	
	chassis->wheel_motor[0].motor_measure = get_3508motor_measure_point(0);
	chassis->wheel_motor[1].motor_measure = get_3508motor_measure_point(1);
	
 	//获取关节角度零飘(需要限位卡初始角度)
	vmc_l->phi1_offset = chassis->joint_motor[0].para.pos;
	vmc_l->phi4_offset = chassis->joint_motor[1].para.pos;	
	vmc_r->phi1_offset = chassis->joint_motor[2].para.pos;
	vmc_r->phi4_offset = chassis->joint_motor[3].para.pos;
	//获取关节角度零飘(绝对值)
	// vmc_l->phi1_offset = -3.24959946f;	
	// vmc_l->phi4_offset = -1.18047619;	
	// vmc_r->phi1_offset = -1.59857368f;			
	// vmc_r->phi4_offset = -3.17864513f;	
}

void chassis_feedback_update(chassis_t *chassis,vmc_leg_t *vmc_l,vmc_leg_t *vmc_r,INS_t *ins)
{
	if (chassis == NULL){
			return;
	}	

	if (switch_is_down(rc_ctrl.sw2)) {
		chassis_behaviour = CHASSIS_ZERO_FORCE;
	}
	else if (switch_is_mid(rc_ctrl.sw2) || switch_is_up(rc_ctrl.sw2)) {
		chassis_behaviour=CHASSIS_FOLLOW_GIMBAL;//CHASSIS_FOLLOW_GIMBAL
	}
	else
		chassis_behaviour = CHASSIS_ZERO_FORCE;
	/******************************move层*******************************************/
	chassis->joint_motor[0].last_motor_mode = chassis->joint_motor[0].motor_mode;
	chassis->joint_motor[1].last_motor_mode = chassis->joint_motor[1].motor_mode;
	chassis->joint_motor[2].last_motor_mode = chassis->joint_motor[2].motor_mode;
	chassis->joint_motor[3].last_motor_mode = chassis->joint_motor[3].motor_mode;
	chassis->wheel_motor[0].last_motor_mode = chassis->wheel_motor[0].motor_mode;
	chassis->wheel_motor[1].last_motor_mode = chassis->wheel_motor[1].motor_mode;
	//根据云台行为状态机设置电机状态机
	if (chassis_behaviour == CHASSIS_ZERO_FORCE) {
		chassis->joint_motor[0].motor_mode = CHASSIS_MOTOR_ZERO;
		chassis->joint_motor[1].motor_mode = CHASSIS_MOTOR_ZERO;
		chassis->joint_motor[2].motor_mode = CHASSIS_MOTOR_ZERO;
		chassis->joint_motor[3].motor_mode = CHASSIS_MOTOR_ZERO;
		chassis->wheel_motor[0].motor_mode = CHASSIS_MOTOR_ZERO;
		chassis->wheel_motor[1].motor_mode = CHASSIS_MOTOR_ZERO;
	}	
	else if (chassis_behaviour == CHASSIS_FOLLOW_GIMBAL) {
		chassis->joint_motor[0].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->joint_motor[1].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->joint_motor[2].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->joint_motor[3].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->wheel_motor[0].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->wheel_motor[1].motor_mode = CHASSIS_MOTOR_FORCE;
	}
	else if (chassis_behaviour == CHASSIS_NO_FOLLOW) {
		chassis->joint_motor[0].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->joint_motor[1].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->joint_motor[2].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->joint_motor[3].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->wheel_motor[0].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->wheel_motor[1].motor_mode = CHASSIS_MOTOR_FORCE;
	}
	else if (chassis_behaviour == CHASSIS_SIT ) {
		chassis->joint_motor[0].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->joint_motor[1].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->joint_motor[2].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->joint_motor[3].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->wheel_motor[0].motor_mode = CHASSIS_MOTOR_FORCE;
		chassis->wheel_motor[1].motor_mode = CHASSIS_MOTOR_FORCE;
	}			
	
	//遥控器数据更新
	static uint16_t stand_time_temp=0;
	static uint16_t jump_time_temp=0;
	chassis->start_flag = chassis->start_flag;	
	//yaw电机状态机切换保存数据
	if (chassis->joint_motor[0].last_motor_mode == CHASSIS_MOTOR_FORCE && chassis->joint_motor[0].motor_mode == CHASSIS_MOTOR_ZERO)
	{   //之前没摊，现在瘫了，就开摆
		vmc_l->Tp=vmc_r->Tp=0;
		vmc_l->F0=vmc_r->F0=0;
		chassis->start_flag=0;   
	}
	else if ((chassis->joint_motor[0].last_motor_mode == CHASSIS_MOTOR_ZERO && chassis->joint_motor[0].motor_mode == CHASSIS_MOTOR_FORCE))
	{   //之前摊着，现在没摊
		chassis->x_set = chassis->x_filter;
		chassis->turn_set = chassis->total_yaw;
		chassis->start_flag=1;
	}	

	if(chassis->start_flag==1){
		if(stand_time_temp<500)stand_time_temp++;//500
		else {
			stand_time_temp=0;
			chassis->start_flag=2;
			chassis->x_set = chassis->x_filter;
			chassis->turn_set = chassis->total_yaw;
		}
	}else
		stand_time_temp=0;
	
/***********************************数据采集**************************************************/	
		chassis->target_v=((float)(rc_ctrl.ch3))*(0.006f);//往前大于0
		slope_following(&chassis->target_v,&chassis->v_set,0.005f);	//	坡度跟随
		if(rc_ctrl.ch3) chassis->x_set=chassis->x_filter+chassis->v_set*0.002f*CHASS_TIME;
		if(switch_is_up(rc_ctrl.sw2))
			chassis->turn_set-=0.015;
		else
			chassis->turn_set-=(rc_ctrl.ch0)*(0.00002f);
		//腿长变化
		if(switch_is_down(rc_ctrl.sw1))
			chassis->leg_set=0.11f;
		else if(switch_is_mid(rc_ctrl.sw1))
			chassis->leg_set=0.15f;	
		else if(switch_is_up(rc_ctrl.sw1))
			chassis->leg_set=0.18f;
		else 
			chassis->leg_set=0.11f;
		
		chassis->roll_target = 0.0f;
		slope_following(&chassis->roll_target,&chassis->roll_set,0.0075f);
		chassis->leg_left_set = chassis->leg_set;
		chassis->leg_right_set = chassis->leg_set;
		mySaturate(&chassis->leg_left_set,0.10f,0.25f);//腿长限幅在0.10m到0.25m之间
		mySaturate(&chassis->leg_right_set,0.10f,0.25f);//腿长限幅在0.10m到0.25m之间		
	
		//VMC数据更新
		vmc_l->phi1 = pi + LEG_OFFSET-chassis->joint_motor[0].para.pos+vmc_l->phi1_offset;
		vmc_l->phi4 = -LEG_OFFSET-chassis->joint_motor[1].para.pos+vmc_l->phi4_offset;
		vmc_r->phi1 = pi+LEG_OFFSET+chassis->joint_motor[2].para.pos-vmc_r->phi1_offset;
		vmc_r->phi4 = -LEG_OFFSET+chassis->joint_motor[3].para.pos-vmc_r->phi4_offset;
		chassis->myPith=ins->Pitch;
		chassis->myPithGyro=ins->Gyro[1];
		chassis->total_yaw=ins->YawTotalAngle;
		chassis->roll=ins->Roll;
}

uint8_t left_flag=0,right_flag=0;
void chassis_control_loop(chassis_t *chassis,gc_t *gc,vmc_leg_t *vmcl,vmc_leg_t *vmcr,INS_t *ins)
{
	VMC_calc_1_left(vmcl,ins,((float)CHASS_TIME)*2.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3*0.001秒
	VMC_calc_1_right(vmcr,ins,((float)CHASS_TIME)*2.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是3*0.001秒
	
	vmcl->d_L0 = OLS_Derivative(&OLS_S0_L,0.002f,vmcl->L0);
	vmcr->d_L0 = OLS_Derivative(&OLS_S0_R,0.002f,vmcr->L0);
	vmcl->d_alpha = OLS_Derivative(&OLS_angle_L,0.002f,vmcl->alpha);
	vmcr->d_alpha = OLS_Derivative(&OLS_angle_R,0.002f,vmcr->alpha);	
	
	if(chassis->start_flag==2){
		//轮毂电机输出力矩（3508电流控制）		
		chassis->wheel_motor[0].wheel_T=(+LQR_K[0][0]*(chassis->x_filter-chassis->x_set)
																		 +LQR_K[0][1]*(chassis->v_filter-chassis->v_set)
																		 +LQR_K[0][2]*(chassis->total_yaw-chassis->turn_set)
																		 +LQR_K[0][3]*(INS.Gyro[2]-0)
																		 +LQR_K[0][4]*(vmcl->alpha-0.0f)	
																		 +LQR_K[0][5]*(vmcl->d_alpha-0.0f)	
																		 +LQR_K[0][6]*(vmcr->alpha-0.0f)	
																		 +LQR_K[0][7]*(vmcr->d_alpha-0.0f)		
																		 +LQR_K[0][8]*(chassis->myPith-0.0f)	
																		 +LQR_K[0][9]*(chassis->myPithGyro-0.0f));
		chassis->wheel_motor[1].wheel_T=(+LQR_K[1][0]*(chassis->x_filter-chassis->x_set)
																		 +LQR_K[1][1]*(chassis->v_filter-chassis->v_set)
																		 +LQR_K[1][2]*(chassis->total_yaw-chassis->turn_set)
																		 +LQR_K[1][3]*(INS.Gyro[2]-0)
																		 +LQR_K[1][4]*(vmcl->alpha-0.0f)	
																		 +LQR_K[1][5]*(vmcl->d_alpha-0.0f)	
																		 +LQR_K[1][6]*(vmcr->alpha-0.0f)	
																		 +LQR_K[1][7]*(vmcr->d_alpha-0.0f)		
																		 +LQR_K[1][8]*(chassis->myPith-0.0f)	
																		 +LQR_K[1][9]*(chassis->myPithGyro-0.0f));
		//髋关节输出力矩				
		vmcl->Tp=(+LQR_K[2][0]*(chassis->x_filter-chassis->x_set)
						  +LQR_K[2][1]*(chassis->v_filter-chassis->v_set)
						  +LQR_K[2][2]*(chassis->total_yaw-chassis->turn_set)
						  +LQR_K[2][3]*(INS.Gyro[2]-0)
						  +LQR_K[2][4]*(vmcl->alpha-0.0f)	
						  +LQR_K[2][5]*(vmcl->d_alpha-0.0f)	
						  +LQR_K[2][6]*(vmcr->alpha-0.0f)	
						  +LQR_K[2][7]*(vmcr->d_alpha-0.0f)		
						  +LQR_K[2][8]*(chassis->myPith-0.0f)	
						  +LQR_K[2][9]*(chassis->myPithGyro-0.0f));
		vmcr->Tp=(+LQR_K[3][0]*(chassis->x_filter-chassis->x_set)
						  +LQR_K[3][1]*(chassis->v_filter-chassis->v_set)
						  +LQR_K[3][2]*(chassis->total_yaw-chassis->turn_set)
						  +LQR_K[3][3]*(INS.Gyro[2]-0)
						  +LQR_K[3][4]*(vmcl->alpha-0.0f)	
						  +LQR_K[3][5]*(vmcl->d_alpha-0.0f)	
						  +LQR_K[3][6]*(vmcr->alpha-0.0f)	
						  +LQR_K[3][7]*(vmcr->d_alpha-0.0f)		
						  +LQR_K[3][8]*(chassis->myPith-0.0f)	
						  +LQR_K[3][9]*(chassis->myPithGyro-0.0f));						
	}
	else{
		//轮毂电机输出力矩（3508电流控制）		
		chassis->wheel_motor[0].wheel_T=(+LQR_K_START[0]*(vmcl->alpha-0.0f)
										+LQR_K_START[1]*(vmcl->d_alpha-0.0f)
										+LQR_K_START[4]*(chassis->myPith-0.0f)
										+LQR_K_START[5]*(chassis->myPithGyro-0.0f));	
		chassis->wheel_motor[1].wheel_T=(+LQR_K_START[0]*(vmcr->alpha-0.0f)
										+LQR_K_START[1]*(vmcr->d_alpha-0.0f)
										+LQR_K_START[4]*(chassis->myPith-0.0f)
										+LQR_K_START[5]*(chassis->myPithGyro-0.0f));	
		vmcl->Tp=0;
		vmcr->Tp=0;
	}
	float F_phi,F_blG,F_blin,F_l,F_r;
	float F_ll,F_lr;
	if(chassis->start_flag==2) {
		F_phi = roll_PD[0]*(chassis->roll_set-chassis->roll)+roll_PD[1]*(0-INS.Gyro[0]);
		F_blG = (0.5*gc->Mb+gc->ksi_l*gc->Ml)*gc->g;
		F_blin = (0.5*gc->Mb+gc->ksi_l*gc->Ml)*(0.25*(vmcl->L0+vmcr->L0)*ins->Gyro[2]*chassis->v_filter)/gc->Rl;
		//调试腿部使用
		F_ll = stand_PD[0]*(chassis->leg_set-vmcl->L0)+stand_PD[1]*(0-vmcl->d_L0);
		F_lr = stand_PD[0]*(chassis->leg_set-vmcr->L0)+stand_PD[1]*(0-vmcr->d_L0);		
		
		vmcl->F0 = F_ll+F_phi+F_blG-F_blin;
		vmcr->F0 = F_lr-F_phi+F_blG+F_blin;
		
		left_flag=ground_detectionL(vmcl,ins);//右腿离地检测
		right_flag=ground_detectionR(vmcr,ins);//右腿离地检测			
	}
	if(right_flag==1&&left_flag==1) {
		//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
		chassis->wheel_motor[0].wheel_T=0.0f;
		chassis->wheel_motor[1].wheel_T=0.0f;
		
		vmcl->Tp=LQR_K[2][4]*(vmcl->alpha-0.0f)+LQR_K[2][5]*(vmcl->d_alpha-0.0f)
				+LQR_K[2][6]*(vmcr->alpha-0.0f)+LQR_K[2][7]*(vmcr->d_alpha-0.0f);
		vmcr->Tp=LQR_K[3][4]*(vmcl->alpha-0.0f)+LQR_K[3][5]*(vmcl->d_alpha-0.0f)
				+LQR_K[3][6]*(vmcr->alpha-0.0f)+LQR_K[3][7]*(vmcr->d_alpha-0.0f);

		chassis->x_filter=0.0f;
		chassis->x_set=chassis->x_filter;
		chassis->turn_set=chassis->total_yaw;
	}	
	
	mySaturate(&vmcl->F0,-100.0f,100.0f);//限幅 
	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅 
	
	VMC_calc_2L(vmcl);//计算期望的关节输出力矩
	VMC_calc_2R(vmcr);//计算期望的关节输出力矩
	chassis->wheel_motor[0].given_current = (int16_t)(chassis->wheel_motor[0].wheel_T*(19/(268/17.0f))*(8/3.0f)*(16384/20.0f));
	chassis->wheel_motor[1].given_current = (int16_t)(chassis->wheel_motor[1].wheel_T*(19/(268/17.0f))*(8/3.0f)*(16384/20.0f));	
		
	if(!chassis->start_flag){
		chassis->wheel_motor[0].given_current =  rc_ctrl.ch3*5;
		chassis->wheel_motor[1].given_current =  rc_ctrl.ch3*5;
		balance_L.torque_set[0] = 0;
		balance_L.torque_set[1] = 0;
		balance_R.torque_set[0] = 0;
		balance_R.torque_set[1] = 0;		
	}
	
	//额定扭矩
	mySaturate(&vmcl->torque_set[1],-10.0f,10.0f);	
	mySaturate(&vmcl->torque_set[0],-10.0f,10.0f);	
	mySaturate(&vmcr->torque_set[1],-10.0f,10.0f);	
	mySaturate(&vmcr->torque_set[0],-10.0f,10.0f);		
	
	LimitMax(chassis->wheel_motor[1].given_current,16000);	
	LimitMax(chassis->wheel_motor[0].given_current,16000);	   
}

/**************功率控制*********************/
fp32 power_control(chassis_t *power_control)
{
	float limit_coefficient;
	/**********************计算当前底盘功率*********************/

	// first_order_filter_cali(&power_limit_filter,limit_coefficient);
	mySaturate(&limit_coefficient,0.0f,1.0f);
  return limit_coefficient;
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}

void slope_following(float *target,float *set,float acc)
{
	if(*target > *set)
	{
		*set = *set + acc;
		if(*set >= *target)
		*set = *target;
	}
	else if(*target < *set)
	{
		*set = *set - acc;
		if(*set <= *target)
		*set = *target;
	}
}

