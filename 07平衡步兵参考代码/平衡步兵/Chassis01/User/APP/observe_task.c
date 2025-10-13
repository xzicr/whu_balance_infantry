/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     该任务是对机体运动速度估计，用于抑制打滑
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "observe_task.h"
#include "kalman_filter.h"
#include "cmsis_os.h"

// 定义全局变量
float lspeed_filtered = 0.0f; // 左轮滤波后的速度
float rspeed_filtered = 0.0f; // 右轮滤波后的速度

// 滤波器参数
const float alpha = 0.3f; // 低通滤波系数，值越小滤波效果越强（范围：0~1）
const float max_speed_change = 0.03f; // 最大允许的速度变化量（单位：m/s）


KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {1.0f, 0.0f, 
                           0.0f, 1.0f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {200.0f, 0.0f, 
                            0.0f,  200.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
														 															 
extern INS_t INS;		
																 															 
																 
extern vmc_leg_t right;			
extern vmc_leg_t left;	

float vel_acc[2]; 
uint32_t OBSERVE_TIME=3;//任务周期是3ms														 												 
				
// 异常处理函数
float apply_limits_and_filter(float current_speed, float raw_speed) {
    // 限制速度突变
    float speed_diff = raw_speed - current_speed;
    if (speed_diff > max_speed_change) {
        raw_speed = current_speed + max_speed_change; // 上限
    } else if (speed_diff < -max_speed_change) {
        raw_speed = current_speed - max_speed_change; // 下限
    }

    // 一阶低通滤波
    return alpha * raw_speed + (1.0f - alpha) * current_speed;
}
																 
void 	Observe_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	static float aver_v=0.0f;
	
	xvEstimateKF_Init(&vaEstimateKF);

  while(1)
	{  
		wr=  chassis_move.wheel_motor[1].motor_measure->speed_rpm/163.0f-INS.Gyro[1]-balance_R.d_alpha;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vrb=wr*0.077f+balance_R.L0*balance_R.d_alpha*arm_cos_f32(balance_R.alpha)+balance_R.d_L0*arm_sin_f32(balance_R.alpha);//机体b系的速度
		
		wl= -chassis_move.wheel_motor[0].motor_measure->speed_rpm/163.0f-INS.Gyro[1]-balance_L.d_alpha;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vlb=wl*0.077f+balance_L.L0*balance_L.d_alpha*arm_cos_f32(balance_L.alpha)+balance_L.d_L0*arm_sin_f32(balance_L.alpha);//机体b系的速度
//		
//		aver_v=(vrb+vlb)/2.0f;//取平均
//    xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_n[1],aver_v);
//		
//		//原地自转的过程中v_filter和x_filter应该都是为0
//		chassis_move.v_filter=aver_v;//得到卡尔曼滤波后的速度
//		chassis_move.x_filter=chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);		
		
//	//如果想直接用轮子速度，不做融合的话可以这样
//		aver_v=(chassis_move.wheel_motor[1].motor_measure->speed_rpm-chassis_move.wheel_motor[0].motor_measure->speed_rpm)*0.077f/290.0f;//0.077是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
		
//		vlb=chassis_move.wheel_motor[1].motor_measure->speed_rpm*0.077f/290.0f;
//		vrb=-chassis_move.wheel_motor[0].motor_measure->speed_rpm*0.077f/290.0f;
		
    // 对左轮速度进行滤波和异常处理
    lspeed_filtered = apply_limits_and_filter(lspeed_filtered, vlb);
    // 对右轮速度进行滤波和异常处理
    rspeed_filtered = apply_limits_and_filter(rspeed_filtered, vrb);
		aver_v=lspeed_filtered+rspeed_filtered;
		
		chassis_move.v_filter=aver_v;//得到卡尔曼滤波后的速度
		chassis_move.x_filter+=chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		osDelay(OBSERVE_TIME);
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
	 memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
   memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

fp32  RAMP_float( fp32  final, fp32  now, fp32  ramp )
{
	  fp32  buffer = 0;
		
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}
