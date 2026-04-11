/**
  ****************************(C) WHU_BAlANCE_FILTER***************************
****************************
  * @file       balance_filter.c
  * @brief      滤波函数封装
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2025     xzicr              1. 轮腿滤波封装函数
  *
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) WHU_BAlANCE_FILTER***************************
  */

#include "balance_filter.h"


foot_motor_kf_t foot_motor_kf_L, foot_motor_kf_R;  
leg_dl_kf_t leg_kf_L, leg_kf_R;
leg_angle_kf_t leg_kf_L_angle,leg_kf_R_angle;


// 添加卡尔曼滤波初始化函数
void FootMotor_Kalman_Init(chassis_move_t *chassis)
{
    if (chassis == NULL) return;
    
    // 初始化左轮电机卡尔曼滤波器
    
    // 设置初始状态矩阵
    static float P_Init[4] = {KF_F_ESTIMATE_ERROR, 0, 0, KF_F_ESTIMATE_ERROR};
    static float F_Init[4] = {1, CHASSIS_CONTROL_TIME, 0, 1}; // 状态转移矩阵
    static float Q_Init[4] = {KF_F_PROCESS_NOISE, 0, 0, KF_F_PROCESS_NOISE};
    static float H_Init[2] = {1, 0}; // 观测矩阵 [1,0] 表示只观测速度
    static float R_Init[1] = {KF_F_MEASURE_NOISE};

    Kalman_Filter_Init(&foot_motor_kf_L.kf, 2, 1, 1); // 2状态(速度,加速度), 1控制量, 1观测量
    foot_motor_kf_L.kf.UseAutoAdjustment = 0;
    memcpy(foot_motor_kf_L.kf.P_data, P_Init, sizeof(P_Init));
    memcpy(foot_motor_kf_L.kf.F_data, F_Init, sizeof(F_Init));
    memcpy(foot_motor_kf_L.kf.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(foot_motor_kf_L.kf.H_data, H_Init, sizeof(H_Init));
    memcpy(foot_motor_kf_L.kf.R_data, R_Init, sizeof(R_Init));
    // 设置控制矩阵B [dt; 1] 表示控制量对速度和加速度的影响
    foot_motor_kf_L.kf.B_data[0] = CHASSIS_CONTROL_TIME * 1;
    foot_motor_kf_L.kf.B_data[1] = 1;
    foot_motor_kf_L.filtered_speed = 0;

    

    // 初始化右轮电机卡尔曼滤波器（参数相同）
    Kalman_Filter_Init(&foot_motor_kf_R.kf, 2, 1, 1);
    foot_motor_kf_R.kf.UseAutoAdjustment = 0;
    memcpy(foot_motor_kf_R.kf.P_data, P_Init, sizeof(P_Init));
    memcpy(foot_motor_kf_R.kf.F_data, F_Init, sizeof(F_Init));
    memcpy(foot_motor_kf_R.kf.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(foot_motor_kf_R.kf.H_data, H_Init, sizeof(H_Init));
    memcpy(foot_motor_kf_R.kf.R_data, R_Init, sizeof(R_Init));
    foot_motor_kf_R.kf.B_data[0] = CHASSIS_CONTROL_TIME * 1;
    foot_motor_kf_R.kf.B_data[1] = 1;
    foot_motor_kf_R.filtered_speed = 0;


}

void FootMotor_Kalman_Update(chassis_move_t *chassis)
{
    if (chassis == NULL) return;
    
    foot_motor_kf_L.kf.MeasuredVector[0] = chassis->foot_motor_L.speed;
    foot_motor_kf_L.kf.ControlVector[0] = chassis->chassis_posture_info.x_accel;
    
    float *filtered_state = Kalman_Filter_Update(&foot_motor_kf_L.kf);
    if (filtered_state != NULL) {
        foot_motor_kf_L.filtered_speed = filtered_state[0]; // 滤波后的速度
    }
    chassis->foot_motor_L.speed_kf = foot_motor_kf_L.filtered_speed;

    foot_motor_kf_R.kf.MeasuredVector[0] = chassis->foot_motor_R.speed;
    foot_motor_kf_R.kf.ControlVector[0] = chassis->chassis_posture_info.x_accel;
    
    filtered_state = Kalman_Filter_Update(&foot_motor_kf_R.kf);
    if (filtered_state != NULL) {
        foot_motor_kf_R.filtered_speed = filtered_state[0]; // 滤波后的速度
    }
    chassis->foot_motor_R.speed_kf = foot_motor_kf_R.filtered_speed;
}


// 记录控制量函数（在发送控制指令前调用）
void Record_FootMotor_Control(chassis_move_t *chassis)
{
    if (chassis == NULL) return;
    
    // 记录当前控制量，用于下一次卡尔曼滤波更新
    chassis->foot_motor_L.last_control_torque = chassis->foot_motor_L.torque_out;
    chassis->foot_motor_R.last_control_torque = chassis->foot_motor_R.torque_out;
}


//腿部角度kalman滤波初始化
void Leg_angle_Kalman_Init(chassis_move_t *chassis)
{
    if (chassis == NULL) return;
    // 设置初始状态矩阵
    static float P_Init[4] = {KF_A_ESTIMATE_ERROR, 0, 0, KF_A_ESTIMATE_ERROR};
    static float F_Init[4] = {1, CHASSIS_CONTROL_TIME, 0, 1}; // 状态转移矩阵
    static float Q_Init[4] = {KF_A_PROCESS_NOISE, 0, 0, KF_A_PROCESS_NOISE};
    static float H_Init[2] = {1, 0}; // 观测矩阵 [1,0] 表示只观测速度
    static float R_Init[1] = {KF_A_MEASURE_NOISE};


    Kalman_Filter_Init(&leg_kf_L_angle.kf, 2, 1, 1); // 2状态(速度,加速度), 1控制量, 1观测量
    leg_kf_L_angle.kf.UseAutoAdjustment = 0;
    memcpy(leg_kf_L_angle.kf.P_data, P_Init, sizeof(P_Init));
    memcpy(leg_kf_L_angle.kf.F_data, F_Init, sizeof(F_Init));
    memcpy(leg_kf_L_angle.kf.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(leg_kf_L_angle.kf.H_data, H_Init, sizeof(H_Init));
    memcpy(leg_kf_L_angle.kf.R_data, R_Init, sizeof(R_Init));

    leg_kf_L_angle.kf.B_data[0] = CHASSIS_CONTROL_TIME * CONTROL_GAIN;
    leg_kf_L_angle.kf.B_data[1] = CONTROL_GAIN;
    leg_kf_L_angle.filtered_angle = 0;

    

    Kalman_Filter_Init(&leg_kf_R_angle.kf, 2, 1, 1);
    leg_kf_R_angle.kf.UseAutoAdjustment = 0;
    memcpy(leg_kf_R_angle.kf.P_data, P_Init, sizeof(P_Init));
    memcpy(leg_kf_R_angle.kf.F_data, F_Init, sizeof(F_Init));
    memcpy(leg_kf_R_angle.kf.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(leg_kf_R_angle.kf.H_data, H_Init, sizeof(H_Init));
    memcpy(leg_kf_R_angle.kf.R_data, R_Init, sizeof(R_Init));
    leg_kf_R_angle.kf.B_data[0] = CHASSIS_CONTROL_TIME * CONTROL_GAIN;
    leg_kf_R_angle.kf.B_data[1] = CONTROL_GAIN;
    leg_kf_R_angle.filtered_angle = 0;


}

void Leg_angle_Kalman_Update(chassis_move_t *chassis)
{
    if (chassis == NULL) return;
    
    if(chassis->mode.chassis_mode == DISABLE_CHASSIS)
    {
        leg_kf_L_angle.kf.MeasuredVector[0] = chassis->chassis_posture_info.leg_angle_L;
        leg_kf_L_angle.kf.ControlVector[0] = 0;
    }
    else 
    {
        leg_kf_L_angle.kf.MeasuredVector[0] = chassis->chassis_posture_info.leg_angle_L;
        leg_kf_L_angle.kf.ControlVector[0] = chassis->torque_info.joint_horizontal_torque_L;
    }
    float *filtered_state = Kalman_Filter_Update(&leg_kf_L_angle.kf);
    if (filtered_state != NULL) {
        leg_kf_L_angle.filtered_angle = filtered_state[0]; // 滤波后的速度
    }
    chassis->chassis_posture_info.leg_angle_L_kf = leg_kf_L_angle.filtered_angle;

    if(chassis->mode.chassis_mode == DISABLE_CHASSIS)
    {
        leg_kf_R_angle.kf.MeasuredVector[0] = chassis->chassis_posture_info.leg_angle_R;
        leg_kf_R_angle.kf.ControlVector[0] = 0;
    }
    else 
    {
        leg_kf_R_angle.kf.MeasuredVector[0] = chassis->chassis_posture_info.leg_angle_R;
        leg_kf_R_angle.kf.ControlVector[0] = chassis->torque_info.joint_horizontal_torque_R;
    }
    
    filtered_state = Kalman_Filter_Update(&leg_kf_R_angle.kf);
    if (filtered_state != NULL) {
        leg_kf_R_angle.filtered_angle = filtered_state[0]; // 滤波后的速度
    }
    chassis->chassis_posture_info.leg_angle_R_kf = leg_kf_R_angle.filtered_angle;
}
















