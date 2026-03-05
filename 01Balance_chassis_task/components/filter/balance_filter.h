#ifndef __BALANCE_FILTER_H__
#define __BALANCE_FILTER_H__ 

#include "main.h"
#include "chassis_task.h"
#include "kalman_filter.h"

extern chassis_move_t chassis_move;


typedef struct {
    KalmanFilter_t kf;
    fp32 filtered_speed;
} foot_motor_kf_t;

typedef struct {
    KalmanFilter_t kf;
    fp32 filtered_dlength;
} leg_dl_kf_t;

typedef struct {
    KalmanFilter_t kf;
    fp32 filtered_angle;
} leg_angle_kf_t;

// foot卡尔曼滤波参数
#define KF_F_ESTIMATE_ERROR 0.6f
#define KF_F_PROCESS_NOISE 0.1f
#define KF_F_MEASURE_NOISE 0.8f

// dlength卡尔曼滤波参数
#define KF_L_ESTIMATE_ERROR 0.3f
#define KF_L_PROCESS_NOISE 0.1f
#define KF_L_MEASURE_NOISE 5.0f

// angle卡尔曼滤波参数
#define KF_A_ESTIMATE_ERROR 2.4f
#define KF_A_PROCESS_NOISE 0.1f
#define KF_A_MEASURE_NOISE 0.1f

#define CONTROL_GAIN 0.0005759f // 控制量到速度的增益系数，需要根据实际系统调整

//外部调用功能函数
void FootMotor_Kalman_Init(chassis_move_t *chassis);
void FootMotor_Kalman_Update(chassis_move_t *chassis);
void Record_FootMotor_Control(chassis_move_t *chassis);

void Leg_dlength_Kalman_Init(chassis_move_t *chassis);
void Leg_dlength_Kalman_Update(chassis_move_t *chassis);
void Record_Leg_Control(chassis_move_t *chassis);

void Leg_angle_Kalman_Init(chassis_move_t *chassis);
void Leg_angle_Kalman_Update(chassis_move_t *chassis);

#endif
