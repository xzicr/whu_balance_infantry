#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H


#include "stdint.h"
#include "ins_task.h"
#include "chassis_task.h"
#include "main.h"


extern void Observe_task(void);
extern void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);
extern fp32  RAMP_float(fp32 final, fp32 now, fp32 ramp);	
extern float apply_limits_and_filter(float current_speed, float raw_speed);
#endif





