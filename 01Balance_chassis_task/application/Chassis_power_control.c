#include "main.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "Chassis_power_control.h"
#include "referee.h"
float Plimit=1;
static float Power_Buffer;

extern power_heat_data_t power_heat_data;


void chassis_power_limit(chassis_move_t *chassis_move_control)
{

	/*缓冲能量占比环-总体约束*/
	Power_Buffer=power_heat_data.buffer_energy;

	if(Power_Buffer<60&&Power_Buffer>=50)		Plimit=0.95;
	else if(Power_Buffer<50&&Power_Buffer>=40)	Plimit=0.9;
	else if(Power_Buffer<40&&Power_Buffer>=35)	Plimit=0.75;
	else if(Power_Buffer<35&&Power_Buffer>=30)	Plimit=0.5;
	else if(Power_Buffer<30&&Power_Buffer>=20)	Plimit=0.25;
	else if(Power_Buffer<20&&Power_Buffer>=10)	Plimit=0.125;
	else if(Power_Buffer<10&&Power_Buffer>=0)	Plimit=0.05;
	else if(Power_Buffer==60)					Plimit=1;
	chassis_move_control->chassis_posture_info.foot_speed_set =  Plimit*chassis_move_control->chassis_posture_info.foot_speed_set;
}
