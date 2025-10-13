/**************************************************************************
Name		: watch task
Input   : none
Output  : none
Auth    : DHY (qq:965849293)
Date		: 2024
**************************************************************************/	
#include "watch_task.h"
#include "tim.h"

extern chassis_t chassis_move;
float VBAT_WARNNING_VAL = 0.0f;
float VBAT_LOW_VAL = 0.0f;

uint32_t WATCH_TIME=2000;//1S

/**************************************************************************
Function: Watch_Task
Des			: Alert voltage,remoter
Input   : none
Output  : none
Auth    : DHY (qq:965849293)
Date		: 2024
**************************************************************************/	
void Watch_task(void)
{
	while(1)
	{

		osDelay(WATCH_TIME);
	}
}
