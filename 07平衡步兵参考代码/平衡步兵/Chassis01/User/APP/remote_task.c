#include "remote_task.h"
#include "watch_task.h"
#include "cmsis_os.h"


#define REMOTE_OVERTIME	1000

extern chassis_t chassis_move;
extern INS_t INS;
uint32_t REMOTE_TIME=10;//ps2手柄任务周期是10ms

extern vmc_leg_t right;			
extern vmc_leg_t left;

extern uint16_t adc_val[2];



/**************************************************************************
Function: Sbus Remote
Input   : none
Output  : none
Auth    : DHY (qq:965849293)
Date		: 2024
**************************************************************************/	
void Remote_task(void)
{

}


