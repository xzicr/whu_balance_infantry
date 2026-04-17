#include "motor_cmd.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include "cmsis_os.h"

void motor_cmd_task(void const *pvParameters)
{
    osDelay(500);
    while(1)
    {
        CAN_LK_Boradcast_Control(chassis_move.foot_motor_L.torque_out,chassis_move.foot_motor_R.torque_out,0,0);
        osDelay(1);
    }
}