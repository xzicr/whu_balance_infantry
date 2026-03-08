#include "iwdg_task.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "CAN_receive.h"
void iwdg_task(void const *pvParamter)
{
    while(1)
    {
        if(chassis_move.chassis_data_->reset_flag)
        {
            uint32_t tick;
            tick = xTaskGetTickCount();
            buzzer_on(50,8399);
            osDelay(100);
            buzzer_off();
            buzzer_on(50,8399);
            osDelay(100);
            buzzer_off();        
            buzzer_on(50,8399);
            osDelay(100);
            buzzer_off();  
            while((xTaskGetTickCount() - tick) < pdMS_TO_TICKS(1000))
            {
                if(chassis_move.chassis_posture_info.pitch_angle>0)
                {
                    CAN_LK_Boradcast_Control(400,-400,0,0);
                }
                else
                {
                    CAN_LK_Boradcast_Control(-400,400,0,0);
                } 
            }
             CAN_LK_Boradcast_Control(0,0,0,0);
             osDelay(100);
            __disable_irq();              // 关闭全局中断
            HAL_NVIC_SystemReset();       // 执行系统复位        
        }
        osDelay(2000);
    }
}