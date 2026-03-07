#include "iwdg_task.h"
#include "cmsis_os.h"
#include "chassis_task.h"
void iwdg_task(void const *pvParamter)
{
    while(1)
    {
        if(chassis_move.chassis_data_->reset_flag)
        {
            buzzer_on(50,8399);
            osDelay(100);
            buzzer_off();
            buzzer_on(50,8399);
            osDelay(100);
            buzzer_off();        
            buzzer_on(50,8399);
            osDelay(100);
            buzzer_off();             
            __disable_irq();              // 关闭全局中断
            HAL_NVIC_SystemReset();       // 执行系统复位        
        }
        osDelay(2000);
    }
}