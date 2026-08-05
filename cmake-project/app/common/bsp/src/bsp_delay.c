#include "bsp_delay.h"

#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"


void bsp_delay_ms(uint32_t milliseconds)
{
    TickType_t delay_ticks;

    if (milliseconds == 0U)
    {
        return;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        delay_ticks = pdMS_TO_TICKS(milliseconds);

        /*
         * 当 FreeRTOS tick 频率较低时，小延时换算结果可能是 0。
         * 至少延时一个 tick。
         */
        if (delay_ticks == 0U)
        {
            delay_ticks = 1U;
        }

        vTaskDelay(delay_ticks);
    }
    else
    {
        HAL_Delay(milliseconds);
    }
}

void bsp_delay_us(uint32_t microseconds)
{
    uint32_t cycles_per_microsecond;
    uint32_t delay_cycles;
    uint32_t start_cycles;
    uint32_t current_chunk_us;

    if (microseconds == 0U)
    {
        return;
    }

    /*
     * 允许使用 Cortex-M4 的 DWT 周期计数器。
     * 不清零 CYCCNT，避免影响调试器或其他性能测量代码。
     */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cycles_per_microsecond =
        SystemCoreClock / 1000000U;

    while (microseconds > 0U)
    {
        /*
         * 分段计算，避免 microseconds 很大时乘法溢出。
         * 在 168 MHz 下，1000000 us 对应 168000000 个周期。
         */
        if (microseconds > 1000000U)
        {
            current_chunk_us = 1000000U;
        }
        else
        {
            current_chunk_us = microseconds;
        }

        delay_cycles =
            cycles_per_microsecond * current_chunk_us;

        start_cycles = DWT->CYCCNT;

        while ((uint32_t)(DWT->CYCCNT - start_cycles) <
               delay_cycles)
        {
            __NOP();
        }

        microseconds -= current_chunk_us;
    }
}