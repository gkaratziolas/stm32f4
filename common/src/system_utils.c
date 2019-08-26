#include "stm32f4xx.h"
#include "system_utils.h"

static uint32_t uptime_ms = 0;

void SysTick_Handler(void)
{
        uptime_ms++;
}

void system_uptime_ms_init(){
        SysTick_Config(SystemCoreClock / 1000);
}

uint32_t system_get_uptime_ms(void)
{
        return uptime_ms;
}

void system_reset_uptime_ms(void)
{
        uptime_ms = 0;
}
