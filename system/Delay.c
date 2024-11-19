#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "core_cm4.h"
#include "misc.h"
static uint8_t  fac_us = 0;
static uint16_t fac_ms = 0;

// Delay configuration. SYSCLK means current clock frequency (MHZ)
void DELAY_Init(uint8_t SYSCLK)
{
// If define OS_CRITICAL_METHOD then use ucosII
#ifdef OS_CRITICAL_METHOD
    uint32_t reload;
#endif

    // System timer configuration (divided by eight)
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    // Get time value (us)
    fac_us = SYSCLK / 8;

// If define OS_CRITICAL_METHOD then use ucosII
#ifdef OS_CRITICAL_METHOD
    // Ticks per second (K)
    reload = SYSCLK / 8;
    // Set real value of reload
    reload *= 1000000 / OS_TICKS_PER_SEC;
    // Minimum unit that can be delayed in ucosII
    fac_ms = 1000 / OS_TICKS_PER_SEC;
    // Enable systick interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    // Interrupt every (1 / OS_TICKS_PER_SEC) seconds
    SysTick->LOAD  = reload;
    // Enable systick
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
#else
    // Get time value (ms)
    fac_ms = (uint16_t)fac_us * 1000;
#endif
}

// If define OS_CRITICAL_METHOD then use ucosII
#ifdef OS_CRITICAL_METHOD
// Delay nus us
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt;
    
    // Set reload
    uint32_t reload = SysTick->LOAD;
    // Set ticks for delay
    ticks = nus * fac_us;
    tcnt = 0;
    // Lock OS schedule
    OSSchedLock();
    // Current systick value
    told = SysTick->VAL;
    
    while(1)
    {
        // Current systick value
        tnow = SysTick->VAL;
        if(tnow != told)
        {
            // Decrement count
            if(tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                // Timer has counted for one round, compensating the reload value
                tcnt += reload - tnow + told;
            }
            told = tnow;
            // Time-out
            if(tcnt >= ticks)
            {
                break;
            }
        }
    };

    // Unlock OS schedule
    OSSchedUnlock();
}

// Delay nms ms
void delay_ms(uint16_t nms)
{
    // If OS is running
    if(OSRunning == OS_TRUE && OSLockNesting == 0)
    {
        // If delay time is greater than the operating system minimum unit
        if(nms >= fac_ms)
        {
            // OS time delay
            OSTimeDly(nms / fac_ms);
        }
        // Use normal method to delay
        nms %= fac_ms;
    }
    // Normal method to delay
    delay_us((uint32_t)(nms * 1000));
}

// Not use ucosII
#else
// Delay nus us
void delay_us(uint32_t nus)
{
    uint32_t temp;
    // Set load
    SysTick->LOAD  = nus * fac_us;
    // Clear timer
    SysTick->VAL   = 0x00;
    // Enable systick
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    // Start timing until time-out
    do
    {
        temp = SysTick->CTRL;
    }
    while((temp & 0x01) && !(temp & (1<<16)));

    // Close systick
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    // Clear timer
    SysTick->VAL   = 0X00; 
}

// Delay nms ms
void delay_ms(uint16_t nms)
{
    uint16_t temp;

    // Set load
    SysTick->LOAD  = (uint32_t)nms * fac_ms;
    // Clear timer
    SysTick->VAL   = 0x00;
    // Enable systick
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    // Start timing until time-out
    do
    {
        temp = SysTick->CTRL;
    }
    while((temp & 0x01) && !(temp & (1<<16)));

    // Close systick
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
    // Clear timer
    SysTick->VAL =0X00;
}
#endif