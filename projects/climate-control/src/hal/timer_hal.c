/**
 * @file    timer_hal.c
 * @brief   Timer Hardware Abstraction Layer Implementation
 * @author  Climate Control Team
 */

#include "timer_hal.h"
#include "plib028.h"
#include "plib028_tmr.h"
#include "plib028_rcu.h"

/* Private variables */
static void (*timer_callback)(void) = NULL;

/**
 * @brief Initialize timer for periodic interrupts
 * @param period_ms Period in milliseconds
 */
void TIMER_HAL_Init(uint32_t period_ms) {
    /* Enable timer clock */
    RCU_TMRClkCmd(RCU_TMRClk_0, ENABLE);
    
    /* Configure timer */
    TMR_Init_TypeDef tmr_init;
    TMR_StructInit(&tmr_init);
    
    /* Assuming 80MHz system clock, prescale to 80kHz (1us tick) */
    tmr_init.Prescaler = 1000 - 1;  /* 80MHz / 1000 = 80kHz */
    tmr_init.Period = period_ms * 80 - 1;  /* period_ms * 80000 cycles = ms in 80kHz */
    tmr_init.CountMode = TMR_CountMode_Up;
    
    TMR_Init(NT_TMR0, &tmr_init);
    
    /* Enable timer update interrupt */
    TMR_ITConfig(NT_TMR0, TMR_IT_Update, ENABLE);
    
    /* Enable NVIC interrupt */
    NVIC_EnableIRQ(TMR0_IRQn);
    NVIC_SetPriority(TMR0_IRQn, 3);
}

/**
 * @brief Start timer
 */
void TIMER_HAL_Start(void) {
    TMR_Cmd(NT_TMR0, ENABLE);
}

/**
 * @brief Stop timer
 */
void TIMER_HAL_Stop(void) {
    TMR_Cmd(NT_TMR0, DISABLE);
}

/**
 * @brief Register callback function for timer interrupt
 * @param callback Function pointer to callback
 */
void TIMER_HAL_RegisterCallback(void (*callback)(void)) {
    timer_callback = callback;
}

/**
 * @brief Timer interrupt handler (called from IRQ handler)
 */
void TIMER_HAL_IRQHandler(void) {
    if (TMR_ITStatus(NT_TMR0, TMR_IT_Update)) {
        TMR_ITStatusClear(NT_TMR0, TMR_IT_Update);
        
        if (timer_callback != NULL) {
            timer_callback();
        }
    }
}
