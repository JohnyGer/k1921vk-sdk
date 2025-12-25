/**
 * @file    timer_hal.h
 * @brief   Timer Hardware Abstraction Layer
 * @author  Climate Control Team
 */

#ifndef TIMER_HAL_H
#define TIMER_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

/* Function prototypes */
void TIMER_HAL_Init(uint32_t period_ms);
void TIMER_HAL_Start(void);
void TIMER_HAL_Stop(void);
void TIMER_HAL_RegisterCallback(void (*callback)(void));

#ifdef __cplusplus
}
#endif

#endif /* TIMER_HAL_H */
