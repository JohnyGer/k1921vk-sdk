/**
 * @file    gpio_hal.h
 * @brief   GPIO Hardware Abstraction Layer
 * @author  Climate Control Team
 */

#ifndef GPIO_HAL_H
#define GPIO_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

/* Function prototypes */
void GPIO_HAL_Init(void);
void GPIO_HAL_SetPin(GPIO_Pin_t pin, bool state);
void GPIO_HAL_TogglePin(GPIO_Pin_t pin);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_HAL_H */
