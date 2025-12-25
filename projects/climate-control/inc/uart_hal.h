/**
 * @file    uart_hal.h
 * @brief   UART Hardware Abstraction Layer
 * @author  Climate Control Team
 */

#ifndef UART_HAL_H
#define UART_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

/* Function prototypes */
void UART_HAL_Init(UART_Channel_t channel, uint32_t baudrate);
int UART_HAL_SendDMA(UART_Channel_t channel, const uint8_t* data, uint16_t len);
int UART_HAL_ReceiveDMA(UART_Channel_t channel, uint8_t* buffer, uint16_t len);
bool UART_HAL_IsTxComplete(UART_Channel_t channel);
uint16_t UART_HAL_GetRxCount(UART_Channel_t channel);

#ifdef __cplusplus
}
#endif

#endif /* UART_HAL_H */
