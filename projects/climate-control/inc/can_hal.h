/**
 * @file    can_hal.h
 * @brief   CAN Hardware Abstraction Layer
 * @author  Climate Control Team
 */

#ifndef CAN_HAL_H
#define CAN_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

/* CAN configuration structure */
typedef struct {
    uint32_t baudrate;
    bool loopback_mode;
    bool silent_mode;
} CAN_Config_t;

/* Function prototypes */
void CAN_HAL_Init(CAN_Channel_t channel, uint32_t baudrate);
int CAN_HAL_Send(CAN_Channel_t channel, uint32_t id, const uint8_t* data, uint8_t len);
int CAN_HAL_Receive(CAN_Channel_t channel, uint32_t* id, uint8_t* data, uint8_t* len);
void CAN_HAL_SetFilter(CAN_Channel_t channel, const uint32_t* filter_ids, uint8_t count);
bool CAN_HAL_IsTxReady(CAN_Channel_t channel);
bool CAN_HAL_IsRxAvailable(CAN_Channel_t channel);

#ifdef __cplusplus
}
#endif

#endif /* CAN_HAL_H */
