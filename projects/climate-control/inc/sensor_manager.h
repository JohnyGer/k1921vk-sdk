/**
 * @file    sensor_manager.h
 * @brief   Temperature Sensor Manager
 * @author  Climate Control Team
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

#define SENSOR_COUNT 11
#define SENSOR_TIMEOUT_MS 5000

/* Function prototypes */
void SENSOR_MGR_Init(void);
void SENSOR_MGR_Update(uint8_t sensor_id, float temperature);
bool SENSOR_MGR_GetTemperature(uint8_t sensor_id, float* temp);
bool SENSOR_MGR_IsOnline(uint8_t sensor_id);
void SENSOR_MGR_GetAllRoomTemperatures(float* temps, uint8_t* count);
void SENSOR_MGR_GetOutdoorTemperatures(float* temps, uint8_t* count);
void SENSOR_MGR_CheckTimeouts(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
