/**
 * @file    sensor_manager.c
 * @brief   Temperature Sensor Manager Implementation
 * @author  Climate Control Team
 */

#include "sensor_manager.h"
#include "plib028.h"
#include <string.h>

/* Sensor data structure */
typedef struct {
    float temperature;
    uint32_t last_update;
    bool online;
    bool valid;
    DeviceStatus_t status;
} Sensor_t;

/* Private variables */
static Sensor_t sensors[SENSOR_COUNT];
static uint32_t system_tick = 0;

/**
 * @brief Initialize sensor manager
 */
void SENSOR_MGR_Init(void) {
    memset(sensors, 0, sizeof(sensors));
    system_tick = 0;
}

/**
 * @brief Update sensor temperature reading
 * @param sensor_id Sensor ID (0-10)
 * @param temperature Temperature value
 */
void SENSOR_MGR_Update(uint8_t sensor_id, float temperature) {
    if (sensor_id >= SENSOR_COUNT) {
        return;
    }
    
    sensors[sensor_id].temperature = temperature;
    sensors[sensor_id].last_update = system_tick;
    sensors[sensor_id].online = true;
    sensors[sensor_id].valid = true;
    sensors[sensor_id].status = STATUS_OK;
}

/**
 * @brief Get temperature from specific sensor
 * @param sensor_id Sensor ID
 * @param temp Pointer to store temperature
 * @return true if sensor online and valid, false otherwise
 */
bool SENSOR_MGR_GetTemperature(uint8_t sensor_id, float* temp) {
    if (sensor_id >= SENSOR_COUNT || temp == NULL) {
        return false;
    }
    
    if (sensors[sensor_id].online && sensors[sensor_id].valid) {
        *temp = sensors[sensor_id].temperature;
        return true;
    }
    
    return false;
}

/**
 * @brief Check if sensor is online
 * @param sensor_id Sensor ID
 * @return true if online, false otherwise
 */
bool SENSOR_MGR_IsOnline(uint8_t sensor_id) {
    if (sensor_id >= SENSOR_COUNT) {
        return false;
    }
    
    return sensors[sensor_id].online;
}

/**
 * @brief Get all room temperature readings (sensors 0-8)
 * @param temps Output array for temperatures
 * @param count Output number of valid temperatures
 */
void SENSOR_MGR_GetAllRoomTemperatures(float* temps, uint8_t* count) {
    if (temps == NULL || count == NULL) {
        return;
    }
    
    *count = 0;
    for (uint8_t i = 0; i < 9; i++) {  /* First 9 are room sensors */
        if (sensors[i].online && sensors[i].valid) {
            temps[*count] = sensors[i].temperature;
            (*count)++;
        }
    }
}

/**
 * @brief Get outdoor temperature readings (sensors 9-10)
 * @param temps Output array for temperatures
 * @param count Output number of valid temperatures
 */
void SENSOR_MGR_GetOutdoorTemperatures(float* temps, uint8_t* count) {
    if (temps == NULL || count == NULL) {
        return;
    }
    
    *count = 0;
    for (uint8_t i = 9; i < 11; i++) {  /* Last 2 are outdoor sensors */
        if (sensors[i].online && sensors[i].valid) {
            temps[*count] = sensors[i].temperature;
            (*count)++;
        }
    }
}

/**
 * @brief Check for sensor timeouts
 */
void SENSOR_MGR_CheckTimeouts(void) {
    system_tick++;
    
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (sensors[i].online) {
            uint32_t time_since_update = system_tick - sensors[i].last_update;
            if (time_since_update > SENSOR_TIMEOUT_MS) {
                sensors[i].online = false;
                sensors[i].valid = false;
                sensors[i].status = STATUS_ERROR;
            }
        }
    }
}
