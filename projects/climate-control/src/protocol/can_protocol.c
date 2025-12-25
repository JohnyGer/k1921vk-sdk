/**
 * @file    can_protocol.c
 * @brief   CAN Protocol Handler Implementation
 * @author  Climate Control Team
 */

#include "can_protocol.h"
#include <string.h>

/* CAN message ID ranges */
#define CAN0_SENSOR_BASE    0x100
#define CAN0_OUTDOOR_BASE   0x104
#define CAN1_SENSOR_BASE    0x200
#define CAN0_ACTUATOR_BASE  0x110
#define CAN1_ACTUATOR_BASE  0x210

/**
 * @brief Initialize CAN protocol handler
 */
void CAN_PROTO_Init(void) {
    /* Nothing to initialize for now */
}

/**
 * @brief Parse temperature sensor message
 * @param id CAN message ID
 * @param data Message data
 * @param len Message length
 * @param out Output sensor data structure
 * @return true if valid sensor message, false otherwise
 */
bool CAN_PROTO_ParseSensorMsg(uint32_t id, const uint8_t* data, uint8_t len, SensorData_t* out) {
    if (data == NULL || out == NULL || len < 4) {
        return false;
    }
    
    /* Check if ID is in valid sensor range */
    uint8_t sensor_id = 0;
    if (id >= CAN0_SENSOR_BASE && id <= (CAN0_SENSOR_BASE + 5)) {
        sensor_id = (uint8_t)(id - CAN0_SENSOR_BASE);
    } else if (id >= CAN1_SENSOR_BASE && id <= (CAN1_SENSOR_BASE + 4)) {
        sensor_id = (uint8_t)(id - CAN1_SENSOR_BASE + 6);
    } else {
        return false;
    }
    
    /* Parse temperature from message (4 bytes, IEEE 754 float) */
    float temp;
    memcpy(&temp, data, sizeof(float));
    
    /* Validate temperature range (-50°C to +150°C) */
    if (temp < -50.0f || temp > 150.0f) {
        out->valid = false;
        return false;
    }
    
    out->temperature = temp;
    out->sensor_id = sensor_id;
    out->timestamp = 0; /* Will be set by sensor manager */
    out->valid = true;
    
    return true;
}

/**
 * @brief Build actuator command message
 * @param id CAN message ID
 * @param cmd Actuator command
 * @param data Output data buffer
 * @param len Output data length
 */
void CAN_PROTO_BuildActuatorCmd(uint32_t id, const ActuatorCmd_t* cmd, uint8_t* data, uint8_t* len) {
    if (cmd == NULL || data == NULL || len == NULL) {
        return;
    }
    
    /* Message format:
     * Byte 0: Enable flag (0=OFF, 1=ON)
     * Byte 1: Power level (0-100)
     * Byte 2: Mode (0=Heating, 1=Cooling)
     */
    data[0] = cmd->enable ? 1 : 0;
    data[1] = cmd->power_level > 100 ? 100 : cmd->power_level;
    data[2] = cmd->mode;
    *len = 3;
}

/**
 * @brief Parse device status message
 * @param id CAN message ID
 * @param data Message data
 * @param len Message length
 * @param out Output status
 * @return true if valid status message, false otherwise
 */
bool CAN_PROTO_ParseStatusMsg(uint32_t id, const uint8_t* data, uint8_t len, DeviceStatus_t* out) {
    if (data == NULL || out == NULL || len < 1) {
        return false;
    }
    
    /* Check if ID is in valid actuator range */
    if ((id < CAN0_ACTUATOR_BASE || id > (CAN0_ACTUATOR_BASE + 4)) &&
        (id < CAN1_ACTUATOR_BASE || id > (CAN1_ACTUATOR_BASE + 4))) {
        return false;
    }
    
    /* Parse status byte */
    switch (data[0]) {
        case 0:
            *out = STATUS_OK;
            break;
        case 1:
            *out = STATUS_ERROR;
            break;
        case 2:
            *out = STATUS_RUNNING;
            break;
        case 3:
            *out = STATUS_WAITING;
            break;
        default:
            return false;
    }
    
    return true;
}
