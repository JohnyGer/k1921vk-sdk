/**
 * @file    can_protocol.h
 * @brief   CAN Protocol Handler for sensor/actuator communication
 * @author  Climate Control Team
 */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

/* Sensor data structure */
typedef struct {
    float temperature;      /* Temperature in Celsius */
    uint8_t sensor_id;      /* Sensor identifier */
    uint32_t timestamp;     /* Message timestamp */
    bool valid;             /* Data validity flag */
} SensorData_t;

/* Actuator command structure */
typedef struct {
    bool enable;            /* ON/OFF command */
    uint8_t power_level;    /* Power level 0-100% */
    uint8_t mode;           /* Heating/Cooling mode */
} ActuatorCmd_t;

/* Function prototypes */
void CAN_PROTO_Init(void);
bool CAN_PROTO_ParseSensorMsg(uint32_t id, const uint8_t* data, uint8_t len, SensorData_t* out);
void CAN_PROTO_BuildActuatorCmd(uint32_t id, const ActuatorCmd_t* cmd, uint8_t* data, uint8_t* len);
bool CAN_PROTO_ParseStatusMsg(uint32_t id, const uint8_t* data, uint8_t len, DeviceStatus_t* out);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */
