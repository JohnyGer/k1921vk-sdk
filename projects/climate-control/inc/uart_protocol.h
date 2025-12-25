/**
 * @file    uart_protocol.h
 * @brief   UART Binary Protocol Handler
 * @author  Climate Control Team
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"

/* Protocol constants */
#define UART_PROTO_HEADER       0xAA55
#define UART_PROTO_MAX_PAYLOAD  64

/* Command types */
typedef enum {
    CMD_SET_TEMPERATURE = 0x01,
    CMD_SET_MODE = 0x02,
    CMD_MANUAL_HEATER = 0x03,
    CMD_MANUAL_AC = 0x04,
    CMD_GET_STATUS = 0x05
} CommandType_t;

/* Command structure */
typedef struct {
    CommandType_t type;
    uint8_t payload[UART_PROTO_MAX_PAYLOAD];
    uint8_t payload_len;
} Command_t;

/* System state structure for reporting */
typedef struct {
    float sensor_temps[11];
    bool sensor_online[11];
    float average_room_temp;
    float target_temp;
    bool heater_state[6];
    uint8_t heater_power[6];
    DeviceStatus_t heater_status[6];
    bool ac_state[4];
    uint8_t ac_power[4];
    DeviceStatus_t ac_status[4];
    ControlMode_t mode;
    uint32_t error_flags;
    uint32_t system_uptime;
} SystemState_t;

/* Function prototypes */
void UART_PROTO_Init(void);
bool UART_PROTO_ParseCommand(const uint8_t* buffer, uint16_t len, Command_t* out);
uint16_t UART_PROTO_BuildStatusReport(const SystemState_t* state, uint8_t* buffer);
uint16_t UART_PROTO_CalculateChecksum(const uint8_t* data, uint16_t len);
bool UART_PROTO_ValidateChecksum(const uint8_t* frame, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* UART_PROTOCOL_H */
