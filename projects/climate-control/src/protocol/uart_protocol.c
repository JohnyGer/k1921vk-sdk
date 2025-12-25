/**
 * @file    uart_protocol.c
 * @brief   UART Binary Protocol Handler Implementation
 * @author  Climate Control Team
 */

#include "uart_protocol.h"
#include <string.h>

/* Frame format: [Header:2][Length:2][Type:1][Payload:N][Checksum:2] */

/**
 * @brief Initialize UART protocol handler
 */
void UART_PROTO_Init(void) {
    /* Nothing to initialize for now */
}

/**
 * @brief Calculate 16-bit checksum (CRC16-like simple sum)
 * @param data Pointer to data
 * @param len Data length
 * @return Checksum value
 */
uint16_t UART_PROTO_CalculateChecksum(const uint8_t* data, uint16_t len) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief Validate frame checksum
 * @param frame Complete frame buffer
 * @param len Frame length
 * @return true if valid, false otherwise
 */
bool UART_PROTO_ValidateChecksum(const uint8_t* frame, uint16_t len) {
    if (frame == NULL || len < 7) {
        return false;
    }
    
    /* Extract checksum from frame */
    uint16_t frame_checksum = (frame[len - 2] << 8) | frame[len - 1];
    
    /* Calculate checksum of data (excluding checksum bytes) */
    uint16_t calc_checksum = UART_PROTO_CalculateChecksum(frame, len - 2);
    
    return (frame_checksum == calc_checksum);
}

/**
 * @brief Parse command frame from laptop
 * @param buffer Input buffer
 * @param len Buffer length
 * @param out Output command structure
 * @return true if valid command, false otherwise
 */
bool UART_PROTO_ParseCommand(const uint8_t* buffer, uint16_t len, Command_t* out) {
    if (buffer == NULL || out == NULL || len < 7) {
        return false;
    }
    
    /* Validate header */
    uint16_t header = (buffer[0] << 8) | buffer[1];
    if (header != UART_PROTO_HEADER) {
        return false;
    }
    
    /* Parse length */
    uint16_t payload_len = (buffer[2] << 8) | buffer[3];
    if (payload_len > UART_PROTO_MAX_PAYLOAD || (payload_len + 7) > len) {
        return false;
    }
    
    /* Validate checksum */
    if (!UART_PROTO_ValidateChecksum(buffer, payload_len + 7)) {
        return false;
    }
    
    /* Parse command type */
    out->type = (CommandType_t)buffer[4];
    out->payload_len = (uint8_t)payload_len;
    
    /* Copy payload */
    if (payload_len > 0) {
        memcpy(out->payload, &buffer[5], payload_len);
    }
    
    return true;
}

/**
 * @brief Build status report frame
 * @param state System state
 * @param buffer Output buffer (must be large enough)
 * @return Frame length
 */
uint16_t UART_PROTO_BuildStatusReport(const SystemState_t* state, uint8_t* buffer) {
    if (state == NULL || buffer == NULL) {
        return 0;
    }
    
    uint16_t idx = 0;
    
    /* Header */
    buffer[idx++] = (UART_PROTO_HEADER >> 8) & 0xFF;
    buffer[idx++] = UART_PROTO_HEADER & 0xFF;
    
    /* Reserve space for length (will fill later) */
    uint16_t len_idx = idx;
    idx += 2;
    
    /* Type (0x80 = status report) */
    buffer[idx++] = 0x80;
    
    /* Payload start */
    uint16_t payload_start = idx;
    
    /* Timestamp/uptime (4 bytes) */
    buffer[idx++] = (state->system_uptime >> 24) & 0xFF;
    buffer[idx++] = (state->system_uptime >> 16) & 0xFF;
    buffer[idx++] = (state->system_uptime >> 8) & 0xFF;
    buffer[idx++] = state->system_uptime & 0xFF;
    
    /* Average room temperature (4 bytes float) */
    memcpy(&buffer[idx], &state->average_room_temp, sizeof(float));
    idx += sizeof(float);
    
    /* Target temperature (4 bytes float) */
    memcpy(&buffer[idx], &state->target_temp, sizeof(float));
    idx += sizeof(float);
    
    /* Sensor data - 11 sensors * 5 bytes each (4 temp + 1 online) */
    for (uint8_t i = 0; i < 11; i++) {
        memcpy(&buffer[idx], &state->sensor_temps[i], sizeof(float));
        idx += sizeof(float);
        buffer[idx++] = state->sensor_online[i] ? 1 : 0;
    }
    
    /* Heater states - 6 heaters * 3 bytes each (1 state + 1 power + 1 status) */
    for (uint8_t i = 0; i < 6; i++) {
        buffer[idx++] = state->heater_state[i] ? 1 : 0;
        buffer[idx++] = state->heater_power[i];
        buffer[idx++] = (uint8_t)state->heater_status[i];
    }
    
    /* AC states - 4 ACs * 3 bytes each */
    for (uint8_t i = 0; i < 4; i++) {
        buffer[idx++] = state->ac_state[i] ? 1 : 0;
        buffer[idx++] = state->ac_power[i];
        buffer[idx++] = (uint8_t)state->ac_status[i];
    }
    
    /* Control mode (1 byte) */
    buffer[idx++] = (uint8_t)state->mode;
    
    /* Error flags (4 bytes) */
    buffer[idx++] = (state->error_flags >> 24) & 0xFF;
    buffer[idx++] = (state->error_flags >> 16) & 0xFF;
    buffer[idx++] = (state->error_flags >> 8) & 0xFF;
    buffer[idx++] = state->error_flags & 0xFF;
    
    /* Calculate payload length */
    uint16_t payload_len = idx - payload_start;
    buffer[len_idx] = (payload_len >> 8) & 0xFF;
    buffer[len_idx + 1] = payload_len & 0xFF;
    
    /* Calculate and append checksum */
    uint16_t checksum = UART_PROTO_CalculateChecksum(buffer, idx);
    buffer[idx++] = (checksum >> 8) & 0xFF;
    buffer[idx++] = checksum & 0xFF;
    
    return idx;
}
