/**
 * @file    common_types.h
 * @brief   Common type definitions for climate control system
 * @author  Climate Control Team
 */

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* CAN channel enumeration */
typedef enum {
    CAN_CHANNEL_0 = 0,
    CAN_CHANNEL_1 = 1
} CAN_Channel_t;

/* UART channel enumeration */
typedef enum {
    UART_CHANNEL_0 = 0,
    UART_CHANNEL_1 = 1
} UART_Channel_t;

/* DMA channel enumeration */
typedef enum {
    DMA_CHANNEL_CAN0_RX = 0,
    DMA_CHANNEL_CAN0_TX = 1,
    DMA_CHANNEL_CAN1_RX = 2,
    DMA_CHANNEL_CAN1_TX = 3,
    DMA_CHANNEL_UART0_RX = 4,
    DMA_CHANNEL_UART0_TX = 5,
    DMA_CHANNEL_UART1_RX = 6,
    DMA_CHANNEL_UART1_TX = 7
} DMA_Channel_t;

/* GPIO pin enumeration for status LEDs */
typedef enum {
    GPIO_PIN_LED_STATUS = 0,
    GPIO_PIN_LED_ERROR = 1,
    GPIO_PIN_LED_CAN0 = 2,
    GPIO_PIN_LED_CAN1 = 3
} GPIO_Pin_t;

/* Device status enumeration */
typedef enum {
    STATUS_OK = 0,
    STATUS_ERROR = 1,
    STATUS_RUNNING = 2,
    STATUS_WAITING = 3
} DeviceStatus_t;

/* Control mode enumeration */
typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL = 1
} ControlMode_t;

/* Error codes */
typedef enum {
    ERR_NONE = 0,
    ERR_SENSOR_TIMEOUT = (1 << 0),
    ERR_ACTUATOR_FAULT = (1 << 1),
    ERR_CAN_BUS_OFF = (1 << 2),
    ERR_UART_OVERFLOW = (1 << 3),
    ERR_INVALID_COMMAND = (1 << 4),
    ERR_TEMP_OUT_OF_RANGE = (1 << 5)
} ErrorCode_t;

#ifdef __cplusplus
}
#endif

#endif /* COMMON_TYPES_H */
