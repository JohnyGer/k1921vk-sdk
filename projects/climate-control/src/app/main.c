/**
 * @file    main.c
 * @brief   Climate Control System Main Application
 * @author  Climate Control Team
 * @details Simple, reliable implementation - AK-47 style!
 */

#include "K1921VK028.h"
#include "plib028.h"
#include "common_types.h"
#include "can_hal.h"
#include "uart_hal.h"
#include "timer_hal.h"
#include "gpio_hal.h"
#include "can_protocol.h"
#include "uart_protocol.h"
#include "sensor_manager.h"

/* System state */
static SystemState_t system_state = {0};
static volatile bool timer_tick = false;
static uint8_t uart_rx_buffer[256];
static uint8_t uart_tx_buffer[512];

/**
 * @brief Initialize system
 */
static void System_Init(void) {
    /* Initialize clock to 80MHz */
    RCU_SysClk_TypeDef rcu_clk;
    RCU_SysClkStructInit(&rcu_clk);
    rcu_clk.PLLRef = RCU_PLLRef_HSE;
    rcu_clk.PLLMul = RCU_PLLMul_10;  /* 8MHz * 10 = 80MHz */
    rcu_clk.SysClkSource = RCU_SysClkSource_PLL;
    RCU_SysClkConfig(&rcu_clk);
    
    /* Initialize HAL */
    GPIO_HAL_Init();
    CAN_HAL_Init(CAN_CHANNEL_0, 500000);
    CAN_HAL_Init(CAN_CHANNEL_1, 500000);
    UART_HAL_Init(UART_CHANNEL_0, 115200);
    UART_HAL_Init(UART_CHANNEL_1, 115200);
    TIMER_HAL_Init(500);  /* 500ms period */
    
    /* Initialize protocols */
    CAN_PROTO_Init();
    UART_PROTO_Init();
    
    /* Initialize managers */
    SENSOR_MGR_Init();
    
    /* Set defaults */
    system_state.target_temp = 22.0f;
    system_state.mode = MODE_AUTO;
    system_state.error_flags = ERR_NONE;
    
    /* Start UART RX */
    UART_HAL_ReceiveDMA(UART_CHANNEL_0, uart_rx_buffer, sizeof(uart_rx_buffer));
    
    /* Enable interrupts */
    NVIC_EnableIRQ(CAN0_IRQn);
    NVIC_EnableIRQ(CAN1_IRQn);
    
    /* Start timer */
    TIMER_HAL_Start();
    
    /* Indicate system ready */
    GPIO_HAL_SetPin(GPIO_PIN_LED_STATUS, true);
}

/**
 * @brief Process CAN messages
 */
static void Process_CAN_Messages(void) {
    /* Process CAN0 messages */
    if (CAN_HAL_IsRxAvailable(CAN_CHANNEL_0)) {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
        
        if (CAN_HAL_Receive(CAN_CHANNEL_0, &id, data, &len) == 0) {
            SensorData_t sensor;
            if (CAN_PROTO_ParseSensorMsg(id, data, len, &sensor)) {
                SENSOR_MGR_Update(sensor.sensor_id, sensor.temperature);
                GPIO_HAL_TogglePin(GPIO_PIN_LED_CAN0);
            }
        }
    }
    
    /* Process CAN1 messages */
    if (CAN_HAL_IsRxAvailable(CAN_CHANNEL_1)) {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
        
        if (CAN_HAL_Receive(CAN_CHANNEL_1, &id, data, &len) == 0) {
            SensorData_t sensor;
            if (CAN_PROTO_ParseSensorMsg(id, data, len, &sensor)) {
                SENSOR_MGR_Update(sensor.sensor_id, sensor.temperature);
                GPIO_HAL_TogglePin(GPIO_PIN_LED_CAN1);
            }
        }
    }
}

/**
 * @brief Simple climate control logic
 */
static void Climate_Control_Execute(void) {
    /* Get room temperatures */
    float temps[9];
    uint8_t count = 0;
    SENSOR_MGR_GetAllRoomTemperatures(temps, &count);
    
    if (count == 0) {
        return;  /* No sensors online */
    }
    
    /* Calculate average */
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += temps[i];
    }
    float avg_temp = sum / count;
    system_state.average_room_temp = avg_temp;
    
    /* Simple bang-bang control */
    float error = system_state.target_temp - avg_temp;
    
    if (error > 1.0f) {
        /* Too cold - turn on heaters */
        for (uint8_t i = 0; i < 6; i++) {
            system_state.heater_state[i] = true;
            system_state.heater_power[i] = 80;  /* 80% power */
        }
        for (uint8_t i = 0; i < 4; i++) {
            system_state.ac_state[i] = false;
            system_state.ac_power[i] = 0;
        }
    } else if (error < -1.0f) {
        /* Too hot - turn on AC */
        for (uint8_t i = 0; i < 6; i++) {
            system_state.heater_state[i] = false;
            system_state.heater_power[i] = 0;
        }
        for (uint8_t i = 0; i < 4; i++) {
            system_state.ac_state[i] = true;
            system_state.ac_power[i] = 80;
        }
    } else {
        /* Temperature OK - turn off all */
        for (uint8_t i = 0; i < 6; i++) {
            system_state.heater_state[i] = false;
            system_state.heater_power[i] = 0;
        }
        for (uint8_t i = 0; i < 4; i++) {
            system_state.ac_state[i] = false;
            system_state.ac_power[i] = 0;
        }
    }
    
    /* Send actuator commands via CAN - simplified for demonstration */
    /* In full implementation, iterate and send commands via CAN_HAL_Send */
}

/**
 * @brief Timer callback for periodic status reporting
 */
void Timer_Callback(void) {
    timer_tick = true;
}

/**
 * @brief Main application entry point
 */
int main(void) {
    /* Initialize system */
    System_Init();
    
    /* Register timer callback */
    TIMER_HAL_RegisterCallback(Timer_Callback);
    
    /* Main loop - simple and reliable! */
    while (1) {
        /* Process CAN messages */
        Process_CAN_Messages();
        
        /* Check sensor timeouts */
        SENSOR_MGR_CheckTimeouts();
        
        /* Execute climate control in AUTO mode */
        if (system_state.mode == MODE_AUTO) {
            Climate_Control_Execute();
        }
        
        /* Handle periodic status reporting */
        if (timer_tick) {
            timer_tick = false;
            system_state.system_uptime += 500;  /* 500ms tick */
            
            /* Build and send status report */
            uint16_t report_len = UART_PROTO_BuildStatusReport(&system_state, uart_tx_buffer);
            if (report_len > 0) {
                UART_HAL_SendDMA(UART_CHANNEL_1, uart_tx_buffer, report_len);
            }
        }
        
        /* Optional: Sleep until next event */
        /* __WFI(); */
    }
    
    return 0;
}
