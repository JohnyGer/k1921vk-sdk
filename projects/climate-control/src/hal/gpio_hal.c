/**
 * @file    gpio_hal.c
 * @brief   GPIO Hardware Abstraction Layer Implementation
 * @author  Climate Control Team
 */

#include "gpio_hal.h"
#include "plib028.h"
#include "plib028_gpio.h"
#include "plib028_rcu.h"

/* Private variables - LED pin mapping */
static const struct {
    GPIO_TypeDef* port;
    uint32_t pin;
} gpio_pins[] = {
    {NT_GPIOA, GPIO_Pin_0},  /* STATUS LED */
    {NT_GPIOA, GPIO_Pin_1},  /* ERROR LED */
    {NT_GPIOA, GPIO_Pin_2},  /* CAN0 LED */
    {NT_GPIOA, GPIO_Pin_3}   /* CAN1 LED */
};

/**
 * @brief Initialize GPIO pins
 */
void GPIO_HAL_Init(void) {
    /* Enable GPIO clock */
    RCU_GPIOClkCmd(RCU_GPIOClk_A, ENABLE);
    
    /* Configure LED pins as outputs */
    GPIO_Init_TypeDef gpio_init;
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Dir = GPIO_Dir_Out;
    gpio_init.GPIO_Out = GPIO_Out_En;
    gpio_init.GPIO_AltFunc = GPIO_AltFunc_1;
    gpio_init.GPIO_PullDown = GPIO_PullDown_Dis;
    gpio_init.GPIO_PullUp = GPIO_PullUp_Dis;
    gpio_init.GPIO_DriveMode = GPIO_DriveMode_HighFast;
    
    for (uint8_t i = 0; i < 4; i++) {
        gpio_init.GPIO_Pin = gpio_pins[i].pin;
        GPIO_Init(gpio_pins[i].port, &gpio_init);
        GPIO_BitsReset(gpio_pins[i].port, gpio_pins[i].pin);
    }
}

/**
 * @brief Set GPIO pin state
 * @param pin GPIO pin identifier
 * @param state true = HIGH, false = LOW
 */
void GPIO_HAL_SetPin(GPIO_Pin_t pin, bool state) {
    if (pin < 4) {
        if (state) {
            GPIO_BitsSet(gpio_pins[pin].port, gpio_pins[pin].pin);
        } else {
            GPIO_BitsReset(gpio_pins[pin].port, gpio_pins[pin].pin);
        }
    }
}

/**
 * @brief Toggle GPIO pin state
 * @param pin GPIO pin identifier
 */
void GPIO_HAL_TogglePin(GPIO_Pin_t pin) {
    if (pin < 4) {
        GPIO_BitsToggle(gpio_pins[pin].port, gpio_pins[pin].pin);
    }
}
