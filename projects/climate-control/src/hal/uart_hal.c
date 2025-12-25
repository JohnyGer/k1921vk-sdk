/**
 * @file    uart_hal.c
 * @brief   UART Hardware Abstraction Layer Implementation
 * @author  Climate Control Team
 */

#include "uart_hal.h"
#include "plib028.h"
#include "plib028_uart.h"
#include "plib028_dma.h"
#include "plib028_rcu.h"

/* Private variables */
static UART_TypeDef* uart_instances[2] = {NT_UART0, NT_UART1};
static volatile bool tx_complete[2] = {true, true};
static volatile uint16_t rx_count[2] = {0, 0};

/**
 * @brief Initialize UART interface with DMA support
 * @param channel UART channel to initialize
 * @param baudrate UART baudrate
 */
void UART_HAL_Init(UART_Channel_t channel, uint32_t baudrate) {
    UART_TypeDef* uart = uart_instances[channel];
    
    /* Enable UART and DMA clocks */
    if (channel == UART_CHANNEL_0) {
        RCU_UARTClkCmd(RCU_UARTClk_0, ENABLE);
    } else {
        RCU_UARTClkCmd(RCU_UARTClk_1, ENABLE);
    }
    RCU_DMAClkCmd(ENABLE);
    
    /* Initialize UART */
    UART_Init_TypeDef uart_init;
    UART_StructInit(&uart_init);
    uart_init.BaudRate = baudrate;
    uart_init.WordLength = UART_WordLen_8bit;
    uart_init.StopBits = UART_StopBits_1;
    uart_init.Parity = UART_Parity_Disable;
    uart_init.FIFOSize = UART_FIFOSize_8byte;
    UART_Init(uart, &uart_init);
    
    /* Enable UART DMA */
    UART_DMAConfig(uart, UART_DMAReq_Tx | UART_DMAReq_Rx, ENABLE);
    
    /* Enable UART */
    UART_Cmd(uart, ENABLE);
}

/**
 * @brief Send data via UART using DMA
 * @param channel UART channel
 * @param data Pointer to data buffer
 * @param len Data length
 * @return 0 on success, -1 on error
 */
int UART_HAL_SendDMA(UART_Channel_t channel, const uint8_t* data, uint16_t len) {
    if (data == NULL || len == 0 || !tx_complete[channel]) {
        return -1;
    }
    
    tx_complete[channel] = false;
    
    /* Configure DMA channel */
    DMA_Channel_TypeDef dma_ch = (channel == UART_CHANNEL_0) ? 
        DMA_CHANNEL_UART0_TX : DMA_CHANNEL_UART1_TX;
    
    DMA_ChannelInit_TypeDef dma_init;
    DMA_ChannelStructInit(&dma_init);
    dma_init.DMA_Dir = DMA_Dir_MemToPeriph;
    dma_init.DMA_Mode = DMA_Mode_Normal;
    dma_init.DMA_MemSize = DMA_MemSize_8bit;
    dma_init.DMA_PeriphSize = DMA_PeriphSize_8bit;
    dma_init.DMA_MemInc = ENABLE;
    dma_init.DMA_PeriphInc = DISABLE;
    dma_init.DMA_PeriphAddr = (uint32_t)&uart_instances[channel]->DR;
    dma_init.DMA_MemAddr = (uint32_t)data;
    dma_init.DMA_ChainMode = DISABLE;
    
    DMA_ChannelInit(NT_DMA, dma_ch, &dma_init);
    DMA_ChannelTransferSizeConfig(NT_DMA, dma_ch, len);
    
    /* Enable DMA channel */
    DMA_ChannelCmd(NT_DMA, dma_ch, ENABLE);
    
    return 0;
}

/**
 * @brief Start DMA reception on UART
 * @param channel UART channel
 * @param buffer Pointer to receive buffer
 * @param len Maximum bytes to receive
 * @return 0 on success, -1 on error
 */
int UART_HAL_ReceiveDMA(UART_Channel_t channel, uint8_t* buffer, uint16_t len) {
    if (buffer == NULL || len == 0) {
        return -1;
    }
    
    rx_count[channel] = 0;
    
    /* Configure DMA channel */
    DMA_Channel_TypeDef dma_ch = (channel == UART_CHANNEL_0) ? 
        DMA_CHANNEL_UART0_RX : DMA_CHANNEL_UART1_RX;
    
    DMA_ChannelInit_TypeDef dma_init;
    DMA_ChannelStructInit(&dma_init);
    dma_init.DMA_Dir = DMA_Dir_PeriphToMem;
    dma_init.DMA_Mode = DMA_Mode_Circular;
    dma_init.DMA_MemSize = DMA_MemSize_8bit;
    dma_init.DMA_PeriphSize = DMA_PeriphSize_8bit;
    dma_init.DMA_MemInc = ENABLE;
    dma_init.DMA_PeriphInc = DISABLE;
    dma_init.DMA_PeriphAddr = (uint32_t)&uart_instances[channel]->DR;
    dma_init.DMA_MemAddr = (uint32_t)buffer;
    dma_init.DMA_ChainMode = DISABLE;
    
    DMA_ChannelInit(NT_DMA, dma_ch, &dma_init);
    DMA_ChannelTransferSizeConfig(NT_DMA, dma_ch, len);
    
    /* Enable DMA channel */
    DMA_ChannelCmd(NT_DMA, dma_ch, ENABLE);
    
    return 0;
}

/**
 * @brief Check if UART transmission is complete
 * @param channel UART channel
 * @return true if complete, false otherwise
 */
bool UART_HAL_IsTxComplete(UART_Channel_t channel) {
    return tx_complete[channel];
}

/**
 * @brief Get number of bytes received
 * @param channel UART channel
 * @return Number of bytes received
 */
uint16_t UART_HAL_GetRxCount(UART_Channel_t channel) {
    return rx_count[channel];
}

/**
 * @brief DMA TX complete callback (called from ISR)
 * @param channel UART channel
 */
void UART_HAL_TxCompleteCallback(UART_Channel_t channel) {
    tx_complete[channel] = true;
}

/**
 * @brief DMA RX complete callback (called from ISR)
 * @param channel UART channel
 * @param count Number of bytes received
 */
void UART_HAL_RxCompleteCallback(UART_Channel_t channel, uint16_t count) {
    rx_count[channel] = count;
}
