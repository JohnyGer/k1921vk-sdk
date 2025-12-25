/**
 * @file    can_hal.c
 * @brief   CAN Hardware Abstraction Layer Implementation
 * @author  Climate Control Team
 */

#include "can_hal.h"
#include "plib028.h"
#include "plib028_can.h"
#include <string.h>

/* Private variables */
static CAN_TypeDef* can_instances[2] = {NT_CAN0, NT_CAN1};

/**
 * @brief Initialize CAN interface
 * @param channel CAN channel to initialize
 * @param baudrate CAN bus baudrate in bps
 */
void CAN_HAL_Init(CAN_Channel_t channel, uint32_t baudrate) {
    CAN_TypeDef* can = can_instances[channel];
    
    /* Enable CAN clock */
    if (channel == CAN_CHANNEL_0) {
        RCU_CANClkCmd(RCU_CANClk_0, ENABLE);
    } else {
        RCU_CANClkCmd(RCU_CANClk_1, ENABLE);
    }
    
    /* Initialize CAN peripheral */
    CAN_Init_TypeDef can_init;
    CAN_StructInit(&can_init);
    
    /* Calculate timing for 500kbps @ 80MHz */
    can_init.SJW = CAN_SJW_1tq;
    can_init.Prescaler = 10;
    can_init.TimeSeg1 = CAN_TimeSeg1_13tq;
    can_init.TimeSeg2 = CAN_TimeSeg2_2tq;
    can_init.Mode = CAN_Mode_Normal;
    
    CAN_Init(can, &can_init);
    
    /* Configure filters to accept all messages initially */
    CAN_Filter_TypeDef filter;
    filter.FilterNumber = 0;
    filter.FilterMode = CAN_FilterMode_Mask;
    filter.FilterScale = CAN_FilterScale_32bit;
    filter.FilterID = 0x0000;
    filter.FilterMask = 0x0000;
    filter.FilterFIFO = CAN_FIFO_RX0;
    filter.FilterCmd = ENABLE;
    
    CAN_FilterConfig(can, &filter);
    
    /* Enable CAN */
    CAN_Cmd(can, ENABLE);
}

/**
 * @brief Send CAN message
 * @param channel CAN channel
 * @param id CAN message ID
 * @param data Pointer to data buffer
 * @param len Data length (0-8)
 * @return 0 on success, -1 on error
 */
int CAN_HAL_Send(CAN_Channel_t channel, uint32_t id, const uint8_t* data, uint8_t len) {
    CAN_TypeDef* can = can_instances[channel];
    
    if (len > 8 || data == NULL) {
        return -1;
    }
    
    /* Wait for empty transmit mailbox */
    uint32_t timeout = 10000;
    while (!CAN_IsTxMailboxEmpty(can, CAN_TxMailbox_0) && timeout--);
    
    if (timeout == 0) {
        return -1;
    }
    
    /* Prepare transmit message */
    CAN_TxMsg_TypeDef tx_msg;
    tx_msg.StdID = id;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = len;
    
    for (uint8_t i = 0; i < len; i++) {
        tx_msg.Data[i] = data[i];
    }
    
    /* Transmit message */
    return CAN_Transmit(can, &tx_msg, CAN_TxMailbox_0) == CAN_TxStatus_Ok ? 0 : -1;
}

/**
 * @brief Receive CAN message
 * @param channel CAN channel
 * @param id Pointer to store message ID
 * @param data Pointer to data buffer
 * @param len Pointer to store data length
 * @return 0 on success, -1 if no message available
 */
int CAN_HAL_Receive(CAN_Channel_t channel, uint32_t* id, uint8_t* data, uint8_t* len) {
    CAN_TypeDef* can = can_instances[channel];
    
    if (!CAN_IsMsgPending(can, CAN_FIFO_RX0)) {
        return -1;
    }
    
    CAN_RxMsg_TypeDef rx_msg;
    CAN_Receive(can, CAN_FIFO_RX0, &rx_msg);
    
    *id = rx_msg.StdID;
    *len = rx_msg.DLC;
    
    for (uint8_t i = 0; i < rx_msg.DLC; i++) {
        data[i] = rx_msg.Data[i];
    }
    
    return 0;
}

/**
 * @brief Configure CAN message filters
 * @param channel CAN channel
 * @param filter_ids Array of message IDs to accept
 * @param count Number of filter IDs
 */
void CAN_HAL_SetFilter(CAN_Channel_t channel, const uint32_t* filter_ids, uint8_t count) {
    CAN_TypeDef* can = can_instances[channel];
    
    /* Configure up to 14 filters (hardware limitation) */
    for (uint8_t i = 0; i < count && i < 14; i++) {
        CAN_Filter_TypeDef filter;
        filter.FilterNumber = i;
        filter.FilterMode = CAN_FilterMode_List;
        filter.FilterScale = CAN_FilterScale_32bit;
        filter.FilterID = filter_ids[i] << 21;
        filter.FilterMask = 0xFFFFFFFF;
        filter.FilterFIFO = CAN_FIFO_RX0;
        filter.FilterCmd = ENABLE;
        
        CAN_FilterConfig(can, &filter);
    }
}

/**
 * @brief Check if transmit mailbox is ready
 * @param channel CAN channel
 * @return true if ready, false otherwise
 */
bool CAN_HAL_IsTxReady(CAN_Channel_t channel) {
    CAN_TypeDef* can = can_instances[channel];
    return CAN_IsTxMailboxEmpty(can, CAN_TxMailbox_0);
}

/**
 * @brief Check if received message is available
 * @param channel CAN channel
 * @return true if message available, false otherwise
 */
bool CAN_HAL_IsRxAvailable(CAN_Channel_t channel) {
    CAN_TypeDef* can = can_instances[channel];
    return CAN_IsMsgPending(can, CAN_FIFO_RX0) > 0;
}
