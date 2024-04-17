/**
 *******************************************************************************
 * @file      : bsp_can.c
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "memory.h"
#include "stdlib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int idx;
static CanInstance *pcan_instance[16] = {NULL};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Registers a CAN instance with the specified initialization configuration.
 *
 * This function is used to register a CAN instance with the provided initialization configuration.
 *
 * @param _pconf Pointer to the CanInitConf structure containing the initialization configuration.
 * @return Pointer to the CanInstance structure representing the registered CAN instance.
 */
CanInstance *pCanRegister(CanInitConf *_pconf)
{
    if (idx >= 16) {
        while (1) {
        }
    }

    for (uint8_t i = 0; i < 16; i++) {
        if (pcan_instance[i]->rx_id == _pconf->rx_id && pcan_instance[i]->hcan == _pconf->hcan) {
            while (1) {
            };
        }
    }

    CanInstance *p_instance = (CanInstance *)malloc(sizeof(CanInstance));
    memset(p_instance, 0, sizeof(CanInstance));

    p_instance->tx_conf.StdId = _pconf->tx_id;
    p_instance->tx_conf.IDE = CAN_ID_STD;
    p_instance->tx_conf.RTR = CAN_RTR_DATA;
    p_instance->tx_conf.DLC = 0x08;

    p_instance->hcan = _pconf->hcan;
    p_instance->rx_id = _pconf->rx_id;
    p_instance->pCanCallBack = _pconf->pCanCallBack;

    pcan_instance[idx++] = p_instance;

    return p_instance;
}

/**
 * @brief Sends a CAN message.
 *
 * This function sends a CAN message using the specified CAN instance and transmit buffer.
 *
 * @param _pinstance Pointer to the CAN instance.
 * @param _ptx_buff Pointer to the transmit buffer.
 */
void CanSend(CanInstance *_pinstance, uint8_t *_ptx_buff)
{
    if (HAL_CAN_AddTxMessage(_pinstance->hcan, &_pinstance->tx_conf, _ptx_buff, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_pinstance->hcan, &_pinstance->tx_conf, _ptx_buff, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK) {
            while (1) {
            }
        }
    }
}

/**
 * @brief 设置CAN数据长度和RTR标志位
 *
 * @param _pinstance CAN实例指针
 * @param _length 数据长度
 * @param _rtr RTR标志位
 */
void CanSetDlcAndRtr(CanInstance *_pinstance, uint8_t _length, uint8_t _rtr)
{
    _pinstance->tx_conf.DLC = _length;
    _pinstance->tx_conf.RTR = _rtr;
}

/**
 * @brief Callback function for CAN receive interrupt.
 *
 * This function is called when a CAN receive interrupt occurs.
 *
 * @param _phcan Pointer to the CAN handle structure.
 * @param _Fifo The FIFO number associated with the interrupt.
 */
static void CanRxCallBack(CAN_HandleTypeDef *_phcan, uint32_t _Fifo)
{
    static CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_phcan, _Fifo)) {
        HAL_CAN_GetRxMessage(_phcan, _Fifo, &rx_header, rx_buff);
        for (uint8_t i = 0; i < 16; i++) {
            if (rx_header.StdId == pcan_instance[i]->rx_id && _phcan == pcan_instance[i]->hcan) {
                if (pcan_instance[i]->pCanCallBack != NULL) {
                    pcan_instance[i]->rx_len = rx_header.DLC;
                    memcpy(pcan_instance[i]->rx_buff, rx_buff, rx_header.DLC);
                    pcan_instance[i]->pCanCallBack();
                }
            }
        }
    }
}

/**
 * @brief Callback function called when a message is pending in Rx FIFO 0.
 *
 * @param hcan Pointer to a CAN_HandleTypeDef structure that contains
 *              the configuration information for the specified CAN.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CanRxCallBack(hcan, CAN_RX_FIFO0);
}