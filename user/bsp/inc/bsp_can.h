/**
 *******************************************************************************
 * @file      : bsp_can.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_CAN_H_
#define __BSP_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "can.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#pragma pack(1)
typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef tx_conf;
    uint32_t rx_id;
    uint8_t rx_len;
    uint8_t rx_buff[8];
    uint8_t rx_rtr;
    void (*pCanCallBack)();
} CanInstance;
#pragma pack()

typedef struct {
    CAN_HandleTypeDef *hcan;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*pCanCallBack)();
} CanInitConf;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
CanInstance *pCanRegister(CanInitConf *_pconf);
void CanSend(CanInstance *_pinstance, uint8_t *_ptx_buff);
void CanSetDlcAndRtr(CanInstance *_pinstance, uint8_t _length, uint8_t _rtr);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H_ */
