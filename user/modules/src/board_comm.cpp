/**
 *******************************************************************************
 * @file      : board_comm.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "board_comm.h"

#include "string.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
BoardComm board_comm;
/* Private function prototypes -----------------------------------------------*/
static void BoardCommCallback();

void BoardComm::Init(CAN_HandleTypeDef *_phcan, uint16_t _id)
{
    CanInitConf conf;
    conf.hcan = _phcan;
    conf.rx_id = _id + 1;
    conf.tx_id = _id;
    p_instance_ = pCanRegister(&conf);
    p_instance_->pCanCallBack = BoardCommCallback;
}

void BoardComm::Send()
{
    uint8_t tx_buff[8];
    memcpy(tx_buff, &send_, 8);
    CanSend(p_instance_, tx_buff);
}

void BoardComm::Receive()
{
    memcpy(&rece_, p_instance_->rx_buff, 8);
}

static void BoardCommCallback()
{
    board_comm.Receive();
}