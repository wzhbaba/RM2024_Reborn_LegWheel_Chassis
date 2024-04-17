/**
 *******************************************************************************
 * @file      : mf9025.cpp
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
#include "mf9025.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void Mf9025::Init(CAN_HandleTypeDef *_phcan, uint16_t _id)
{
    CanInitConf conf;
    conf.hcan = _phcan;
    conf.rx_id = _id;
    conf.tx_id = _id;
    p_motor_instance_ = pCanRegister(&conf);
}

void Mf9025::Update()
{
    if (p_motor_instance_->rx_buff[0] == 0xA1) {
        temp_ = p_motor_instance_->rx_buff[1];
        iq_ = p_motor_instance_->rx_buff[3] << 8 | p_motor_instance_->rx_buff[2];
        speed_ = p_motor_instance_->rx_buff[5] << 8 | p_motor_instance_->rx_buff[4];
        encode_ = p_motor_instance_->rx_buff[7] << 8 | p_motor_instance_->rx_buff[6];
    }
}

void Mf9025::SetTor(float _tor)
{
    iq_ctrl_ = (_tor / 0.32f) * 62.5f;
    Math::AbsLimit(iq_ctrl_, 2000);
}

void Mf9025::Send()
{
    uint8_t tx_buff[8];
    tx_buff[0] = 0xA1;
    tx_buff[1] = 0x00;
    tx_buff[2] = 0x00;
    tx_buff[3] = 0x00;
    tx_buff[4] = iq_ctrl_;
    tx_buff[5] = iq_ctrl_ >> 8;
    tx_buff[6] = 0x00;
    tx_buff[7] = 0x00;
    CanSend(p_motor_instance_, tx_buff);
}

float Mf9025::GetSpeed()
{
    speed_real_ = 0.15f * speed_real_ + 0.85f * speed_ * 3.1415926f / 180.0f;
    return speed_real_;
}