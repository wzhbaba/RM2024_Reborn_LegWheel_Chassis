/**
 *******************************************************************************
 * @file      : dji_motor.cpp
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
#include "dji_motor.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DjiMotor dji_motor;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void DjiMotor::Update()
{
    encode_ = pdji_motor_instance->rx_buff[0] << 8 | pdji_motor_instance->rx_buff[1];
    spd_ = pdji_motor_instance->rx_buff[2] << 8 | pdji_motor_instance->rx_buff[3];
    tor_ = pdji_motor_instance->rx_buff[4] << 8 | pdji_motor_instance->rx_buff[5];
    temp_ = pdji_motor_instance->rx_buff[6];

    if (init_ == 1) {
        if (encode_ - last_encode_ > 4096) {
            round_cnt_--;
        } else if (encode_ - last_encode_ < -4096) {
            round_cnt_++;
        }
    } else {
        encode_offest_ = encode_;
        init_ = 1;
    }

    last_encode_ = encode_;
    int32_t encode_now = encode_ + round_cnt_ * 8192 - encode_offest_;
    ang_real_ = (encode_now / 8192.0f) * 360.0f;
}

void DjiMotorCallBack()
{
    dji_motor.Update();
}

void DjiMotor::Init(uint32_t _idx, CAN_HandleTypeDef *_phcan, uint8_t _init)
{
    CanInitConf conf;
    init_ = _init;
    conf.hcan = _phcan;
    conf.rx_id = _idx;
    pdji_motor_instance = pCanRegister(&conf);
}

void DjiMotorSend(CAN_HandleTypeDef *_phcan, uint32_t _idx, int16_t _data1, int16_t _data2, int16_t _data3, int16_t _data4)
{
    CAN_TxHeaderTypeDef tx_conf;
    uint8_t tx_data[8];
    tx_conf.StdId = _idx;
    tx_conf.IDE = CAN_ID_STD;
    tx_conf.RTR = CAN_RTR_DATA;
    tx_conf.DLC = 0x08;
    tx_data[0] = _data1 >> 8;
    tx_data[1] = _data1;
    tx_data[2] = _data2 >> 8;
    tx_data[3] = _data2;
    tx_data[4] = _data3 >> 8;
    tx_data[5] = _data3;
    tx_data[6] = _data4 >> 8;
    tx_data[7] = _data4;
    if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK) {
            while (1) {
            }
        }
    }
}