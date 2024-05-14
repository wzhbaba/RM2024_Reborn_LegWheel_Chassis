/**
 *******************************************************************************
 * @file      : Super_Cap.cpp
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
#include "cap.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
PM01_t cap;
/* Private function prototypes -----------------------------------------------*/

static void CapStateCallback()
{
    cap.Update();
}

void PM01_t::Init(CAN_HandleTypeDef *_p_hcan, uint16_t _id)
{
    CanInitConf conf;
    conf.hcan = _p_hcan;
    conf.tx_id = _id;
    conf.rx_id = _id;
    conf.pCanCallBack = CapStateCallback;
    p_instance_ = pCanRegister(&conf);
}

void PM01_t::Update()
{
    response_flg_ = (access_id_ == p_instance_->rx_id);

    if (p_instance_->rx_rtr == CAN_RTR_REMOTE)
        return;

    switch (p_instance_->rx_id) {
        case 0x600:
            data.ccr = (uint16_t)(p_instance_->rx_buff[0] << 8 | p_instance_->rx_buff[1]);
            break;
        case 0x601:
            data.p_set = (uint16_t)(p_instance_->rx_buff[0] << 8 | p_instance_->rx_buff[1]);
            break;
        case 0x602:
            data.v_set = (uint16_t)(p_instance_->rx_buff[0] << 8 | p_instance_->rx_buff[1]);
            break;
        case 0x603:
            data.i_set = (uint16_t)(p_instance_->rx_buff[0] << 8 | p_instance_->rx_buff[1]);
            break;
        case 0x610:
            data.sta_code.all = (uint16_t)(p_instance_->rx_buff[0] << 8 | p_instance_->rx_buff[1]);
            data.err_code = (uint16_t)(p_instance_->rx_buff[2] << 8 | p_instance_->rx_buff[3]);
            break;
        case 0x611:
            data.p_in = (uint16_t)p_instance_->rx_buff[0] << 8 | p_instance_->rx_buff[1];
            data.v_in = (uint16_t)p_instance_->rx_buff[2] << 8 | p_instance_->rx_buff[3];
            data.i_in = (uint16_t)p_instance_->rx_buff[4] << 8 | p_instance_->rx_buff[5];
            break;
        case 0x612:
            data.p_out = (uint16_t)p_instance_->rx_buff[0] << 8 | p_instance_->rx_buff[1];
            data.v_out = (uint16_t)p_instance_->rx_buff[2] << 8 | p_instance_->rx_buff[3];
            data.i_out = (uint16_t)p_instance_->rx_buff[4] << 8 | p_instance_->rx_buff[5];
            break;
        case 0x613:
            data.temp = (uint16_t)p_instance_->rx_buff[0] << 8 | p_instance_->rx_buff[1];
            data.total_time = (uint16_t)p_instance_->rx_buff[2] << 8 | p_instance_->rx_buff[3];
            data.run_time = (uint16_t)p_instance_->rx_buff[4] << 8 | p_instance_->rx_buff[5];
            break;
    }
}

void PM01_t::AccessPoll()
{
    static uint8_t state = 0x00;    /* 状态    */
    static uint16_t can_id = 0x600; /* 标识符  */
    static uint32_t timeout = 0;    /* 超时    */
    static uint16_t i = 0;

    switch (state) {
        case 0x00: /* 发送数据 */

            timeout = 0; /* 清除超时定时器 */

            state = 0x01; /* 切换到等待状态 */

            switch (i) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                    access_id_ = SetRxID();
                    SendRemote();
                    break;
                case 8:
                    access_id_ = SetRxID(); /* 记录本次访问的标识符 */
                    SendData(cmd_set_);
                    break;
                case 9:
                    access_id_ = SetRxID(); /* 记录本次访问的标识符 */
                    SendData(power_set_);
                    break;
                case 10:
                    access_id_ = SetRxID(); /* 记录本次访问的标识符 */
                    SendData(vout_set_);
                    break;
                case 11:
                    access_id_ = SetRxID(); /* 记录本次访问的标识符 */
                    SendData(iout_set_);
                    break;
            }

            break;
        case 0x01: /* 等待响应 */

            if (response_flg_ == 0x01) {
                response_flg_ = 0x00; /* 复位应答标志位 */

                /* 访问成功，继续访问下一个标识符 */

                i = (i + 1) % 12;

                switch (i) {
                    case 0:
                        can_id = SetTxID(0x600);
                        break;
                    case 1:
                        can_id = SetTxID(0x601);
                        break;
                    case 2:
                        can_id = SetTxID(0x602);
                        break;
                    case 3:
                        can_id = SetTxID(0x603);
                        break;
                    case 4:
                        can_id = SetTxID(0x610);
                        break;
                    case 5:
                        can_id = SetTxID(0x611);
                        break;
                    case 6:
                        can_id = SetTxID(0x612);
                        break;
                    case 7:
                        can_id = SetTxID(0x613);
                        break;
                    case 8:
                        can_id = SetTxID(0x600);
                        break; /* 写 */
                    case 9:
                        can_id = SetTxID(0x601);
                        break; /* 写 */
                    case 10:
                        can_id = SetTxID(0x602);
                        break; /* 写 */
                    case 11:
                        can_id = SetTxID(0x603);
                        break; /* 写 */
                }

                state = 0x00; /* 启动下一轮访问 */

            } else {
                timeout++;
            }

            /* 超时1s，重新访问  */
            if (timeout > 100) {
                state = 0x00;
            }

            break;
        default:
            state = 0x00;
    }
}

void PM01_t::SendRemote()
{
    CanSetDlcAndRtr(p_instance_, 0x00, CAN_RTR_REMOTE);
    CanSend(p_instance_, 0);
    CanSetDlcAndRtr(p_instance_, 0x04, CAN_RTR_DATA);
}

void PM01_t::SendData(uint16_t _cmd)
{
    uint8_t tx[8];
    tx[0] = _cmd >> 8;
    tx[1] = _cmd;
    tx[2] = 0x00;
    tx[3] = 0x01;
    CanSend(p_instance_, tx);
}

uint16_t PM01_t::SetRxID()
{
    p_instance_->rx_id = p_instance_->tx_conf.StdId;
    return p_instance_->rx_id;
}

uint16_t PM01_t::SetTxID(uint16_t _id)
{
    p_instance_->tx_conf.StdId = _id;
    return p_instance_->tx_conf.StdId;
}

void PM01_t::SetCapState()
{
    if (data.v_out < 1500) {
        cap_state_ = 0;
    } else if (data.v_out > 1800) {
        cap_state_ = 1;
    }
}