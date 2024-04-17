/**
 *******************************************************************************
 * @file      : dt7.cpp
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
#include "remote.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Remote_t remote;

const int kremote_offest = 1024;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Converts Sbus data to RC data.
 *
 * This function takes a pointer to Sbus data and converts it to RC data.
 *
 * @param _pdata Pointer to the Sbus data.
 */
void Remote_t::SbusToRc(uint8_t *_pdata)
{
    Pack_.ch0 = ((_pdata[0] | _pdata[1] << 8) & 0x07FF);
    Pack_.ch1 = ((_pdata[1] >> 3 | _pdata[2] << 5) & 0x07FF);
    Pack_.ch2 = ((_pdata[2] >> 6 | _pdata[3] << 2 | _pdata[4] << 10) & 0x07FF);
    Pack_.ch3 = ((_pdata[4] >> 1 | _pdata[5] << 7) & 0x07FF);
    Pack_.s1 = ((_pdata[5] >> 4) & 0x000C) >> 2;
    Pack_.s2 = ((_pdata[5] >> 4) & 0x0003);
    Pack_.mouse_x = _pdata[6] | _pdata[7] << 8;
    Pack_.mouse_y = _pdata[8] | _pdata[9] << 8;
    Pack_.mouse_z = _pdata[10] | _pdata[11] << 8;
    Pack_.press_l = _pdata[12];
    Pack_.press_r = _pdata[13];
    Pack_.key = _pdata[14] | _pdata[15] << 8;
    Pack_.ch0 -= kremote_offest;
    Pack_.ch1 -= kremote_offest;
    Pack_.ch2 -= kremote_offest;
    Pack_.ch3 -= kremote_offest;
}

/**
 * @brief This function is the callback for remote control events.
 *
 * It is called when a remote control event occurs.
 *
 * @return void
 */
static void RemoteControlCallBack()
{
    remote.SbusToRc(remote.premote_instance->rx_buffer);
}

/**
 * @brief Initializes the remote control module.
 *
 * This function initializes the remote control module by configuring the UART peripheral.
 *
 * @param _phuart Pointer to the UART handle structure.
 */
void RemoteControlInit(UART_HandleTypeDef *_phuart)
{
    UartInitConfig conf;
    conf.huart = _phuart;
    conf.rx_buffer_size = 18;
    conf.callback_function = RemoteControlCallBack;
    remote.premote_instance = pUartRegister(&conf);
    return;
}
