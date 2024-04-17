/**
 *******************************************************************************
 * @file      : dt7.h
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
#ifndef __REMOTE_H_
#define __REMOTE_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

struct DR16DatePack_t {
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1;
    uint8_t s2;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t press_l;
    uint8_t press_r;
    uint16_t key;
};

class Remote_t
{
   public:
    UartInstance *premote_instance;
    void SbusToRc(uint8_t *_pdata);

    int GetCh0()
    {
        return Pack_.ch0;
    }

    int GetCh1()
    {
        return Pack_.ch1;
    }

    int GetCh2()
    {
        return Pack_.ch2;
    }

    int GetCh3()
    {
        return Pack_.ch3;
    }

    int GetS1()
    {
        return Pack_.s1;
    }

    int GetS2()
    {
        return Pack_.s2;
    }

    int GetMouseX()
    {
        return Pack_.mouse_x;
    }

    int GetMouseY()
    {
        return Pack_.mouse_y;
    }

   private:
    DR16DatePack_t Pack_;
};
/* Exported variables --------------------------------------------------------*/
extern Remote_t remote;
/* Exported function prototypes ----------------------------------------------*/
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

void RemoteControlInit(UART_HandleTypeDef *_phuart);

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_H_ */
