/**
 *******************************************************************************
 * @file      : bsp_uart.h
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
#ifndef __BSP_UART_H_
#define __BSP_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef void (*UART_CallBack)();

typedef struct
{
    UART_HandleTypeDef *huart;
    uint16_t rx_buffer_size;
    uint8_t rx_buffer[256];
    UART_CallBack callback_function;
} UartInstance;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint16_t rx_buffer_size;
    UART_CallBack callback_function;
} UartInitConfig;

typedef enum {
    UART_TRAMSMIT_NONE = 0,
    UART_TRAMSMIT_BLOCKING,
    UART_TRANSMIT_IT,
    UART_TRAMSMIT_DMA,
} UART_TRANSMIT_MODE;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

UartInstance *pUartRegister(UartInitConfig *_pconfig);
void UartSendData(UART_HandleTypeDef *_phuart, uint8_t *_send_buf, uint16_t _send_size, UART_TRANSMIT_MODE _mode);
void UartInit(UartInstance *_pinstance);

#ifdef __cplusplus
}
#endif

#endif /* __FILE_H_ */
