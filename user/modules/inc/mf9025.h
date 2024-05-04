/**
 *******************************************************************************
 * @file      : mf9025.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MF9025_H_
#define __MF9025_H_

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

#include "pid.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Mf9025 {
 public:
  void Init(CAN_HandleTypeDef* _phcan, uint16_t _id);
  void Update();
  void SetTor(float _tor);
  float GetSpeed();
  void Send();
  CanInstance* p_motor_instance_;

 private:
  Pid tor_;
  int8_t temp_;
  int16_t iq_, speed_, encode_, iq_ctrl_;
  float speed_real_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* __MF9025_H_ */
