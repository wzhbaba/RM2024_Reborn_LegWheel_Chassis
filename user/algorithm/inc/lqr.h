/**
 *******************************************************************************
 * @file      : lqr.h
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
#ifndef __LQR_H_
#define __LQR_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdlib.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Lqr {
 public:
  void Calc();
  void SetData(float _dist, float _speed, float _phi, float _w_phi,
               float _theta, float _w_theta, float _leg_len);
  void SetDist(float _dist);
  void SetSpeed(float _speed);
  float GetWheelTor();
  float GetLegTor();
  void SetNowDist(float _dist) { dist_ = _dist; }
  void SetForceNormal(float _F_N);

 private:
  float dist_, speed_, phi_, w_phi_, theta_, w_theta_, leg_len_;
  float target_dist_, target_speed_;
  float F_N_;
  float T[2];
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __LQR_H_ */
