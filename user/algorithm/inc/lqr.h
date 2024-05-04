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

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Lqr {
 public:
  void Calc();
  void SetData(const float _dist, const float _speed, const float _phi,
               const float _w_phi, const float _theta, const float _w_theta,
               const float _leg_len, const float _F_N) {
    dist_ = _dist;
    speed_ = _speed;
    phi_ = _phi;
    w_phi_ = _w_phi;
    theta_ = _theta;
    w_theta_ = _w_theta;
    leg_len_ = _leg_len;
    F_N_ = _F_N;
  }
  void SetSpeed(const float _speed) { target_speed_ = _speed; };
  void SetNowDist(const float _dist) { dist_ = _dist; }
  float GetWheelTor() { return T_[0]; };
  float GetLegTor() { return T_[1]; };

 private:
  float dist_, speed_, phi_, w_phi_, theta_, w_theta_, leg_len_;
  float target_speed_, F_N_, T_[2], T_K_[2][6];
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* __LQR_H_ */
