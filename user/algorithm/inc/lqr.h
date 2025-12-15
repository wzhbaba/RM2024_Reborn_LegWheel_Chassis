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
               const float _w_phi, const float _yaw, const float _w_yaw,
               const float _theta_l, const float _w_theta_l,
               const float _theta_r, const float _w_theta_r,
               const float _roll, const float _w_roll,
               const float _leg_len_l, const float _leg_len_r, const float _F_N_l, const float _F_N_r) {
    dist_ = _dist;
    speed_ = _speed;
    phi_ = _phi;
    w_phi_ = _w_phi;
    yaw_ = _yaw;
    w_yaw_ = _w_yaw;
    theta_r_ = _theta_r;
    theta_l_ = _theta_l;
    w_theta_l = _w_theta_l;
    w_theta_r = _w_theta_r;
    roll_ = _roll;
    w_roll_ = _w_roll;
    leg_len_l_ = _leg_len_l;
    leg_len_r_ = _leg_len_r;
    F_N_r_ = _F_N_r;
    F_N_l_ = _F_N_l;
  }
  void SetSpeed(const float _speed) { target_speed_ = _speed; };
  void SetNowDist(const float _dist) { target_dist_ = _dist; }
  void SetYaw(const float _yaw) { target_yaw_ = yaw_; }
  void SetWYaw(const float _w_yaw) { target_w_yaw_ = _w_yaw; }
  float GetLeftWheelTor() { return T_[0]; };
  float GetRightWheelTor() { return T_[2]; };
  float GetLeftLegTor() { return T_[1]; };
  float GetRightLegTor() { return T_[3]; };

private:
  float dist_, speed_, phi_, w_phi_, leg_len_l_, leg_len_r_;
  float yaw_, w_yaw_, roll_, w_roll_, theta_l_, w_theta_l, theta_r_, w_theta_r;
  float target_speed_, target_dist_, target_yaw_, target_w_yaw_, F_N_l_, F_N_r_, T_[4], T_K_[4][12];
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* __LQR_H_ */
