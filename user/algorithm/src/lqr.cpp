/**
 *******************************************************************************
 * @file      : lqr.cpp
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
#include "lqr.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const float k_normal[12][3] = {
    52.4761f,   -55.1002f,  -6.7853f,  -1.5895f,  -0.8728f,  -0.9765f,
    30.3675f,   -20.7042f,  -10.1873f, 5.1366f,   -5.0293f,  -8.8986f,
    158.3351f,  -135.1063f, 36.7366f,  10.8276f,  -9.5354f,  3.3765f,
    27.0323f,   -32.5116f,  11.5731f,  9.5448f,   -7.1970f,  1.5601f,
    79.2267f,   -70.1871f,  19.2449f,  77.5068f,  -61.6245f, 14.7540f,
    -220.4920f, 147.1135f,  62.3805f,  -17.8303f, 12.3671f,  2.0752f};

const float k_fly[12][3] = {
    0.0f, 0.0f, 0.0f,     0.0f,      0.0f,     0.0f,    0.0f,     0.0f,
    0.0f, 0.0f, 0.0f,     0.0f,      0.0f,     0.0f,    0.0f,     0.0f,
    0.0f, 0.0f, 27.0323f, -32.5116f, 11.5731f, 9.5448f, -7.1970f, 1.5601f,
    0.0f, 0.0f, 0.0f,     0.0f,      0.0f,     0.0f,    0.0f,     0.0f,
    0.0f, 0.0f, 0.0f,     0.0f};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void Lqr::SetData(float _dist, float _speed, float _phi, float _w_phi,
                  float _theta, float _w_theta, float _leg_len) {
  dist_ = _dist;
  speed_ = _speed;
  phi_ = _phi;
  w_phi_ = _w_phi;
  theta_ = _theta;
  w_theta_ = _w_theta;
  leg_len_ = _leg_len;
}

void Lqr::SetDist(float _dist) {
  target_dist_ = _dist;
}

void Lqr::SetSpeed(float _speed) {
  target_speed_ = _speed;
}

void Lqr::SetForceNormal(float _F_N) {
  F_N_ = _F_N;
}

float Lqr::GetWheelTor() {
  return T[0];
}

float Lqr::GetLegTor() {
  return T[1];
}

void Lqr::Calc() {
  float lsqr = leg_len_ * leg_len_;
  if (F_N_ < 20.0f) {
    for (uint8_t i = 0; i < 2; ++i) {
      uint8_t j = i * 6;
      T[i] = (k_fly[j + 0][0] * lsqr + k_fly[j + 0][1] * leg_len_ +
              k_fly[j + 0][2]) *
                 -theta_ +
             (k_fly[j + 1][0] * lsqr + k_fly[j + 1][1] * leg_len_ +
              k_fly[j + 1][2]) *
                 -w_theta_ +
             (k_fly[j + 2][0] * lsqr + k_fly[j + 2][1] * leg_len_ +
              k_fly[j + 2][2]) *
                 -dist_ +
             (k_fly[j + 3][0] * lsqr + k_fly[j + 3][1] * leg_len_ +
              k_fly[j + 3][2]) *
                 (target_speed_ - speed_) +
             (k_fly[j + 4][0] * lsqr + k_fly[j + 4][1] * leg_len_ +
              k_fly[j + 4][2]) *
                 -phi_ +
             (k_fly[j + 5][0] * lsqr + k_fly[j + 5][1] * leg_len_ +
              k_fly[j + 5][2]) *
                 -w_phi_;
    }
  } else {
    for (uint8_t i = 0; i < 2; ++i) {
      uint8_t j = i * 6;
      T[i] = (k_normal[j + 0][0] * lsqr + k_normal[j + 0][1] * leg_len_ +
              k_normal[j + 0][2]) *
                 -theta_ +
             (k_normal[j + 1][0] * lsqr + k_normal[j + 1][1] * leg_len_ +
              k_normal[j + 1][2]) *
                 -w_theta_ +
             (k_normal[j + 2][0] * lsqr + k_normal[j + 2][1] * leg_len_ +
              k_normal[j + 2][2]) *
                 -dist_ +
             (k_normal[j + 3][0] * lsqr + k_normal[j + 3][1] * leg_len_ +
              k_normal[j + 3][2]) *
                 (target_speed_ - speed_) +
             (k_normal[j + 4][0] * lsqr + k_normal[j + 4][1] * leg_len_ +
              k_normal[j + 4][2]) *
                 -phi_ +
             (k_normal[j + 5][0] * lsqr + k_normal[j + 5][1] * leg_len_ +
              k_normal[j + 5][2]) *
                 -w_phi_;
    }
  }
}