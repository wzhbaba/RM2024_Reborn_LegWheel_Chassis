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
    127.0450f,  -125.3059f, -13.4588f, 23.6918f,  -21.6399f, -3.2698f,
    13.8229f,   -18.3555f,  -6.9740f,  2.1570f,   -9.1866f,  -9.5180f,
    74.9013f,   -86.8550f,  41.0150f,  9.5681f,   -8.3822f,  3.6301f,
    -184.9687f, 77.4922f,   24.3763f,  -6.6134f,  -7.8840f,  10.3030f,
    -46.8989f,  16.0003f,   8.7239f,   -18.9932f, -4.2825f,  12.7259f,
    -194.1780f, 158.9239f,  47.8383f,  -11.0802f, 9.8402f,   1.5287f};

const float k_fly[12][3] = {
    0.0f, 0.0f, 0.0f,       0.0f,     0.0f,     0.0f,     0.0f,     0.0f,
    0.0f, 0.0f, 0.0f,       0.0f,     0.0f,     0.0f,     0.0f,     0.0f,
    0.0f, 0.0f, -184.9687f, 77.4922f, 24.3763f, -6.6134f, -7.8840f, 10.3030f,
    0.0f, 0.0f, 0.0f,       0.0f,     0.0f,     0.0f,     0.0f,     0.0f,
    0.0f, 0.0f, 0.0f,       0.0f};
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