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
    161.5242f,  -163.7835f, -15.7297f, 30.4330f,  -28.1035f, -3.6883f,
    21.6596f,   -28.7062f,  -8.8252f,  6.1433f,   -16.0155f, -11.9988f,
    106.3954f,  -124.9992f, 57.0086f,  12.4580f,  -11.8198f, 5.3525f,
    -237.8572f, 98.5413f,   28.5542f,  -12.2442f, -6.9050f,  11.4729f,
    -69.1019f,  24.9105f,   10.5928f,  -34.3554f, 0.0185f,   15.1019f,
    -274.3419f, 223.0869f,  66.7683f,  -18.6832f, 16.4481f,  2.9898f};

const float k_fly[12][3] = {
    0.0f, 0.0f, 0.0f,       0.0f,     0.0f,     0.0f,      0.0f,     0.0f,
    0.0f, 0.0f, 0.0f,       0.0f,     0.0f,     0.0f,      0.0f,     0.0f,
    0.0f, 0.0f, -237.8572f, 98.5413f, 28.5542f, -12.2442f, -6.9050f, 11.4729f,
    0.0f, 0.0f, 0.0f,       0.0f,     0.0f,     0.0f,      0.0f,     0.0f,
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