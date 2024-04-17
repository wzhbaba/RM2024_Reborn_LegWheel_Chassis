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
    80.3358f,   -75.1603f,  -2.8836f, 3.8191f,   -4.6610f,  -0.2156f,
    71.1150f,   -49.8081f,  -3.9310f, 43.1462f,  -31.1109f, -3.4800f,
    183.8205f,  -157.3780f, 42.5967f, 8.5675f,   -8.6392f,  3.4622f,
    -3.5551f,   -11.8045f,  9.1028f,  2.6910f,   -3.0386f,  0.9481f,
    30.1358f,   -39.4817f,  14.5203f, 28.4033f,  -31.6603f, 10.2124f,
    -269.5233f, 189.1161f,  48.1903f, -16.8206f, 12.8475f,  2.3726f};

const float k_fly[12][3] = {
    0.0f,     0.0f,      0.0f,    0.0f,    0.0f,     0.0f,    0.0f, 0.0f, 0.0f,
    0.0f,     0.0f,      0.0f,    0.0f,    0.0f,     0.0f,    0.0f, 0.0f, 0.0f,
    -3.5551f, -11.8045f, 9.1028f, 2.6910f, -3.0386f, 0.9481f, 0.0f, 0.0f, 0.0f,
    0.0f,     0.0f,      0.0f,    0.0f,    0.0f,     0.0f,    0.0f, 0.0f, 0.0f};
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