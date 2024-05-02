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

#include "stdint.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const float k_normal[12][3] = {
    241.9500f,  -216.7255f, -17.2145f, 41.4590f,  -35.0665f, -3.1927f,
    38.4106f,   -37.8546f,  -7.9534f,  23.4326f,  -26.3768f, -11.2291f,
    144.9368f,  -139.5013f, 53.5035f,  22.5104f,  -18.7940f, 6.8232f,
    -250.0759f, 100.9504f,  31.7428f,  -9.5804f,  -8.9727f,  11.9230f,
    -62.4203f,  18.6225f,   12.1779f,  -32.5684f, -2.2591f,  15.8797f,
    -301.8437f, 225.6687f,  60.5659f,  -23.8920f, 19.4070f,  3.6610f};

const float k_fly[12][3] = {
    0.0f, 0.0f, 0.0f,       0.0f,      0.0f,     0.0f,     0.0f,     0.0f,
    0.0f, 0.0f, 0.0f,       0.0f,      0.0f,     0.0f,     0.0f,     0.0f,
    0.0f, 0.0f, -250.0759f, 100.9504f, 31.7428f, -9.5804f, -8.9727f, 11.9230f,
    0.0f, 0.0f, 0.0f,       0.0f,      0.0f,     0.0f,     0.0f,     0.0f,
    0.0f, 0.0f, 0.0f,       0.0f};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

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