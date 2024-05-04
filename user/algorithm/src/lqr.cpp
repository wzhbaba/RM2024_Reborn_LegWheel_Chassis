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
const float k[12][3] = {
    195.7494,  -180.3480, -13.6831, 37.7849,  -32.6044, -3.5317,
    19.3118,   -19.2291,  -3.9199,  22.7366,  -24.3851, -8.0670,
    133.8854,  -140.4911, 54.0384,  21.2890,  -21.0922, 8.3306,
    -188.8917, 60.8518,   29.3627,  -6.6440,  -11.9558, 11.5662,
    -24.1159,  3.7865,    6.3998,   -19.5286, -7.2078,  12.6792,
    -303.4386, 229.5646,  63.6633,  -41.9449, 32.6389,  7.1862,
};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void Lqr::Calc() {
  float lsqr = leg_len_ * leg_len_;

  for (uint8_t i = 0; i < 2; ++i) {
    uint8_t j = i * 6;
    T_K_[i][0] =
        (k[j + 0][0] * lsqr + k[j + 0][1] * leg_len_ + k[j + 0][2]) * -theta_;
    T_K_[i][1] =
        (k[j + 1][0] * lsqr + k[j + 1][1] * leg_len_ + k[j + 1][2]) * -w_theta_;
    T_K_[i][2] =
        (k[j + 2][0] * lsqr + k[j + 2][1] * leg_len_ + k[j + 2][2]) * -dist_;
    T_K_[i][3] = (k[j + 3][0] * lsqr + k[j + 3][1] * leg_len_ + k[j + 3][2]) *
                 (target_speed_ - speed_);
    T_K_[i][4] =
        (k[j + 4][0] * lsqr + k[j + 4][1] * leg_len_ + k[j + 4][2]) * -phi_;
    T_K_[i][5] =
        (k[j + 5][0] * lsqr + k[j + 5][1] * leg_len_ + k[j + 5][2]) * -w_phi_;
  }

  if (F_N_ < 20.0f) {
    for (uint8_t i = 0; i < 6; ++i) {
      T_K_[0][i] = 0.0f;
    };
    T_K_[1][2] = T_K_[1][3] = T_K_[1][4] = T_K_[1][5] = 0.0f;
  }

  for (uint8_t i = 0; i < 2; ++i) {
    T_[i] = T_K_[i][0] + T_K_[i][1] + T_K_[i][2] + T_K_[i][3] + T_K_[i][4] +
            T_K_[i][5];
  }
}