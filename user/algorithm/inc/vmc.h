/**
 *******************************************************************************
 * @file      : vmc.h
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
#ifndef __VMC_H_
#define __VMC_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief The Vmc class represents a virtual machine controller.
 *
 * This class provides methods for calculating leg and torso movements,
 * setting and getting various parameters, and storing internal state variables.
 */
class Vmc
{
   public:
    void LegCalc();
    void TorCalc();
    void LegForceCalc();
    void SetPhi(const float _phi);
    void SetPhi1(const float _phi1, const float _w_phi1);
    void SetPhi4(const float _phi4, const float _w_phi4);
    void SetTor(const float _F, const float _Tp);
    void SetMeasureTor(const float _T1, const float _T2);
    void SetMeasureAccelZ(const float _ddot_z_M);
    float GetT1();
    float GetT2();
    float GetTheta();
    float GetDotTheta();
    float GetLegLen();
    float GetLegSpeed();
    float GetPhi0();
    float GetPhi2Speed();
    float GetForceNormal();

   private:
    float phi_, phi0_, phi1_, phi4_, w_phi1_, w_phi4_, l0_;
    float theta_, height_, w_theta_, v_height_, dotw_theta_, dotv_height_, w_phi2_, v_l0_, ddot_z_w_, dot_v_l0_, w_phi0_, ddot_z_M_;
    float T1_, T2_, F_, Tp_, mea_t1_, mea_t2_, mea_F_, mea_Tp_, F_N_, P_;
    float x_b_, y_b_, x_c_, y_c_, x_d_, y_d_, p_;
    float coord_[6];  // xb yb xc yc xd yd
    float last_theta_, last_height_, last_w_theta_, last_v_height_, last_phi2_, last_l0_, last_v_l0_;
    float phi2_, phi3_;
    float a0_, b0_;
    float bd_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __VMC_H_ */
