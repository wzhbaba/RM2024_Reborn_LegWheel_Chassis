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

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief The Vmc class represents a virtual machine controller.
 *
 * This class provides methods for calculating leg and torso movements,
 * setting and getting various parameters, and storing internal state variables.
 */
class Vmc {
 public:
  void LegCalc();
  void Jacobian();
  void TorCalc();
  void LegForceCalc();
  void SetBodyData(const float _phi, const float _acc_z) {
    phi_ = _phi;
    ddot_z_M_ = _acc_z;
  }
  void SetLegData(const float _phi1, const float _w_phi1, const float _phi4,
                  const float _w_phi4, const float _t1, const float _t2) {
    phi1_ = _phi1;
    w_phi1_ = _w_phi1;
    phi4_ = _phi4;
    w_phi4_ = _w_phi4;
    mea_t1_ = _t1;
    mea_t2_ = _t2;
  };
  void SetTor(const float _F, const float _Tp) {
    F_ = _F;
    Tp_ = _Tp;
  };
  float GetT1() { return T1_; };
  float GetT2() { return T2_; };
  float GetTheta() { return theta_; };
  float GetDotTheta() { return w_theta_; };
  float GetLegLen() { return l0_; };
  float GetLegSpeed() { return v_l0_; };
  float GetPhi0() { return phi0_; };
  float GetPhi2Speed() { return w_phi2_; };
  float GetForceNormal() { return F_N_; };

 private:
  float phi_, phi0_, phi1_, phi4_, w_phi1_, w_phi4_, l0_;
  float theta_, height_, w_theta_, v_height_, dotw_theta_, dotv_height_,
      w_phi2_, v_l0_, ddot_z_w_, dot_v_l0_, w_phi0_, ddot_z_M_;
  float T1_, T2_, F_, Tp_, mea_t1_, mea_t2_, mea_F_, mea_Tp_, F_N_, P_;
  float x_b_, y_b_, x_c_, y_c_, x_d_, y_d_, p_;
  float coord_[6];  // xb yb xc yc xd yd
  float last_w_theta_, last_v_l0_;
  float phi2_, phi3_;
  float a0_, b0_, bd_;
  float j_[4], inv_j_[4];
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* __VMC_H_ */
