/**
 *******************************************************************************
 * @file      : vmc.cpp
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
#include "vmc.h"
#include "bsp_dwt.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const float k_thigh_len = 0.15f;
const float k_calf_len = 0.25f;
const float k_joint_len = 0.14f;
const float k_m_wheel = 0.753f;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Sets the value of phi0.
 *
 * This function is used to set the value of phi0 in the Vmc class.
 *
 * @param _phi0 The new value of phi0.
 */
void Vmc::SetPhi(const float _phi) {
  phi_ = _phi;
}

/**
 * @brief Sets the value of phi1 and its speed.
 *
 * @param _phi1 The value of phi1.
 * @param _w_phi1 The speed of phi1.
 */
void Vmc::SetPhi1(const float _phi1, const float _w_phi1) {
  phi1_ = _phi1;
  w_phi1_ = _w_phi1;
}

/**
 * @brief Sets the value of phi4 and its speed.
 *
 * @param _phi4 The value of phi4.
 * @param _w_phi4 The speed of phi4.
 */
void Vmc::SetPhi4(const float _phi4, const float _w_phi4) {
  phi4_ = _phi4;
  w_phi4_ = _w_phi4;
}

void Vmc::SetTor(const float _F, const float Tp) {
  F_ = _F;
  Tp_ = Tp;
}

void Vmc::SetMeasureTor(const float _T1, const float _T2) {
  mea_t1_ = _T1;
  mea_t2_ = _T2;
}

void Vmc::SetMeasureAccelZ(const float _ddot_z_M) {
  ddot_z_M_ = _ddot_z_M;
}

/**
 * @brief 根据关节角度和角速度,计算单杆长度和角度以及变化率
 *
 * @note 右侧视图
 *  ___x
 * |    1  _____  4
 * |y     /     \
 *      2 \     / 3
 *         \   /
 *          \./
 *           5
 * @param p 5连杆和腿的参数
 */
void Vmc::LegCalc() {
  coord_[0] = x_b_ = k_thigh_len * arm_cos_f32(phi1_);
  coord_[1] = y_b_ = k_thigh_len * arm_sin_f32(phi1_);
  coord_[4] = x_d_ = k_joint_len + k_thigh_len * arm_cos_f32(phi4_);
  coord_[5] = y_d_ = k_thigh_len * arm_sin_f32(phi4_);
  bd_ = powf(x_d_ - x_b_, 2) + powf(y_d_ - y_b_, 2);
  a0_ = 2 * k_calf_len * (x_d_ - x_b_);
  b0_ = 2 * k_calf_len * (y_d_ - y_b_);

  phi2_ = 2 * atan2f(b0_ + sqrtf(powf(a0_, 2) + powf(b0_, 2) - powf(bd_, 2)),
                     a0_ + bd_);
  coord_[2] = x_c_ = x_b_ + k_calf_len * arm_cos_f32(phi2_);
  coord_[3] = y_c_ = y_b_ + k_calf_len * arm_sin_f32(phi2_);

  phi0_ = atan2f(y_c_, x_c_ - k_joint_len / 2);
  l0_ = sqrtf(powf(x_c_ - k_joint_len / 2, 2) + powf(y_c_, 2));
  phi3_ = atan2f(y_c_ - y_d_, x_c_ - x_d_);
  theta_ = phi0_ - 0.5 * PI - phi_;
  height_ = l0_ * arm_cos_f32(theta_);

  static float predict_dt = 0.0001f;
  float phi1_pred =
      phi1_ + w_phi1_ * predict_dt;  // 预测下一时刻的关节角度(利用关节角速度)
  float phi4_pred = phi4_ + w_phi4_ * predict_dt;
  // 重新计算腿长和腿角度
  x_b_ = k_thigh_len * arm_cos_f32(phi1_pred);
  y_b_ = k_thigh_len * arm_sin_f32(phi1_pred);
  x_d_ = k_joint_len + k_thigh_len * arm_cos_f32(phi4_pred);
  y_d_ = k_thigh_len * arm_sin_f32(phi4_pred);
  bd_ = powf(x_d_ - x_b_, 2) + powf(y_d_ - y_b_, 2);
  a0_ = 2 * k_calf_len * (x_d_ - x_b_);
  b0_ = 2 * k_calf_len * (y_d_ - y_b_);

  float phi2_pred =
      2 * atan2f(b0_ + sqrtf(powf(a0_, 2) + powf(b0_, 2) - powf(bd_, 2)),
                 a0_ + bd_);
  x_c_ = x_b_ + k_calf_len * arm_cos_f32(phi2_pred);
  y_c_ = y_b_ + k_calf_len * arm_sin_f32(phi2_pred);
  float phi0_pred = atan2f(y_c_, x_c_ - k_joint_len / 2);
  // 差分计算腿长变化率和腿角速度
  w_phi2_ = (phi2_pred - phi2_) / predict_dt;
  w_phi0_ = (phi0_pred - phi0_) / predict_dt;
  v_l0_ = ((sqrtf(powf(x_c_ - k_joint_len / 2, 2) + powf(y_c_, 2))) - l0_) /
          predict_dt;
  w_theta_ = (phi0_pred - 0.5 * PI - phi_ - theta_) / predict_dt;
  v_height_ =
      v_l0_ * arm_cos_f32(theta_) - l0_ * arm_sin_f32(theta_) * w_theta_;
  dot_v_l0_ = (v_l0_ - last_v_l0_) / (0.002f + 0.008f) +
              dot_v_l0_ * 0.008f / (0.002f + 0.008f);
  dotw_theta_ = (w_theta_ - last_w_theta_) / (0.002f + 0.008f) +
                dotw_theta_ * 0.008f / (0.002f + 0.008f);
  ddot_z_w_ = ddot_z_M_ - dot_v_l0_ * arm_cos_f32(theta_) +
              2.0f * v_l0_ * w_theta_ * arm_sin_f32(theta_) +
              l0_ * dotw_theta_ * arm_cos_f32(theta_) +
              l0_ * powf(w_theta_, 2) * arm_sin_f32(theta_);
  last_w_theta_ = w_theta_;
  last_v_l0_ = v_l0_;
}

/**
 * @brief Calculate the torque for Vmc.
 */
void Vmc::TorCalc() {
  float phi23 = arm_sin_f32(phi2_ - phi3_);
  float phi03 = phi0_ - phi3_;
  float phi02 = phi0_ - phi2_;
  float F_m_L = F_ * l0_;
  T1_ = -(k_thigh_len * arm_sin_f32(phi1_ - phi2_)) *
        (F_m_L * arm_sin_f32(phi03) + Tp_ * arm_cos_f32(phi03)) / (l0_ * phi23);
  T2_ = -(k_thigh_len * arm_sin_f32(phi3_ - phi4_)) *
        (F_m_L * arm_sin_f32(phi02) + Tp_ * arm_cos_f32(phi02)) / (l0_ * phi23);
}

void Vmc::LegForceCalc() {
  float phi23 = arm_sin_f32(phi2_ - phi3_);
  float phi03 = phi0_ - phi3_;
  float phi02 = phi0_ - phi2_;

  mea_F_ =
      -arm_cos_f32(phi02) * mea_t1_ / k_thigh_len * arm_sin_f32(phi1_ - phi2_) +
      arm_cos_f32(phi03) * mea_t2_ / k_thigh_len * arm_sin_f32(phi3_ - phi4_);
  mea_Tp_ = l0_ * arm_sin_f32(phi02) * mea_t1_ / k_thigh_len *
                arm_sin_f32(phi1_ - phi2_) -
            l0_ * arm_sin_f32(phi03) * mea_t2_ / k_thigh_len *
                arm_sin_f32(phi3_ - phi4_);
  P_ = mea_F_ * arm_cos_f32(theta_) + mea_Tp_ * arm_sin_f32(theta_) / l0_;
  F_N_ = P_ + k_m_wheel * (9.8f + ddot_z_w_);
}

/**
 * @brief Get the value of T1.
 *
 * @return The value of T1.
 */
float Vmc::GetT1() {
  return T1_;
}

/**
 * @brief Get the value of T2.
 *
 * @return The value of T2 as a float.
 */
float Vmc::GetT2() {
  return T2_;
}

/**
 * @brief Get the value of theta.
 *
 * @return The value of theta.
 */
float Vmc::GetTheta() {
  return theta_;
}

/**
 * @brief Returns the value of dot theta.
 *
 * @return The value of dot theta.
 */
float Vmc::GetDotTheta() {
  return w_theta_;
}

float Vmc::GetLegLen() {
  return l0_;
}

float Vmc::GetLegSpeed() {
  return v_l0_;
}

float Vmc::GetPhi0() {
  return phi0_;
}

float Vmc::GetPhi2Speed() {
  return w_phi2_;
}

float Vmc::GetForceNormal() {
  return F_N_;
}