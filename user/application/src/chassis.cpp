/**
 *******************************************************************************
 * @file      : chassis.cpp
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
#include <chassis.h>

#include "board_comm.h"
#include "bsp_dwt.h"
#include "ins.h"
#include "kalman_filter.h"
#include "remote.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const float k_gravity_comp = 4.03f * 9.8f;
const float k_roll_extra_comp_p = 500.0f;
const float k_wheel_radius = 0.1f;

const float k_lf_joint_bias = 0.724f;
const float k_lb_joint_bias = 4.623f;
const float k_rf_joint_bias = 2.357f;
const float k_rb_joint_bias = 0.62f;

static float target_yaw;
static KalmanFilter_t kf;

/* External variables --------------------------------------------------------*/
Chassis chassis;
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 底盘为右手系
 *
 *    ^ y            左轮     右轮
 *    |               |      |  前
 *    |_____ >  x     |------|
 *    z 轴从屏幕向外   |      |  后
 */

// 电机选型：宇树A1
// 电机位置：前1后0

void Chassis::SpeedEstInit() {
  // 使用kf同时估计速度和加速度
  Kalman_Filter_Init(&kf, 2, 0, 2);
  float F[4] = {1, 0.001, 0, 1};
  float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_PROCESS_NOISE};
  float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};
  float P[4] = {100000, 0, 0, 100000};
  float H[4] = {1, 0, 0, 1};
  memcpy(kf.F_data, F, sizeof(F));
  memcpy(kf.Q_data, Q, sizeof(Q));
  memcpy(kf.R_data, R, sizeof(R));
  memcpy(kf.P_data, P, sizeof(P));
  memcpy(kf.H_data, H, sizeof(H));
}

static void LeftWheelCallback() {
  chassis.l_wheel_.Update();
}

static void RightWheelCallback() {
  chassis.r_wheel_.Update();
}

/**
 * @brief Initializes the motors of the chassis.
 */
void Chassis::MotorInit() {
  l_wheel_.Init(&hcan1, 0x142);
  r_wheel_.Init(&hcan1, 0x141);
  lf_joint_.Init(&huart2, 0x01, 10);
  lb_joint_.Init(&huart2, 0x00, 10);
  rf_joint_.Init(&huart1, 0x01, 10);
  rb_joint_.Init(&huart1, 0x00, 10);

  l_wheel_.p_motor_instance_->pCanCallBack = LeftWheelCallback;
  r_wheel_.p_motor_instance_->pCanCallBack = RightWheelCallback;
}

void Chassis::PidInit() {
  left_leg_len_.Init(800.0f, 0.0f, 5.0f, 60.0f, 0.0001f);
  right_leg_len_.Init(800.0f, 0.0f, 5.0f, 60.0f, 0.0001f);
  anti_crash_.Init(8.0f, 0.0f, 2.0f, 10.0f, 0.001f);
  roll_ctrl_.Init(0.001f, 0.00065f, 0.0f, 0.04f, 0.001f);
  yaw_pos_.Init(0.5f, 0.0f, 0.0f, 5.0f, 0.01f);
  yaw_speed_.Init(1.0f, 0.0f, 0.0f, 10.0f, 0.0f);
  left_leg_len_.Inprovement(PID_CHANGING_INTEGRATION_RATE |
                                PID_TRAPEZOID_INTEGRAL | PID_DERIVATIVE_FILTER |
                                PID_DERIVATIVE_ON_MEASUREMENT,
                            0.0f, 0.0f, 0.01f, 0.02f, 0.08f);
  right_leg_len_.Inprovement(
      PID_CHANGING_INTEGRATION_RATE | PID_TRAPEZOID_INTEGRAL |
          PID_DERIVATIVE_FILTER | PID_DERIVATIVE_ON_MEASUREMENT,
      0.0f, 0.0f, 0.01f, 0.02f, 0.08f);
  anti_crash_.Inprovement(PID_DERIVATIVE_FILTER, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f);
  roll_ctrl_.Inprovement(PID_DERIVATIVE_FILTER, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f);
  yaw_pos_.Inprovement(PID_DERIVATIVE_FILTER, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f);
  yaw_speed_.Inprovement(PID_DERIVATIVE_FILTER | PID_INTEGRAL_LIMIT, 2.0f, 0.0f,
                         0.0f, 0.0f, 0.05f);
}

void Chassis::SetLegData() {
  float left_angle[2], right_angle[2];
  left_angle[0] = (lf_joint_.GetAngle() - k_lf_joint_bias) / 9.1f - 0.724f;
  left_angle[1] =
      (lb_joint_.GetAngle() - k_lb_joint_bias) / 9.1f + 0.724f + 3.141f;
  right_angle[0] = -(rf_joint_.GetAngle() - k_rf_joint_bias) / 9.1f - 0.724f;
  right_angle[1] =
      -(rb_joint_.GetAngle() - k_rb_joint_bias) / 9.1f + 0.724f + 3.141f;
  vmc_left_.SetPhi(INS.Pitch * DEGREE_2_RAD);
  vmc_right_.SetPhi(INS.Pitch * DEGREE_2_RAD);
  vmc_left_.SetPhi1(left_angle[1], lb_joint_.GetSpeed());
  vmc_right_.SetPhi1(right_angle[1], -rb_joint_.GetSpeed());
  vmc_left_.SetPhi4(left_angle[0], lf_joint_.GetSpeed());
  vmc_right_.SetPhi4(right_angle[0], -rf_joint_.GetSpeed());
  vmc_left_.SetMeasureTor(lb_joint_.GetTor(), lf_joint_.GetTor());
  vmc_right_.SetMeasureTor(-rb_joint_.GetTor(), -rf_joint_.GetTor());
  vmc_left_.SetMeasureAccelZ(INS.MotionAccel_n[Z]);
  vmc_right_.SetMeasureAccelZ(INS.MotionAccel_n[Z]);
}

void Chassis::SetTargetYaw(float _pos) {
  target_yaw_ += _pos;
}

void Chassis::SetLegLen() {
  roll_ctrl_.SetRef(0.0f);

  left_leg_len_.SetMeasure(vmc_left_.GetLegLen());
  right_leg_len_.SetMeasure(vmc_right_.GetLegLen());
}

void Chassis::SetVMCData() {
  vmc_left_.SetTor(left_leg_F_, left_leg_T_);
  vmc_right_.SetTor(right_leg_F_, right_leg_T_);
}

void Chassis::SetLQRData() {
  lqr_left_.SetSpeed(target_speed_);
  lqr_right_.SetSpeed(target_speed_);
  lqr_left_.SetDist(target_dist_);
  lqr_right_.SetDist(target_dist_);
  lqr_left_.SetData(dist_, vel_, INS.Pitch * DEGREE_2_RAD, INS.Gyro[X],
                    vmc_left_.GetTheta(), vmc_left_.GetDotTheta(),
                    vmc_left_.GetLegLen());
  lqr_right_.SetData(dist_, vel_, INS.Pitch * DEGREE_2_RAD, INS.Gyro[X],
                     vmc_right_.GetTheta(), vmc_right_.GetDotTheta(),
                     vmc_right_.GetLegLen());
  lqr_left_.SetForceNormal(vmc_left_.GetForceNormal());
  lqr_right_.SetForceNormal(vmc_right_.GetForceNormal());
}

void Chassis::LQRCalc() {
  lqr_left_.Calc();
  lqr_right_.Calc();
}

void Chassis::VMCLegCalc() {
  vmc_left_.LegCalc();
  vmc_right_.LegCalc();
  vmc_left_.LegForceCalc();
  vmc_right_.LegForceCalc();
}

void Chassis::VMCTorCalc() {
  vmc_left_.TorCalc();
  vmc_right_.TorCalc();
}

// TODO:速度融合仍需完善
void Chassis::SpeedCalc() {
  left_w_wheel_ = l_wheel_.GetSpeed() + vmc_left_.GetPhi2Speed() - INS.Gyro[X];
  right_w_wheel_ =
      -r_wheel_.GetSpeed() + vmc_right_.GetPhi2Speed() - INS.Gyro[X];

  left_v_body_ = left_w_wheel_ * k_wheel_radius +
                 vmc_left_.GetLegLen() * vmc_left_.GetDotTheta() +
                 vmc_left_.GetLegSpeed() * arm_sin_f32(vmc_left_.GetTheta());
  right_v_body_ = right_w_wheel_ * k_wheel_radius +
                  vmc_right_.GetLegLen() * vmc_right_.GetDotTheta() +
                  vmc_right_.GetLegSpeed() * arm_sin_f32(vmc_right_.GetTheta());
  vel_m = (left_v_body_ + right_v_body_) / 2;

  static float yaw_ddwrNwwr, yaw_p_ddwrNwwr, pitch_ddwrNwwr;
  static float macc_y, macc_z, acc_last,
      acc_m;  // 补偿后的实际平动加速度,机体系前进方向和竖直方向

  yaw_ddwrNwwr =
      INS.dGyro[Z] *
      CENTER_IMU_W;  // yaw旋转导致motion_acc[1]的额外加速度(机体前后方向)
  yaw_p_ddwrNwwr =
      powf(INS.Gyro[X], 2) * CENTER_IMU_L +
      INS.dGyro[X] *
          CENTER_IMU_H;  // pitch旋转导致motion_acc[1]的额外加速度(机体前后方向)
  pitch_ddwrNwwr =
      powf(INS.Gyro[X], 2) * CENTER_IMU_H +
      INS.dGyro[X] *
          CENTER_IMU_L;  // pitch旋转导致motion_acc[2]的额外加速度(机体竖直方向)

  macc_y = INS.MotionAccel_b[Y] - yaw_ddwrNwwr - yaw_p_ddwrNwwr;
  macc_z = INS.MotionAccel_b[Z] - pitch_ddwrNwwr;

  acc_last = acc_m;
  acc_m =
      macc_y * arm_cos_f32(INS.Pitch * DEGREE_2_RAD) -
      macc_z *
          arm_sin_f32(
              INS.Pitch *
              DEGREE_2_RAD);  // 绝对系下的平动加速度,即机体系下的加速度投影到绝对系

  // 使用kf同时估计加速度和速度,滤波更新
  kf.MeasuredVector[0] = vel_m;
  kf.MeasuredVector[1] = acc_m;
  kf.F_data[1] = dt_;  // 更新F矩阵
  Kalman_Filter_Update(&kf);
  vel_ = kf.xhat_data[0];
  acc_ = kf.xhat_data[1];

  dist_ = dist_ + vel_ * dt_;
}

void Chassis::LegCalc() {
  roll_ctrl_.SetMeasure(INS.Roll);
  roll_ctrl_.Calculate();
  left_leg_len_.SetRef(0.18f + roll_ctrl_.GetOutput());
  right_leg_len_.SetRef(0.18f - roll_ctrl_.GetOutput());

  float roll_comp = k_roll_extra_comp_p * INS.Roll * DEGREE_2_RAD;
  left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp - roll_comp;
  right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp + roll_comp;
}

void Chassis::SynthesizeMotion() {
  SetTargetYaw(((-remote.GetCh0() / 660.0f) * 1000.0f) * 0.001f);
  if (vmc_left_.GetForceNormal() < 20.0f ||
      vmc_right_.GetForceNormal() < 20.0f) {
    yaw_pos_.SetRef(INS.YawTotalAngle);
  } else {
    yaw_pos_.SetRef(target_yaw_);
  }
  yaw_pos_.SetMeasure(INS.YawTotalAngle);
  yaw_speed_.SetRef(yaw_pos_.Calculate());
  yaw_speed_.SetMeasure(INS.Gyro[Z]);
  yaw_speed_.Calculate();
  if (vmc_left_.GetForceNormal() < 20.0f) {
    l_wheel_T_ = lqr_left_.GetWheelTor();
  } else {
    l_wheel_T_ = lqr_left_.GetWheelTor() - yaw_speed_.GetOutput();
  }
  if (vmc_right_.GetForceNormal() < 20.0f) {
    r_wheel_T_ = lqr_right_.GetWheelTor();
  } else {
    r_wheel_T_ = lqr_right_.GetWheelTor() + yaw_speed_.GetOutput();
  }

  anti_crash_.SetRef(0.0f);
  anti_crash_.SetMeasure(vmc_left_.GetPhi0() - vmc_right_.GetPhi0());
  anti_crash_.Calculate();
  left_leg_T_ = lqr_left_.GetLegTor() + anti_crash_.GetOutput();
  right_leg_T_ = lqr_right_.GetLegTor() - anti_crash_.GetOutput();
}

void Chassis::Ctrl() {
  dt_ = DWT_GetDeltaT(&dwt_cnt_);
  // Switch();
  SetState();
  SetLegData();
  VMCLegCalc();
  SpeedCalc();
  SetLQRData();
  LQRCalc();
  SynthesizeMotion();
  SetLegLen();
  LegCalc();
  SetVMCData();
  VMCTorCalc();
}

void Chassis::SetMotorTor() {
  chassis.lf_joint_.SetMotorT(chassis.GetLFJoint());
  chassis.lb_joint_.SetMotorT(chassis.GetLBJoint());
  chassis.rf_joint_.SetMotorT(chassis.GetRFJoint());
  chassis.rb_joint_.SetMotorT(chassis.GetRBJoint());
  chassis.l_wheel_.SetTor(chassis.GetLeftWheel());
  chassis.r_wheel_.SetTor(chassis.GetRightWheel());
}

void Chassis::StopMotor() {
  chassis.lf_joint_.SetMotorT(0.0f);
  chassis.rf_joint_.SetMotorT(0.0f);
  chassis.lb_joint_.SetMotorT(0.0f);
  chassis.rb_joint_.SetMotorT(0.0f);
  chassis.l_wheel_.SetTor(0.0f);
  chassis.r_wheel_.SetTor(0.0f);
}

void Chassis::Reset() {
  target_dist_ = 0.0f;
  lqr_left_.SetNowDist(0.0f);
  lqr_right_.SetNowDist(0.0f);
  if (fabsf(lf_joint_.GetAngle()) < 1.9f &&
      fabsf(lf_joint_.GetAngle()) > 1.5f &&
      fabsf(lb_joint_.GetAngle()) < 4.2f &&
      fabsf(lb_joint_.GetAngle()) > 3.8f &&
      fabsf(rf_joint_.GetAngle()) < 0.95f &&
      fabsf(rf_joint_.GetAngle()) > 0.6f &&
      fabsf(rb_joint_.GetAngle()) < 0.65f &&
      fabsf(rb_joint_.GetAngle()) > 0.45f) {
    robot_state_ = ROBOT_READY;
    // chassis.lf_joint_.SetMotorT(0.0f);
    // chassis.rf_joint_.SetMotorT(0.0f);
    // chassis.lb_joint_.SetMotorT(0.0f);
    // chassis.rb_joint_.SetMotorT(0.0f);
    // chassis_state_ = CHASSIS_NORMAL;
    return;
  } else {
    robot_state_ = ROBOT_STOP;
  }

  lf_joint_.SetMotorPos(k_lf_joint_bias);
  lb_joint_.SetMotorPos(k_lb_joint_bias);
  rf_joint_.SetMotorPos(k_rf_joint_bias);
  rb_joint_.SetMotorPos(k_rb_joint_bias);
}

void Chassis::Switch() {
  if (board_comm.GetReadyFlag() == 1) {
    chassis_state_ = CHASSIS_RESET;
  } else {
    chassis_state_ = CHASSIS_STOP;
  }
}

void Chassis::SetState() {
  // if (chassis_state_ == CHASSIS_STOP) {
  //     StopMotor();
  //     return;
  // } else if (chassis_state_ == CHASSIS_RESET) {
  //     Reset();
  //     return;
  // }

  target_speed_ = (board_comm.GetYSpeed() / 2000.0f) * 2.0f;
  target_dist_ += target_speed_ * dt_;
}

float Chassis::GetLeftWheel() {
  return l_wheel_T_;
}

float Chassis::GetRightWheel() {
  return -r_wheel_T_;
}

float Chassis::GetLFJoint() {
  return vmc_left_.GetT2();
}
float Chassis::GetRFJoint() {
  return -vmc_right_.GetT2();
}
float Chassis::GetLBJoint() {
  return vmc_left_.GetT1();
}
float Chassis::GetRBJoint() {
  return -vmc_right_.GetT1();
}
