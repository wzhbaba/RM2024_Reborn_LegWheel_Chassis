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
const float k_gravity_comp = 7.4f * 9.8f;
const float k_roll_extra_comp_p = 500.0f;
const float k_wheel_radius = 0.076f;

const float k_phi1_bias = 0.724f + 3.141f;
const float k_phi4_bias = -0.724f;

const float k_lf_joint_bias = 0.7236f;
const float k_lb_joint_bias = 2.941f;
const float k_rf_joint_bias = 3.3689f;
const float k_rb_joint_bias = 2.492f;

const float k_jump_force = 200.0f;
const float k_jump_time = 0.15f;
const float k_retract_force = -120.0f;
const float k_retract_time = 0.1f;

static float target_yaw;

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
  float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_MEASURE_NOISE};
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

static void YawMotorCallback() {
  chassis.yaw_motor_.Update();
}

// 电机初始化
void Chassis::MotorInit() {
  l_wheel_.Init(&hcan2, 0x141);
  r_wheel_.Init(&hcan2, 0x142);
  yaw_motor_.Init(0x206, &hcan1, ABSOLUTE_FLAG);
  yaw_motor_.SetOffest(6812);
  SetTargetYaw(6812);
  lf_joint_.Init(&huart2, 0x01, 10, k_lf_joint_bias);
  lb_joint_.Init(&huart2, 0x00, 10, k_lb_joint_bias);
  rf_joint_.Init(&huart1, 0x01, 10, k_rf_joint_bias);
  rb_joint_.Init(&huart1, 0x00, 10, k_rb_joint_bias);

  l_wheel_.p_motor_instance_->pCanCallBack = LeftWheelCallback;
  r_wheel_.p_motor_instance_->pCanCallBack = RightWheelCallback;
  yaw_motor_.pdji_motor_instance->pCanCallBack = YawMotorCallback;
}

// PID参数初始化
void Chassis::PidInit() {
  left_leg_len_.Init(800.0f, 0.0f, 20.0f, 60.0f, 0.0001f);
  right_leg_len_.Init(800.0f, 0.0f, 20.0f, 60.0f, 0.0001f);
  anti_crash_.Init(8.0f, 0.0f, 2.0f, 10.0f, 0.001f);
  roll_ctrl_.Init(500.0f, 0.0f, 0.0f, 15.0f, 0.001f);
  yaw_pos_.Init(8.0f, 0.0f, 0.0f, 5.0f, 0.001f);
  yaw_speed_.Init(16.0f, 0.0f, 0.0f, 20.0f, 0.0f);
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

// 腿部状态计算
void Chassis::LegCalc() {
  left_leg_.SetBodyData(INS.Pitch * DEGREE_2_RAD, INS.MotionAccel_n[Z]);
  right_leg_.SetBodyData(INS.Pitch * DEGREE_2_RAD, INS.MotionAccel_n[Z]);
  left_leg_.SetLegData(lb_joint_.GetAngle() + k_phi1_bias, lb_joint_.GetSpeed(),
                       lf_joint_.GetAngle() + k_phi4_bias, lf_joint_.GetSpeed(),
                       lb_joint_.GetTor(), lf_joint_.GetTor());
  right_leg_.SetLegData(
      -rb_joint_.GetAngle() + k_phi1_bias, -rb_joint_.GetSpeed(),
      -rf_joint_.GetAngle() + k_phi4_bias, -rf_joint_.GetSpeed(),
      -rb_joint_.GetTor(), -rf_joint_.GetTor());

  left_leg_.LegCalc();
  right_leg_.LegCalc();
  left_leg_.Jacobian();
  right_leg_.Jacobian();
  left_leg_.LegForceCalc();
  right_leg_.LegForceCalc();
}

void Chassis::LQRCalc() {
  lqr_left_.SetSpeed(target_speed_);
  lqr_right_.SetSpeed(target_speed_);

  if (fabsf(vel_) < 0.1f) {
    dist_ += vel_ * controller_dt_;
  } else {
    dist_ = 0.0f;
  }

  lqr_left_.SetData(dist_, vel_, INS.Pitch * DEGREE_2_RAD, INS.Gyro[X],
                    left_leg_.GetTheta(), left_leg_.GetDotTheta(),
                    left_leg_.GetLegLen(), left_leg_.GetForceNormal());
  lqr_right_.SetData(dist_, vel_, INS.Pitch * DEGREE_2_RAD, INS.Gyro[X],
                     right_leg_.GetTheta(), right_leg_.GetDotTheta(),
                     right_leg_.GetLegLen(), right_leg_.GetForceNormal());

  lqr_left_.Calc();
  lqr_right_.Calc();
}

void Chassis::TorCalc() {
  left_leg_.SetTor(left_leg_F_, left_leg_T_);
  right_leg_.SetTor(right_leg_F_, right_leg_T_);
  left_leg_.TorCalc();
  right_leg_.TorCalc();
}

void Chassis::LegLenCalc() {
  left_leg_len_.SetMeasure(left_leg_.GetLegLen());
  right_leg_len_.SetMeasure(right_leg_.GetLegLen());

  roll_comp = k_roll_extra_comp_p * INS.Roll * DEGREE_2_RAD;
  left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp - roll_comp;
  right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp + roll_comp;
}

void Chassis::SynthesizeMotion() {
  yaw_pos_.SetRef((ang_yaw_ / 8192.0f) * 2 * PI);
  yaw_pos_.SetMeasure((target_yaw_ / 8192.0f) * 2 * PI);
  yaw_speed_.SetRef(yaw_pos_.Calculate());
  yaw_speed_.SetMeasure(INS.Gyro[Z]);
  yaw_speed_.Calculate();

  if (left_leg_.GetForceNormal() < 20.0f) {
    l_wheel_T_ = lqr_left_.GetWheelTor();
  } else {
    l_wheel_T_ = lqr_left_.GetWheelTor() - yaw_speed_.GetOutput();
  }
  if (right_leg_.GetForceNormal() < 20.0f) {
    r_wheel_T_ = lqr_right_.GetWheelTor();
  } else {
    r_wheel_T_ = lqr_right_.GetWheelTor() + yaw_speed_.GetOutput();
  }

  anti_crash_.SetRef(0.0f);
  anti_crash_.SetMeasure(left_leg_.GetPhi0() - right_leg_.GetPhi0());
  anti_crash_.Calculate();
  left_leg_T_ = lqr_left_.GetLegTor() + anti_crash_.GetOutput();
  right_leg_T_ = lqr_right_.GetLegTor() - anti_crash_.GetOutput();
}

void Chassis::Controller() {
  SetState();
  LegCalc();
  SpeedCalc();
  LQRCalc();
  SynthesizeMotion();
  if (jump_state_ == true)
    Jump();
  else
    LegLenCalc();
  TorCalc();
}

// 电机力矩输入模式
void Chassis::SetMotorTor() {
  lf_joint_.SetMotorT(left_leg_.GetT2());
  lb_joint_.SetMotorT(left_leg_.GetT1());
  rf_joint_.SetMotorT(-right_leg_.GetT2());
  rb_joint_.SetMotorT(-right_leg_.GetT1());
  l_wheel_.SetTor(l_wheel_T_);
  r_wheel_.SetTor(-r_wheel_T_);
}

// 电机急停模式
void Chassis::StopMotor() {
  lf_joint_.SetMotorT(0.0f);
  rf_joint_.SetMotorT(0.0f);
  lb_joint_.SetMotorT(0.0f);
  rb_joint_.SetMotorT(0.0f);
  l_wheel_.SetTor(0.0f);
  r_wheel_.SetTor(0.0f);
}

void Chassis::SetLegLen() {
  if (fabsf(INS.Pitch) < 10.0f) {
    if (board_comm.GetLongLenFlag() && !board_comm.GetShortLenFlag()) {
      left_leg_len_.SetRef(0.3f);
      right_leg_len_.SetRef(0.3f);
    } else if (!board_comm.GetLongLenFlag() && board_comm.GetShortLenFlag()) {
      left_leg_len_.SetRef(0.08f);
      right_leg_len_.SetRef(0.08f);
    } else {
      left_leg_len_.SetRef(0.18f);
      right_leg_len_.SetRef(0.18f);
    }
  } else {
    left_leg_len_.SetRef(0.08f);
    right_leg_len_.SetRef(0.08f);
  }
}

void Chassis::SetFollow() {
  if (fabsf(board_comm.GetYSpeed()) > 0.0f && side_flag_ == 1) {
    side_flag_ = 0;
  }

  if (fabsf(board_comm.GetXSpeed()) > 0.0f && side_flag_ == 0) {
    side_flag_ = 1;
  }

  if (side_flag_ == 0) {
    if (arm_cos_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) >= 0.0f){
      SetTargetYaw(6812);
    } else {
      SetTargetYaw(2716);
    }
  } else {
    if (arm_sin_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) >= 0.0f) {
      SetTargetYaw(668);
    } else {
      SetTargetYaw(4764);
    }
  }

  ang_yaw_ = yaw_motor_.GetEncode();
  if (ang_yaw_ - target_yaw_ > 4096) {
    ang_yaw_ -= 8192;
  } else if (ang_yaw_ - target_yaw_ < -4096) {
    ang_yaw_ += 8192;
  }
}

void Chassis::SetSpd() {
  if (board_comm.GetCapFlag()) {
    set_spd_ = 2.0f;
  } else {
    set_spd_ = 1.5f;
  }
}

void Chassis::SetState() {
  controller_dt_ = DWT_GetDeltaT(&dwt_cnt_controller_);

  SetLegLen();
  SetFollow();
  SetSpd();
  float y_spd_ = +arm_cos_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) *
                     board_comm.GetYSpeed() +
                 arm_sin_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) *
                     board_comm.GetXSpeed();

  if (fabsf((y_spd_ / 2000.0f) * set_spd_ - target_speed_) / controller_dt_ <
      5.0f) {
    target_speed_ = (y_spd_ / 2000.0f) * set_spd_;
  } else {
    target_speed_ += Math::Sign((y_spd_ / 2000.0f) * set_spd_ - target_speed_) *
                     5.0f * controller_dt_;
  }
  if (board_comm.GetReadyFlag() == 0) {
    lqr_left_.SetNowDist(0.0f);
    lqr_right_.SetNowDist(0.0f);
  }

  // if (remote.GetS1() == 3) {
  //   ready_jump = 1;
  // }
  // if (remote.GetS1() == 2 && ready_jump == 1) {
  //   jump_state_ = true;
  //   ready_jump = 0;
  // }
}

// 相关功能函数

// 跳跃函数
void Chassis::Jump() {
  if (left_leg_.GetLegLen() > 0.12f && right_leg_.GetLegLen() > 0.12f &&
      last_jump_state_ == false) {
    left_leg_len_.SetRef(0.1f);
    right_leg_len_.SetRef(0.1f);
    roll_comp = k_roll_extra_comp_p * INS.Roll * DEGREE_2_RAD;
    left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp - roll_comp;
    right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp + roll_comp;
    last_jump_state_ = false;
    return;
  } else {
    if (jump_state_ == true && last_jump_state_ == false) {
      jump_start_time_ = HAL_GetTick() / 1000.0f;
    }

    jump_now_time_ = HAL_GetTick() / 1000.0f;

    if (fabs(jump_now_time_ - jump_start_time_) <= k_jump_time) {
      left_leg_F_ = k_jump_force;
      right_leg_F_ = k_jump_force;
    }

    jump_now_time_ = HAL_GetTick() / 1000.0f;
    if ((jump_now_time_ - jump_start_time_ - k_jump_time) <= k_retract_time &&
        (jump_now_time_ - jump_start_time_) > k_jump_time) {
      left_leg_F_ = k_retract_force;
      right_leg_F_ = k_retract_force;
    }
    if ((jump_now_time_ - jump_start_time_ - k_jump_time) > k_retract_time) {
      jump_state_ = false;
    }
    last_jump_state_ = jump_state_;
  }
}

// 速度融合函数
void Chassis::SpeedCalc() {
  left_w_wheel_ = l_wheel_.GetSpeed() + left_leg_.GetPhi2Speed() - INS.Gyro[X];
  right_w_wheel_ =
      -r_wheel_.GetSpeed() + right_leg_.GetPhi2Speed() - INS.Gyro[X];

  left_v_body_ = left_w_wheel_ * k_wheel_radius +
                 left_leg_.GetLegLen() * left_leg_.GetDotTheta() +
                 left_leg_.GetLegSpeed() * arm_sin_f32(left_leg_.GetTheta());
  right_v_body_ = right_w_wheel_ * k_wheel_radius +
                  right_leg_.GetLegLen() * right_leg_.GetDotTheta() +
                  right_leg_.GetLegSpeed() * arm_sin_f32(right_leg_.GetTheta());
  vel_m = (left_v_body_ + right_v_body_) / 2;
  if (left_leg_.GetForceNormal() < 20.0f &&
      right_leg_.GetForceNormal() < 20.0f) {
    vel_m = 0;
  }

  // 使用kf同时估计加速度和速度,滤波更新
  kf.MeasuredVector[0] = vel_m;
  kf.MeasuredVector[1] = INS.MotionAccel_n[Y];
  kf.F_data[1] = controller_dt_;  // 更新F矩阵
  Kalman_Filter_Update(&kf);
  vel_ = kf.xhat_data[0];
  acc_ = kf.xhat_data[1];
}
