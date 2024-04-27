/**
 *******************************************************************************
 * @file      : chassis.h
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
#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "leg.h"
#include "lqr.h"
#include "mf9025.h"
#include "pid.h"
#include "unitree.h"
#include "vmc.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define RAD_2_DEGREE 57.2957795f     // 180/pi
#define DEGREE_2_RAD 0.01745329252f  // pi/180

#define CENTER_IMU_W 0.0827f
#define CENTER_IMU_L 0.0f
#define CENTER_IMU_H 0.035f

#define VEL_PROCESS_NOISE 25   // 速度过程噪声
#define VEL_MEASURE_NOISE 800  // 速度测量噪声
// 同时估计加速度和速度时对加速度的噪声
// 更好的方法是设置为动态,当有冲击时/加加速度大时更相信轮速
#define ACC_PROCESS_NOISE 2000  // 加速度过程噪声
#define ACC_MEASURE_NOISE 0.01  // 加速度测量噪声

/* Exported types ------------------------------------------------------------*/

typedef enum {
  CHASSIS_STOP = 0,
  CHASSIS_RESET,
  CHASSIS_NORMAL,
} CHASSIS_MODE;

typedef enum {
  ROBOT_STOP = 0,
  ROBOT_READY,
} Robot_Status_e;

/**
 * @brief The Chassis class represents the chassis of a robot.
 *
 * This class contains various methods and variables related to the chassis of a
 * robot. It provides functionality for setting leg data, LQR data, motor
 * initialization, PID initialization, control, VMC leg calculation, VMC torque
 * calculation, LQR calculation, setting VMC data, leg calculation, setting leg
 * length, speed calculation, and motion synthesis. It also provides methods for
 * retrieving the values of various parameters such as left wheel speed, right
 * wheel speed, left front joint position, right front joint position, left back
 * joint position, and right back joint position.
 */
class Chassis {
 public:
  Unitree_Motor lf_joint_, lb_joint_, rf_joint_, rb_joint_;
  Mf9025 l_wheel_, r_wheel_;
  void Observer();
  void SetLegData();
  void SetLQRData();
  void SetTargetYaw(float _pos);
  void MotorInit();
  void PidInit();
  void Controller();
  void VMCLegCalc();
  void VMCTorCalc();
  void LQRCalc();
  void SetVMCData();
  void LegCalc();
  void SetLegLen();
  void SpeedCalc();
  void SynthesizeMotion();
  void Jump();
  float GetLeftWheel();
  float GetRightWheel();
  float GetLFJoint();
  float GetRFJoint();
  float GetLBJoint();
  float GetRBJoint();
  void SpeedEstInit();
  void SetMotorTor();
  void StopMotor();
  void Reset();
  void Switch();
  void SetState();
  uint8_t GetState() { return robot_state_; }
  uint8_t GetChassisState() { return chassis_state_; }

 private:
  Leg left_leg_, right_leg_;
  Vmc vmc_left_, vmc_right_;
  Lqr lqr_left_, lqr_right_;
  Pid left_leg_len_, right_leg_len_, anti_crash_, roll_ctrl_, yaw_pos_,
      yaw_speed_;
  float left_target_len_, right_target_len_, left_leg_F_, right_leg_F_;
  float l_wheel_T_, r_wheel_T_, left_leg_T_, right_leg_T_;
  float target_speed_, target_dist_, target_yaw_, vel_, dist_, pitch_, acc_;
  float vel_m, left_v_body_, right_v_body_, left_w_wheel_, right_w_wheel_;
  float jump_start_time_, jump_now_time_;
  uint32_t dwt_cnt_;
  bool jump_state_ = false, last_jump_state_ = false;
  float dt_;
  uint8_t robot_state_;
  uint8_t chassis_state_;
};
/* Exported variables --------------------------------------------------------*/
extern Chassis chassis;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __FILE_H_ */
