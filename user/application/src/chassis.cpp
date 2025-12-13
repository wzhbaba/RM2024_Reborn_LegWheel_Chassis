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
#include "user_lib.h"
#include "TOF_middleware.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float k_gravity_comp = 76.5478;
const float k_roll_extra_comp_p = 400.0f;
const float k_wheel_radius = 0.076f;

const float k_phi1_bias = PI + 0.3228859f;
const float k_phi4_bias = -0.3228859f;

const float k_lf_joint_bias = 0.204f;
const float k_lb_joint_bias = 2.628f;
const float k_rf_joint_bias = 4.737f;
const float k_rb_joint_bias = 4.933f;

const float k_jump_force = 220.0f;
const float k_jump_time = 0.2f;
const float k_retract_force = -120.0f;
const float k_retract_time = 0.1f;

static float target_yaw;

/*debug*/
static float ExpectLen = 0.16f;
/*debug*/

/* External variables --------------------------------------------------------*/
Chassis chassis;
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 底盘为右手系
 *
 *    ^ y            左轮     右轮
 *    |               |      |  前
 *    |_____ >  x     |------|
 *    z 轴从屏幕向外    |      |  后
 */

 // 电机选型：宇树A1
 // 电机位置：前1后0

void Chassis::SpeedEstInit() {
  // 速度卡尔曼观测初始化
  Kalman_Filter_Init(&kf, 2, 0, 2);
  float F[4] = { 1, 0.001, 0, 1 };
  float Q[4] = { VEL_PROCESS_NOISE, 0, 0, ACC_MEASURE_NOISE };
  float R[4] = { VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE };
  float P[4] = { 100000, 0, 0, 100000 };    //先验估计协方差P
  float H[4] = { 1, 0, 0, 1 };              //观测矩阵H

  //将矩阵赋值到kf结构体中
  memcpy(kf.F_data, F, sizeof(F));
  memcpy(kf.Q_data, Q, sizeof(Q));
  memcpy(kf.R_data, R, sizeof(R));
  memcpy(kf.P_data, P, sizeof(P));
  memcpy(kf.H_data, H, sizeof(H));
}

/**
 * @brief 左轮Can回调函数接口
 */
static void LeftWheelCallback() {
  chassis.l_wheel_.Update();
}

/**
 * @brief 右轮Can回调函数接口
 */
static void RightWheelCallback() {
  chassis.r_wheel_.Update();
}

/**
 * @brief yaw轴电机Can回调函数接口
 */
static void YawMotorCallback() {
  chassis.yaw_motor_.Update();
}

/**
 * @brief 底盘电机初始化函数
 */
void Chassis::MotorInit() {
  //轮电机初始化
  l_wheel_.Init(&hcan1, 0x142);
  r_wheel_.Init(&hcan1, 0x141);

  //yaw轴电机初始化
  yaw_motor_.Init(0x205, &hcan2, ABSOLUTE_FLAG);
  yaw_motor_.SetOffest(2059);

  //设置初始yaw轴零点
  SetTargetYaw(6812);

  //关节电机初始化
  lf_joint_.Init(&huart2, 0x01, 10, k_lf_joint_bias);
  lb_joint_.Init(&huart2, 0x00, 10, k_lb_joint_bias);
  rf_joint_.Init(&huart1, 0x01, 10, k_rf_joint_bias);
  rb_joint_.Init(&huart1, 0x00, 10, k_rb_joint_bias);

  //给3个使用can线的电机赋回调函数指针
  l_wheel_.p_motor_instance_->pCanCallBack = LeftWheelCallback;
  r_wheel_.p_motor_instance_->pCanCallBack = RightWheelCallback;
  yaw_motor_.pdji_motor_instance->pCanCallBack = YawMotorCallback;
}

// PID参数初始化
void Chassis::PidInit() {
  //变腿高pid
  left_leg_len_.Init(400.0f, 0.0f, 20.0f, 60.0f, 0.0001f);
  right_leg_len_.Init(400.0f, 0.0f, 20.0f, 60.0f, 0.0001f);

  //防劈叉，roll轴补偿
  anti_crash_.Init(16.0f, 0.0f, 2.0f, 20.0f, 0.001f);
  roll_ctrl_.Init(100.0f, 0.0f, 0.0f, 30.0f, 0.001f);

  //yaw轴双环pid
  yaw_pos_.Init(4.0f, 0.0f, 1.0f, 3.0f, 0.001f);
  yaw_speed_.Init(4.0f, 4.0f, 0.0f, 10.0f, 0.0f);

  //pid增强
  left_leg_len_.Inprovement(PID_CHANGING_INTEGRATION_RATE |
                            PID_TRAPEZOID_INTEGRAL | PID_DERIVATIVE_FILTER |
                            PID_DERIVATIVE_ON_MEASUREMENT,
                            0.0f, 0.0f, 0.01f, 0.02f, 0.08f);
  right_leg_len_.Inprovement(
    PID_CHANGING_INTEGRATION_RATE | PID_TRAPEZOID_INTEGRAL |
    PID_DERIVATIVE_FILTER | PID_DERIVATIVE_ON_MEASUREMENT,
    0.0f, 0.0f, 0.01f, 0.02f, 0.08f);
  //微分滤波、梯形积分、变速积分
  anti_crash_.Inprovement(PID_DERIVATIVE_FILTER | PID_TRAPEZOID_INTEGRAL |
                          PID_CHANGING_INTEGRATION_RATE | PID_OUTPUT_FILTER, 0.0f, 3.0f, 0.0f, 0.05f, 0.05f);
  //仅开启微分滤波
  roll_ctrl_.Inprovement(PID_DERIVATIVE_FILTER, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f);
  yaw_pos_.Inprovement(PID_DERIVATIVE_FILTER, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f);
  yaw_speed_.Inprovement(PID_DERIVATIVE_FILTER | PID_INTEGRAL_LIMIT, 2.0f, 0.0f,
                         0.0f, 0.0f, 0.05f);
}

/**
 * @brief 腿部状态计算
 * @note 设置俯仰角，关节电机角度反推腿部各状态，
 */
void Chassis::LegCalc() {
  //设置机体俯仰角,竖直加速度
  left_leg_.SetBodyData(INS.Pitch * DEGREE_2_RAD, INS.MotionAccel_n[Z]);
  right_leg_.SetBodyData(INS.Pitch * DEGREE_2_RAD, INS.MotionAccel_n[Z]);
  //传入关节电机数据
  left_leg_.SetLegData(-lb_joint_.GetAngle() + k_phi1_bias, -lb_joint_.GetSpeed(),
                       -lf_joint_.GetAngle() + k_phi4_bias, -lf_joint_.GetSpeed(),
                       -lb_joint_.GetTor(), -lf_joint_.GetTor());
  right_leg_.SetLegData(
    rb_joint_.GetAngle() + k_phi1_bias, rb_joint_.GetSpeed(),
    rf_joint_.GetAngle() + k_phi4_bias, rf_joint_.GetSpeed(),
    rb_joint_.GetTor(), rf_joint_.GetTor());
  //根据上一句传入的数据进行腿的状态计算
  left_leg_.LegCalc();
  right_leg_.LegCalc();
  //Jacobian矩阵计算
  left_leg_.Jacobian();
  right_leg_.Jacobian();
  //支持力解算
  left_leg_.LegForceCalc();
  right_leg_.LegForceCalc();
}

void Chassis::LQRCalc() {
  //向lqr类中传入速度
  lqr_left_.SetSpeed(target_speed_);
  lqr_right_.SetSpeed(target_speed_);

  //仅当速度小于0.1m/s时计算位移,否则位移清零
  if (fabsf(vel_) < 0.1f) {
    dist_ += vel_ * controller_dt_;
  }
  else {
    dist_ = 0.0f;
  }
  //向lqr类传入数据
  lqr_left_.SetData(dist_, vel_, -(INS.Pitch) * DEGREE_2_RAD, -INS.Gyro[X],
                    -(left_leg_.GetTheta() - 0.04),
                    -(left_leg_.GetDotTheta()),
                    left_leg_.GetLegLen(), left_leg_.GetForceNormal());
  lqr_right_.SetData(dist_, vel_, -(INS.Pitch) * DEGREE_2_RAD, -INS.Gyro[X],
                     -(right_leg_.GetTheta() - 0.04),
                     -(right_leg_.GetDotTheta()),
                     right_leg_.GetLegLen(), right_leg_.GetForceNormal());
  //lqr K增益计算控制量
  lqr_left_.Calc();
  lqr_right_.Calc();
}
/**
 * 计算关节力矩
 */
void Chassis::TorCalc() {
  //将足端力和髋关节力矩传入vmc
  left_leg_.SetTor(left_leg_F_, left_leg_T_);
  right_leg_.SetTor(right_leg_F_, right_leg_T_);

  //利用雅可比矩阵计算两关节力矩提供的力矩
  left_leg_.TorCalc();
  right_leg_.TorCalc();
}


void Chassis::LegLenCalc() {
  //变腿高PID测量值
  left_leg_len_.SetMeasure(left_leg_.GetLegLen());
  right_leg_len_.SetMeasure(right_leg_.GetLegLen());
  //因为和Jump()是二选一，所以两个函数中都有roll轴补偿计算
  roll_comp = k_roll_extra_comp_p * INS.Roll * DEGREE_2_RAD;
  //前馈(重力)+pd+roll轴补偿

  left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp + roll_comp;
  right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp - roll_comp;
}


/**
 * @brief 转向闭环控制(真正的底盘跟随控制)(以yaw轴电机编码值为measure)
 */
void Chassis::SynthesizeMotion() {
  //设置yaw轴位置环PID目标值和观测值
  yaw_pos_.SetRef((ang_yaw_ / 8192.0f) * 2 * PI);         //目标值
  yaw_pos_.SetMeasure((target_yaw_ / 8192.0f) * 2 * PI);  //观测值

  // 云台板子收到小陀螺指令，根据指令进行5rad/s左旋右旋
  if (board_comm.GetLeftRotate() && !board_comm.GetRightRotate()) {
    yaw_speed_.SetRef(5.0f);
  }
  else if (board_comm.GetRightRotate() && !board_comm.GetLeftRotate()) {
    yaw_speed_.SetRef(-5.0f);
  }
  //否则yaw轴速度环设定目标值为位置环PID结果
  else {
    yaw_speed_.SetRef(yaw_pos_.Calculate());
  }

  //速度环测量值为yaw方向角速度
  yaw_speed_.SetMeasure(INS.Gyro[Z]);
  //计算速度环pid
  yaw_speed_.Calculate();

  // /*debug*/
  // yaw_speed_.GetOutput() = 0;
  // /*debug*/

  //仅当腿支持力大于等于20(未离地)，进行旋转控制
  if (left_leg_.GetForceNormal() < 20.0f) {
    l_wheel_T_ = lqr_left_.GetWheelTor();
  }
  else {
    l_wheel_T_ = lqr_left_.GetWheelTor() - yaw_speed_.GetOutput();
  }

  if (right_leg_.GetForceNormal() < 20.0f) {
    r_wheel_T_ = lqr_right_.GetWheelTor();
  }
  else {
    r_wheel_T_ = lqr_right_.GetWheelTor() + yaw_speed_.GetOutput();
  }

  //防劈叉PID
  anti_crash_.SetRef(0.0f);
  anti_crash_.SetMeasure(left_leg_.GetPhi0() - right_leg_.GetPhi0());
  anti_crash_.Calculate();

  //防劈叉处理
  left_leg_T_ = -lqr_left_.GetLegTor() + anti_crash_.GetOutput();
  right_leg_T_ = -lqr_right_.GetLegTor() - anti_crash_.GetOutput();
}

void Chassis::Controller() {
  SetState();   //更新状态
  LegCalc();    //腿部状态计算
  SpeedCalc();  //速度计算
  LQRCalc();    //LQR计算轮力矩、髋关节虚拟力矩
  SynthesizeMotion();   //动作合成，主要是旋转和防劈叉处理

  if (jump_state_ == true)
    Jump();     //跳跃函数
  else
    LegLenCalc(); //腿部虚拟力计算(跳跃以外情况)
  TorCalc();    //VMC计算关节电机力矩
}

// 电机力矩输入模式
void Chassis::SetMotorTor() {

  lf_joint_.SetMotorT(-left_leg_.GetT2());
  lb_joint_.SetMotorT(-left_leg_.GetT1());
  rf_joint_.SetMotorT(right_leg_.GetT2());
  rb_joint_.SetMotorT(right_leg_.GetT1());

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
  anti_crash_.GetOutput() = 0;
}
/**
 * @brief 设置腿长
 *
 * 接受云台控制指令,默认最低腿长，(发送的好像是状态量)
 */
void Chassis::SetLegLen() {
  if (fabsf(INS.Pitch) < 8.0f) {
    /*debug*/
    // ExpectLen += 0.05f * map(remote.GetCh1(), 660.f, -660.f, 0.03f, -0.03f);

    // if (ExpectLen > 0.36f) {
    //   ExpectLen = 0.36f;
    // }
    // if (ExpectLen < 0.1f) {
    //   ExpectLen = 0.1f;
    // }

    // left_leg_len_.SetRef(ExpectLen);
    // right_leg_len_.SetRef(ExpectLen);
    // /*debug*/

    if (board_comm.GetLongLenFlag() && !board_comm.GetShortLenFlag()) {
      left_leg_len_.SetRef(0.3f);
      right_leg_len_.SetRef(0.3f);
    }
    else if (!board_comm.GetLongLenFlag() && board_comm.GetShortLenFlag()) {
      left_leg_len_.SetRef(0.1f);
      right_leg_len_.SetRef(0.1f);
    }
    else {
      left_leg_len_.SetRef(0.12f);
      right_leg_len_.SetRef(0.12f);
    }
  }
  else {
    left_leg_len_.SetRef(0.08f);
    right_leg_len_.SetRef(0.08f);
  }
}


/**
 * @brief 底盘跟随
 */
void Chassis::SetFollow() {

  //侧向判断
  if (fabsf(board_comm.GetYSpeed()) > 0.0f && side_flag_ == 1) {
    side_flag_ = 0;
  }

  if (fabsf(board_comm.GetXSpeed()) > 0.0f && side_flag_ == 0) {
    side_flag_ = 1;
  }

  if (side_flag_ == 0) {    //正向对敌
    if (arm_cos_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) >= 0.0f) {
      SetTargetYaw(2059);
    }
    else {
      SetTargetYaw(6155);
    }
  }
  else {      //侧向对敌
    if (arm_sin_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) >= 0.0f) {
      SetTargetYaw(4107);
    }
    else {
      SetTargetYaw(11);
    }
  }


  //更新旋转pid的ref
  ang_yaw_ = yaw_motor_.GetEncode();
  if (ang_yaw_ - target_yaw_ > 4096) {
    ang_yaw_ -= 8192;
  }
  else if (ang_yaw_ - target_yaw_ < -4096) {
    ang_yaw_ += 8192;
  }
}
/**
 * @brief 设置速度增益
 */
void Chassis::SetSpd() {
  if (board_comm.GetCapFlag()) {
    set_spd_gain_ = 2.5f;
  }
  else {
    set_spd_gain_ = 1.5f;
  }
}
/**
 * @brief 设置底盘状态(直译)
 *
 */
void Chassis::SetState() {
  controller_dt_ = DWT_GetDeltaT(&dwt_cnt_controller_);   //获取控制周期

  SetLegLen();
  SetFollow();
  SetSpd();

  //y轴速度(前进后退速度)   
  volatile float y_spd_ = +arm_cos_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) *
    board_comm.GetYSpeed() +
    arm_sin_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) *
    board_comm.GetXSpeed();

  //y_spd_ = board_comm.GetYSpeed() * 0.5f;

  // y_spd_ = map(remote.GetCh3(), 660, -660, 1000, -1000);

  if (fabsf((y_spd_ / 2000.0f) * set_spd_gain_ - target_speed_) / controller_dt_ <
      5.0f) {
    //当期望加速度小于5,直接设定速度
    target_speed_ = (y_spd_ / 2000.0f) * set_spd_gain_;
  }
  else {
    target_speed_ += Math::Sign((y_spd_ / 2000.0f) * set_spd_gain_ - target_speed_) *
      4.f * controller_dt_;    //当期望加速度大于5时,作加速度为4的匀加速运动
  }
  if (board_comm.GetReadyFlag() == 0) {
    //如果云台未发送准备信号,位移设置为0
    lqr_left_.SetNowDist(0.0f);
    lqr_right_.SetNowDist(0.0f);
  }

  if (board_comm.GetJumpFlag()) {
    //跳跃标志位置1
    jump_state_ = true;
  }

  jump_dist_ = tof.GetDistance();
}

// 相关功能函数

// 跳跃函数
// 感觉可以改为达妙例程的三状态自动机
void Chassis::Jump() {
  //当两腿长度大于12cm，且上次未进行跳跃，先变换到最低腿高
  if (left_leg_.GetLegLen() > 0.12f && right_leg_.GetLegLen() > 0.12f &&
      last_jump_state_ == false) {

    //设置腿长pid目标值为10cm
    left_leg_len_.SetRef(0.1f);
    right_leg_len_.SetRef(0.1f);
    roll_comp = k_roll_extra_comp_p * INS.Roll * DEGREE_2_RAD;    //roll轴补偿,P控制器
    //计算虚拟力，重力前馈加变腿长pd，改进开了微分滤波,微分先行，增加roll轴补偿
    left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp + roll_comp;
    right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp - roll_comp;
    // 更新上次跳跃状态
    last_jump_state_ = false;
    return;
  }
  else {
    //正在跳跃且上次没跳
    if (jump_state_ == true && last_jump_state_ == false) {
      jump_start_time_ = HAL_GetTick() / 1000.0f;
    }

    jump_now_time_ = HAL_GetTick() / 1000.0f;
    //时间轴0~0.2s
    if (fabs(jump_now_time_ - jump_start_time_) <= k_jump_time) {
      // 设置跳跃力
      left_leg_F_ = k_jump_force;
      right_leg_F_ = k_jump_force;
    }

    jump_now_time_ = HAL_GetTick() / 1000.0f;
    //时间轴0.2s~0.3s
    if ((jump_now_time_ - jump_start_time_ - k_jump_time) <= k_retract_time &&
        (jump_now_time_ - jump_start_time_) > k_jump_time) {
      //设置跳跃力
      left_leg_F_ = k_retract_force;
      right_leg_F_ = k_retract_force;
      l_wheel_T_ = 0.0f;
      r_wheel_T_ = 0.0f;
    }
    //结束跳跃
    if ((jump_now_time_ - jump_start_time_ - k_jump_time) > k_retract_time) {
      jump_state_ = false;
    }
    last_jump_state_ = jump_state_;
  }
}

// 速度融合函数，卡尔曼观测速度，加速度
void Chassis::SpeedCalc() {
  //轮角速度 = 电机反馈角速度 + 前方膝关节角速度 - 机体俯仰角速度 
  left_w_wheel_ = l_wheel_.GetSpeed() + left_leg_.GetPhi2Speed() - INS.Gyro[X];
  right_w_wheel_ = -r_wheel_.GetSpeed() + right_leg_.GetPhi2Speed() - INS.Gyro[X];

  //机体速度 = 轮角速度*轮半径+摆杆与竖直夹角*腿长+变腿长速度在水平方向分量
  left_v_body_ = left_w_wheel_ * k_wheel_radius -
    left_leg_.GetLegLen() * left_leg_.GetDotTheta() +
    left_leg_.GetLegSpeed() * arm_sin_f32(left_leg_.GetTheta());
  right_v_body_ = right_w_wheel_ * k_wheel_radius -
    right_leg_.GetLegLen() * right_leg_.GetDotTheta() +
    right_leg_.GetLegSpeed() * arm_sin_f32(right_leg_.GetTheta());

  //使用两部分平均速度作为卡尔曼滤波观测值
  vel_m = (left_v_body_ + right_v_body_) / 2;

  //离地检测
  if (left_leg_.GetForceNormal() < 20.0f &&
      right_leg_.GetForceNormal() < 20.0f) {
    vel_m = 0;
  }

  // 使用kf同时估计加速度和速度,滤波更新
  kf.MeasuredVector[0] = vel_m;
  kf.MeasuredVector[1] = INS.MotionAccel_n[Y];
  kf.F_data[1] = controller_dt_;  // 更新F矩阵,采用更精确的deltaT
  Kalman_Filter_Update(&kf);      // 卡尔曼滤波
  vel_ = kf.xhat_data[0];   //滤波后速度
  acc_ = kf.xhat_data[1];   //滤波后加速度

  //动态调整噪声参数
  if (fabsf(acc_) > 5.f) {
    kf.R_data[0] = 25;
    kf.R_data[3] = 900;
  }
  else {
    kf.R_data[0] = VEL_MEASURE_NOISE;
    kf.R_data[3] = VEL_MEASURE_NOISE;
  }
}
