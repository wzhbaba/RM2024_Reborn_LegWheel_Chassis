/**
 *******************************************************************************
 * @file      : infantry_chassis.cpp
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
#include "infantry_chassis.h"

#include "board_comm.h"
#include "bsp_dwt.h"
#include "cap.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "referee.h"
#include "remote.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t referee_last_state;
static uint8_t referee_ready;
static uint16_t cnt;
static uint8_t cnt_flag;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void ChassisMotorInit() {
  remote.Init(&huart3);
  referee.Init(&huart5);
  chassis.MotorInit();
  board_comm.Init(&hcan1, 0x101);
  cap.Init(&hcan2, 0x600);
  chassis.PidInit();
  chassis.SpeedEstInit();
}

void ChassisCalcTask() {
  chassis.Controller();
  if (referee.game_robot_state_.power_management_chassis_output == 1 &&
      referee_last_state == 0 && cnt_flag == 0) {
    cnt_flag = 1;
    referee_ready = 0;
  }
  if (cnt_flag) {
    cnt++;
    if (cnt > 1600) {
      cnt_flag = 0;
      referee_ready = 1;
      cnt = 0;
    }
  }
  if (board_comm.GetReadyFlag() == 1 &&
      referee.game_robot_state_.power_management_chassis_output == 1 &&
      referee_ready) {
    chassis.SetMotorTor();
  } else {
    chassis.StopMotor();
  }
  referee_last_state =
      referee.game_robot_state_.power_management_chassis_output;
}

void CapTask() {
  cap.SetPower(referee.game_robot_state_.chassis_power_limit);
  cap.AccessPoll();
  cap.SetCapState();
}

void UnitreeMotorTask() {
  chassis.lf_joint_.Ctrl();
  chassis.rf_joint_.Ctrl();
  DWT_Delay(0.0003f);
  chassis.lb_joint_.Ctrl();
  chassis.rb_joint_.Ctrl();
}

void WheelMotorTask() {
  chassis.l_wheel_.Send();
  chassis.r_wheel_.Send();
}

void CtrlCommTask() {
  board_comm.Send();
}