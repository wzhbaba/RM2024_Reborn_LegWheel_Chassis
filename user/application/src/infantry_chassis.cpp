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
#include "chassis.h"
#include "referee.h"
#include "remote.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void ChassisMotorInit() {
  board_comm.Init(&hcan1, 0x411);
  remote.Init(&huart3);
  referee.Init(&huart5);
  chassis.MotorInit();
  chassis.PidInit();
  chassis.SpeedEstInit();
}

void ChassisObserverTask() {
  chassis.Observer();
}

void ChassisCalcTask() {
  chassis.Controller();
  if (remote.GetS2() == 1) {
    chassis.SetMotorTor();
  } else {
    chassis.StopMotor();
  }
}

void UnitreeMotorTask() {
  chassis.lf_joint_.Ctrl();
  chassis.rf_joint_.Ctrl();
  chassis.lb_joint_.Ctrl();
  chassis.rb_joint_.Ctrl();
}

void WheelMotorTask() {
  chassis.l_wheel_.Send();
  chassis.r_wheel_.Send();
}