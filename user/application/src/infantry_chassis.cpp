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
#include "client_ui.h"
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
bool ui_init = false;
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
    if (cnt > 3500) {
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

void UITask() {
  if (ui_init == false || board_comm.GetRefreshIDFlag()) {
    UI.Draw_String(&UI.UI_String[0].String, (char*)"110", UI_Graph_ADD, 0,
                   UI_Color_White, 20, 22, 2, 740, 874,
                   (char*)"SHOOT  FRIC  RFID  CAP");
    UI.PushUp_String(&UI.UI_String[0], referee.game_robot_state_.robot_id);
    osDelay(60);
    UI.Draw_String(&UI.UI_String[1].String, (char*)"111", UI_Graph_ADD, 0,
                   UI_Color_Main, 20, 6, 2, 129, 753, (char*)"BULLET");
    UI.PushUp_String(&UI.UI_String[1], referee.game_robot_state_.robot_id);
    osDelay(60);
    UI.Draw_String(&UI.UI_String[2].String, (char*)"112", UI_Graph_ADD, 0,
                   UI_Color_Yellow, 20, 3, 2, 1454, 800, (char*)"CAP");
    UI.PushUp_String(&UI.UI_String[2], referee.game_robot_state_.robot_id);
    osDelay(60);
    UI.Draw_Circle(&UI.UI_Graph1[0].Graphic[0], (char*)"113", UI_Graph_ADD, 2,
                   UI_Color_Green, 4, 737, 124, 70);
    UI.PushUp_Graphs(1, &UI.UI_Graph1[0], referee.game_robot_state_.robot_id);
    osDelay(60);
    UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[0], (char*)"100", UI_Graph_ADD,
                      1, UI_Color_Green, 5, 872, 840, 963, 885);
    UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[1], (char*)"101", UI_Graph_ADD,
                      1, UI_Color_Green, 5, 991, 840, 1080, 885);
    UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[2], (char*)"102", UI_Graph_ADD,
                      1, UI_Color_Green, 5, 1103, 840, 1192, 885);
    UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[3], (char*)"103", UI_Graph_ADD,
                      1, UI_Color_Green, 5, 731, 840, 842, 885);
    UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[4], (char*)"104", UI_Graph_ADD,
                      1, UI_Color_Orange, 5, 114, 720, 257, 760);
    UI.PushUp_Graphs(5, &UI.UI_Graph5[0], referee.game_robot_state_.robot_id);
    osDelay(60);

    UI.Draw_Line(&UI.UI_Graph7[0].Graphic[0], (char*)"120", UI_Graph_ADD, 0,
                 UI_Color_Yellow, 3, 1200, 220, 1340, 220);
    UI.Draw_Line(&UI.UI_Graph7[0].Graphic[1], (char*)"121", UI_Graph_ADD, 0,
                 UI_Color_Yellow, 3, 1200, 160, 1340, 160);
    UI.Draw_Line(&UI.UI_Graph7[0].Graphic[2], (char*)"122", UI_Graph_ADD, 0,
                 UI_Color_Yellow, 3, 1200, 100, 1340, 100);
    UI.Draw_Line(&UI.UI_Graph7[0].Graphic[3], (char*)"123", UI_Graph_ADD, 0,
                 UI_Color_Yellow, 3, 1200, 40, 1340, 40);
    UI.Draw_Line(&UI.UI_Graph7[0].Graphic[4], (char*)"124", UI_Graph_ADD, 0,
                 UI_Color_Pink, 30, 1240, 40, 1240, 40);
    UI.Draw_Line(&UI.UI_Graph7[0].Graphic[5], (char*)"125", UI_Graph_ADD, 0,
                 UI_Color_Cyan, 30, 1300, 40, 1300, 40);
    UI.Draw_Rectangle(&UI.UI_Graph7[0].Graphic[6], (char*)"126", UI_Graph_ADD,
                      1, UI_Color_White, 3, 557, 289, 1355, 769);
    UI.PushUp_Graphs(7, &UI.UI_Graph7[0], referee.game_robot_state_.robot_id);
    osDelay(60);
    UI.Draw_Line(&UI.UI_Graph2[0].Graphic[0], (char*)"130", UI_Graph_ADD, 3,
                 UI_Color_Purplish_red, 4, 667, 124, 807, 124);
    UI.Draw_Line(&UI.UI_Graph2[0].Graphic[1], (char*)"131", UI_Graph_ADD, 3,
                 UI_Color_Pink, 4, 737, 124, 737, 194);
    UI.PushUp_Graphs(2, &UI.UI_Graph2[0], referee.game_robot_state_.robot_id);
    osDelay(60);
    UI.Draw_Line(&UI.UI_Graph1[1].Graphic[0], (char*)"132", UI_Graph_ADD, 3,
                 UI_Color_Green, 30, 1540, 790, 1540, 790);
    UI.PushUp_Graphs(1, &UI.UI_Graph1[1], referee.game_robot_state_.robot_id);
    osDelay(60);
    ui_init = true;
  } else {
    int16_t cap_volt = 1540 + cap.GetVolt() / 10;
    UI.Draw_Line(&UI.UI_Graph1[1].Graphic[0], (char*)"132", UI_Graph_Change, 3,
                 UI_Color_Green, 30, 1540, 790, cap_volt, 790);
    UI.PushUp_Graphs(1, &UI.UI_Graph1[1], referee.game_robot_state_.robot_id);
    osDelay(40);
    int16_t left_len = 40 + (chassis.left_leg_.GetLegLen() / 0.3f) * 180;
    int16_t right_len = 40 + (chassis.right_leg_.GetLegLen() / 0.3f) * 180;
    UI.Draw_Line(&UI.UI_Graph7[0].Graphic[4], (char*)"124", UI_Graph_Change, 0,
                 UI_Color_Pink, 30, 1240, 40, 1240, left_len);
    UI.Draw_Line(&UI.UI_Graph7[0].Graphic[5], (char*)"125", UI_Graph_Change, 0,
                 UI_Color_Cyan, 30, 1300, 40, 1300, right_len);
    if (board_comm.GetAimFlag()) {
      if (board_comm.GetEnergy() == 1) {
        UI.Draw_Rectangle(&UI.UI_Graph7[0].Graphic[6], (char*)"126",
                          UI_Graph_Change, 1, UI_Color_Green, 3, 557, 289, 1355,
                          769);
      } else if (board_comm.GetEnergy() == 3) {
        UI.Draw_Rectangle(&UI.UI_Graph7[0].Graphic[6], (char*)"126",
                          UI_Graph_Change, 1, UI_Color_Yellow, 3, 557, 289, 1355,
                          769);
      } else if (board_comm.GetEnergy() == 4) {
        UI.Draw_Rectangle(&UI.UI_Graph7[0].Graphic[6], (char*)"126",
                          UI_Graph_Change, 1, UI_Color_Pink, 3, 557, 289, 1355,
                          769);
      }
    } else {
      UI.Draw_Rectangle(&UI.UI_Graph7[0].Graphic[6], (char*)"126",
                        UI_Graph_Change, 1, UI_Color_White, 3, 557, 289, 1355,
                        769);
    }
    UI.PushUp_Graphs(7, &UI.UI_Graph7[0], referee.game_robot_state_.robot_id);
    osDelay(40);
    if (board_comm.GetFricFlag() == 1) {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[0], (char*)"100", UI_Graph_ADD,
                        1, UI_Color_Green, 5, 872, 840, 963, 885);
    } else {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[0], (char*)"100", UI_Graph_Del,
                        1, UI_Color_Green, 5, 872, 840, 963, 885);
    }
    if (board_comm.GetCapFlag()) {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[2], (char*)"102", UI_Graph_ADD,
                        1, UI_Color_Green, 5, 1103, 840, 1192, 885);
    } else {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[2], (char*)"102", UI_Graph_Del,
                        1, UI_Color_Green, 5, 1103, 840, 1192, 885);
    }
    if (referee.rfid_status_.rfid_status > 0) {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[1], (char*)"101", UI_Graph_ADD,
                        1, UI_Color_Green, 5, 991, 840, 1080, 885);
    } else {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[1], (char*)"101", UI_Graph_Del,
                        1, UI_Color_Green, 5, 991, 840, 1080, 885);
    }
    if (board_comm.GetShootFlag()) {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[3], (char*)"103",
                        UI_Graph_Change, 1, UI_Color_Purplish_red, 5, 731, 840,
                        842, 885);
    } else {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[3], (char*)"103",
                        UI_Graph_Change, 1, UI_Color_Green, 5, 731, 840, 842,
                        885);
    }
    if (referee.projectile_allowance.projectile_allowance_17mm <= 10) {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[4], (char*)"104", UI_Graph_ADD,
                        1, UI_Color_Orange, 5, 114, 720, 257, 760);
    } else {
      UI.Draw_Rectangle(&UI.UI_Graph5[0].Graphic[4], (char*)"104", UI_Graph_Del,
                        1, UI_Color_Orange, 5, 114, 720, 257, 760);
    }
    UI.PushUp_Graphs(5, &UI.UI_Graph5[0], referee.game_robot_state_.robot_id);
    osDelay(40);
    int16_t sin_theta =
        arm_sin_f32(chassis.yaw_motor_.GetAngle() * DEGREE_2_RAD) * 70;
    int16_t cos_theta =
        arm_cos_f32(chassis.yaw_motor_.GetAngle() * DEGREE_2_RAD) * 70;
    int16_t sin_theta_90 =
        arm_sin_f32((chassis.yaw_motor_.GetAngle() + 90.0f) * DEGREE_2_RAD) *
        70;
    int16_t cos_theta_90 =
        arm_cos_f32((chassis.yaw_motor_.GetAngle() + 90.0f) * DEGREE_2_RAD) *
        70;
    UI.Draw_Line(&UI.UI_Graph2[0].Graphic[0], (char*)"130", UI_Graph_Change, 3,
                 UI_Color_Purplish_red, 4, 737 + sin_theta_90,
                 124 + cos_theta_90, 737 - sin_theta_90, 124 - cos_theta_90);
    UI.Draw_Line(&UI.UI_Graph2[0].Graphic[1], (char*)"131", UI_Graph_Change, 3,
                 UI_Color_Pink, 4, 737, 124, 737 + sin_theta, 124 + cos_theta);
    UI.PushUp_Graphs(2, &UI.UI_Graph2[0], referee.game_robot_state_.robot_id);
    osDelay(40);
  }
}