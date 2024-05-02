/**
 *******************************************************************************
 * @file      : unitree.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0     2024-01-22    Jerry Gong       1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "unitree.h"

#include "crc.h"
#include "string.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Initializes the Unitree Motor.
 *
 * This function is used to initialize the Unitree Motor with the specified UART
 * handle, ID, and mode.
 *
 * @param _p_huart Pointer to the UART handle.
 * @param _id The ID of the motor.
 * @param _mode The mode of the motor.
 */
void Unitree_Motor::Init(UART_HandleTypeDef* _p_huart, uint8_t _id,
                         uint8_t _mode, float _ang_bias) {
  p_huart_ = _p_huart;
  id_ = _id;
  mode_ = _mode;
  ang_bias_ = _ang_bias;
  if (p_huart_->Instance == USART1) {
    p_port_ = RS485_DIR1_GPIO_Port;
    pin_ = RS485_DIR1_Pin;
  } else if (p_huart_->Instance == USART2) {
    p_port_ = RS485_DIR2_GPIO_Port;
    pin_ = RS485_DIR2_Pin;
  }
}

/**
 * @brief Sets the motor data for the Unitree_Motor class.
 *
 * @param _Pos The position value.
 * @param _T The torque value.
 * @param _W The angular velocity value.
 * @param _K_P The proportional gain value.
 * @param _K_W The angular velocity gain value.
 */
void Unitree_Motor::SetMotorData(float _Pos, float _T, float _W, float _K_P,
                                 float _K_W) {
  motor_send_.id = id_;
  motor_send_.mode = mode_;
  motor_send_.Pos = _Pos;
  motor_send_.T = _T;
  motor_send_.W = _W;
  motor_send_.K_P = _K_P;
  motor_send_.K_W = _K_W;
}

/**
 * @brief Sets the position of the motor.
 *
 * @param _Pos The desired position of the motor.
 */
void Unitree_Motor::SetMotorPos(float _Pos) {
  SetMotorData(_Pos, 0, 0, 0.2f, 3.0f);
}

/**
 * @brief Sets the motor torque.
 *
 * This function is used to set the torque of the motor.
 *
 * @param _T The torque value to be set.
 */
void Unitree_Motor::SetMotorT(float _T) {
  SetMotorData(0, Math::AbsLimit(_T, 30.f) / 9.1f, 0, 0, 1.0f);
}

/**
 * @brief Sends data from the Unitree_Motor class.
 *
 * This function is responsible for sending data from the Unitree_Motor class.
 * It performs the necessary operations to send the data.
 *
 * @return void
 */
int Unitree_Motor::SendData() {
  motor_send_.ComData.head.start[0] = 0xFE;
  motor_send_.ComData.head.start[1] = 0xEE;
  motor_send_.ComData.head.motorID = motor_send_.id;
  motor_send_.ComData.Mdata.mode = motor_send_.mode;
  motor_send_.ComData.Mdata.ModifyBit = 0xFF;
  motor_send_.ComData.Mdata.Pos = (motor_send_.Pos / 6.2832f) * 16384;
  motor_send_.ComData.Mdata.T = motor_send_.T * 256;
  motor_send_.ComData.Mdata.W = motor_send_.W * 128;
  motor_send_.ComData.Mdata.K_P = motor_send_.K_P * 2048;
  motor_send_.ComData.Mdata.K_W = motor_send_.K_W * 1024;
  motor_send_.ComData.CRC32 =
      Get_CRC32_Check_Sum((uint32_t*)&motor_send_.ComData, 7);

  return 0;
}

/**
 * @brief Updates the Unitree Motor.
 *
 * This function is responsible for updating the Unitree Motor based on the
 * provided data.
 *
 * @param pData Pointer to the data used for updating the motor.
 * @return None.
 */
int Unitree_Motor::Update(uint8_t* pData) {
  if (motor_recv_.ServoData.CRCdata ==
      Get_CRC32_Check_Sum((uint32_t*)(pData), 18)) {
    motor_recv_.motor_id = motor_recv_.ServoData.head.motorID;
    motor_recv_.mode = motor_recv_.ServoData.Mdata.mode;
    motor_recv_.Temp = motor_recv_.ServoData.Mdata.Temp;
    motor_recv_.MError = motor_recv_.ServoData.Mdata.MError;
    motor_recv_.W = ((float)motor_recv_.ServoData.Mdata.W / 128) * 0.95f +
                    motor_recv_.W * 0.05f;
    motor_recv_.T = (float)motor_recv_.ServoData.Mdata.T / 256;
    motor_recv_.Pos =
        6.2832f * ((float)motor_recv_.ServoData.Mdata.Pos) / 16384;
  }
  return 0;
}

/**
 * @brief Controls the Unitree motor.
 *
 * This function is responsible for controlling the Unitree motor.
 * It returns the status of the operation.
 *
 * @return HAL_StatusTypeDef The status of the operation.
 */
void Unitree_Motor::Ctrl() {
  SendData();
  HAL_GPIO_WritePin(p_port_, pin_, GPIO_PIN_SET);
  HAL_UART_Transmit(p_huart_, (uint8_t*)&motor_send_.ComData,
                    sizeof(motor_send_.ComData), 10);
  HAL_GPIO_WritePin(p_port_, pin_, GPIO_PIN_RESET);
  HAL_UARTEx_ReceiveToIdle_DMA(p_huart_, (uint8_t*)&motor_recv_.ServoData,
                               sizeof(motor_recv_.ServoData));

  uint8_t* rp = (uint8_t*)&motor_recv_.ServoData;
  if (rp[0] == 0xFE && rp[1] == 0xEE) {
    Update((uint8_t*)&motor_recv_.ServoData);
  }
}

float Unitree_Motor::GetAngle() {
  return (motor_recv_.Pos - ang_bias_) / 9.1f;
}

float Unitree_Motor::GetSpeed() {
  return motor_recv_.W / 9.1f;
}

float Unitree_Motor::GetTor() {
  return motor_recv_.T * 9.1f;
}