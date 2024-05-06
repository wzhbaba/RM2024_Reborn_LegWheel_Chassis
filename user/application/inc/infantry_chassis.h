/**
 *******************************************************************************
 * @file      : infantry_chassis.h
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
#ifndef __INFANTRY_CHASSIS_H_
#define __INFANTRY_CHASSIS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void ChassisMotorInit();
void UnitreeMotorTask();
void WheelMotorTask();
void ChassisCalcTask();
void CtrlCommTask();

#ifdef __cplusplus
}
#endif

#endif /* __INFANTRY_CHASSIS_H_ */
