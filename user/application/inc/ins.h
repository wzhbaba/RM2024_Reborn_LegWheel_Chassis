/**
 *******************************************************************************
 * @file      : INS.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INS_TASK_H
#define __INS_TASK_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bmi088_driver.h"
#include "quaternion_ekf.h"
#include "stdint.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    float q[4];  // 四元数估计值

    float Gyro[3];  // 角速度
    float dGyro[3];
    float Accel[3];          // 加速度
    float MotionAccel_b[3];  // 机体坐标加速度
    float MotionAccel_n[3];  // 绝对系加速度

    float AccelLPF;  // 加速度低通滤波系数
    float DGyroLPF;
    
    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
extern INS_t INS;

void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void INS_Init(void);
void INS_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __INS_TASK_H */
