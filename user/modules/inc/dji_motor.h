/**
 *******************************************************************************
 * @file      : dji_motor.h
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
#ifndef __DJI_MOTOR_H_
#define __DJI_MOTOR_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
typedef enum {
    RELATIVE_FLAG = 0,
    ABSOLUTE_FLAG = 1,
} Pos_Flag;

/* Exported types ------------------------------------------------------------*/

class DjiMotor
{
   public:
    CanInstance *pdji_motor_instance;
    void Update();
    void Init(uint32_t _idx, CAN_HandleTypeDef *_phcan, uint8_t _init);
    float GetAngle()
    {
        return ang_real_;
    }
    int GetEncode()
    {
        return encode_;
    }
    int GetSpd()
    {
        return spd_;
    }

   private:
    int16_t round_cnt_;
    int16_t encode_;
    int16_t encode_offest_;
    int16_t last_encode_;
    int16_t spd_;
    int16_t tor_;
    int16_t temp_;
    uint8_t init_;
    float ang_real_;
};
/* Exported variables --------------------------------------------------------*/
extern DjiMotor dji_motor;
/* Exported function prototypes ----------------------------------------------*/

void DjiMotorSend(CAN_HandleTypeDef *_phcan, uint32_t _idx, int16_t _data1, int16_t _data2, int16_t _data3, int16_t _data4);

#endif

#endif /* __DJI_MOTOR_H_ */
