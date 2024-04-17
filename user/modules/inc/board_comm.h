/**
 *******************************************************************************
 * @file      : board_comm.h
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
#ifndef __BOARD_COMM_H_
#define __BOARD_COMM_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#pragma pack(1)
typedef struct {
    int16_t x_speed;
    int16_t y_speed;
    int16_t w_speed;
    uint8_t shoot_flag : 1;
    uint8_t fric_flag : 1;
    uint8_t aim_flag : 1;
    uint8_t rotate_flag : 1;
    uint8_t cap_flag : 1;
    uint8_t stop_flag : 1;
    uint8_t side_flag : 1;
} rece_pack;

typedef struct {
    uint16_t cooling_heat;
    uint16_t cooling_limit;
    uint16_t shooter_output;
    uint16_t robot_id;
} send_pack;

#pragma pack()

class BoardComm
{
   public:
    int16_t GetXSpeed()
    {
        return rece_.x_speed;
    };
    int16_t GetYSpeed()
    {
        return rece_.y_speed;
    };
    int16_t GetWSpeed()
    {
        return rece_.w_speed;
    };
    uint8_t GetShootFlag()
    {
        return rece_.shoot_flag;
    };
    uint8_t GetFricFlag()
    {
        return rece_.fric_flag;
    };
    uint8_t GetAimFlag()
    {
        return rece_.aim_flag;
    };
    uint8_t GetRotateFlag()
    {
        return rece_.rotate_flag;
    };
    uint8_t GetCapFlag()
    {
        return rece_.cap_flag;
    };
    uint8_t GetReadyFlag()
    {
        return rece_.stop_flag;
    };
    void SetCoolingHeat(uint16_t _data)
    {
        send_.cooling_heat = _data;
    };
    void SetCoolingLimit(uint16_t _data)
    {
        send_.cooling_limit = _data;
    };
    void SetShooterOutput(uint16_t _data)
    {
        send_.shooter_output = _data;
    };
    void SetRobotID(uint16_t _data)
    {
        send_.robot_id = _data;
    }
    void Send();
    void Receive();
    void Init(CAN_HandleTypeDef *_phcan, uint16_t _id);

   private:
    CanInstance *p_instance_;
    send_pack send_;
    rece_pack rece_;
};

/* Exported variables --------------------------------------------------------*/
extern BoardComm board_comm;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __BOARD_COMM_H_ */
