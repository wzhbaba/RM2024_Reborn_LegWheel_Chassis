/**
 *******************************************************************************
 * @file      : unitree.h
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
#ifndef __UNITREE_H_
#define __UNITREE_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#pragma pack(1)

// 发送用单个数据数据结构
typedef union {
  int32_t L;
  uint8_t u8[4];
  uint16_t u16[2];
  uint32_t u32;
  float F;
} COMData32;

typedef struct {
  // 定义 数据包头
  unsigned char start[2];  // 包头
  unsigned char
      motorID;  // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
  unsigned char reserved;
} COMHead;

#pragma pack()

#pragma pack(1)

typedef struct {
  uint8_t fan_d;   // 关节上的散热风扇转速
  uint8_t Fmusic;  // 电机发声频率   /64*1000   15.625f 频率分度
  uint8_t Hmusic;  // 电机发声强度   推荐值4  声音强度 0.1 分度
  uint8_t reserved4;

  uint8_t FRGB[4];  // 足端LED

} LowHzMotorCmd;

typedef struct {      // 以 4个字节一组排列 ，不然编译器会凑整
                      // 定义 数据
  uint8_t mode;       // 关节模式选择
  uint8_t ModifyBit;  // 电机控制参数修改位
  uint8_t ReadBit;    // 电机控制参数发送位
  uint8_t reserved;

  COMData32 Modify;  // 电机参数修改 的数据
  // 实际给FOC的指令力矩为：
  //  K_P*delta_Pos + K_W*delta_W + T
  int16_t T;  // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
  int16_t W;  // 期望关节速度 （电机本身的速度） x128,       8 + 7描述
  int32_t Pos;  // 期望关节位置 x 16384/6.2832,
                // 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

  int16_t K_P;  // 关节刚度系数 x2048  4+11 描述
  int16_t K_W;  // 关节速度系数 x1024  5+10 描述

  uint8_t LowHzMotorCmdIndex;  // 电机低频率控制命令的索引, 0-7,
                               // 分别代表LowHzMotorCmd中的8个字节
  uint8_t LowHzMotorCmdByte;   // 电机低频率控制命令的字节

  COMData32 Res[1];  // 通讯 保留字节  用于实现别的一些通讯内容

} MasterComdV3;  // 加上数据包的包头 和CRC 34字节

typedef struct {
  // 定义 电机控制命令数据包
  COMHead head;
  MasterComdV3 Mdata;
  uint32_t CRC32;
} MasterComdDataV3;  // 返回数据

// typedef struct {
// 	// 定义 总得485 数据包

//   MasterComdData M1;
// 	MasterComdData M2;
// 	MasterComdData M3;

// }DMA485TxDataV3;

#pragma pack()

#pragma pack(1)

typedef struct {  // 以 4个字节一组排列 ，不然编译器会凑整
  // 定义 数据
  uint8_t mode;     // 当前关节模式
  uint8_t ReadBit;  // 电机控制参数修改     是否成功位
  int8_t Temp;      // 电机当前平均温度
  uint8_t MError;   // 电机错误 标识

  COMData32 Read;  // 读取的当前 电机 的控制数据
  int16_t T;       // 当前实际电机输出力矩       7 + 8 描述

  int16_t W;  // 当前实际电机速度（高速）   8 + 7 描述
  float LW;   // 当前实际电机速度（低速）

  int16_t W2;  // 当前实际关节速度（高速）   8 + 7 描述
  float LW2;   // 当前实际关节速度（低速）

  int16_t Acc;     // 电机转子加速度       15+0 描述  惯量较小
  int16_t OutAcc;  // 输出轴加速度         12+3 描述  惯量较大

  int32_t Pos;  // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
  int32_t Pos2;  // 关节编码器位置(输出编码器)

  int16_t gyro[3];  // 电机驱动板6轴传感器数据
  int16_t acc[3];

  // 力传感器的数据
  int16_t Fgyro[3];  //
  int16_t Facc[3];
  int16_t Fmag[3];
  uint8_t Ftemp;  // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率

  int16_t Force16;  // 力传感器高16位数据
  int8_t Force8;    // 力传感器低8位数据

  uint8_t FError;  //  足端传感器错误标识

  int8_t Res[1];  // 通讯 保留字节

} ServoComdV3;  // 加上数据包的包头 和CRC 78字节（4+70+4）

typedef struct {
  // 定义 电机控制命令数据包
  COMHead head;
  ServoComdV3 Mdata;

  uint32_t CRCdata;

} ServoComdDataV3;  // 发送数据

// typedef struct {
// 	// 定义 总的485 接受数据包

//   ServoComdDataV3 M[3];
//  // uint8_t  nullbyte1;

// }DMA485RxDataV3;

#pragma pack()

typedef struct {
  MasterComdDataV3 ComData;
  int hex_len;          // 发送的16进制命令数组长度, 34
  long long send_time;  // 发送该命令的时间, 微秒(us)
  // 待发送的各项数据
  unsigned short id;    // 电机ID，0代表全部电机
  unsigned short mode;  // 0:空闲, 5:开环转动, 10:闭环FOC控制
  // 实际给FOC的指令力矩为：
  //  K_P*delta_Pos + K_W*delta_W + T
  float T;    // 期望关节的输出力矩（电机本身的力矩）（Nm）
  float W;    // 期望关节速度（电机本身的速度）(rad/s)
  float Pos;  // 期望关节位置（rad）
  float K_P;  // 关节刚度系数
  float K_W;  // 关节速度系数
  COMData32 Res;  // 通讯 保留字节  用于实现别的一些通讯内容

} MOTOR_send;

typedef struct {
  // 定义 接收数据
  ServoComdDataV3 ServoData;  // 电机接收数据结构体，详见motor_msg.h
  int hex_len;                // 接收的16进制命令数组长度, 78
  long long resv_time;        // 接收该命令的时间, 微秒(us)
  int correct;  // 接收数据是否完整（1完整，0不完整）
  // 解读得出的电机数据
  unsigned char motor_id;  // 电机ID
  unsigned char mode;      // 0:空闲, 5:开环转动, 10:闭环FOC控制
  int Temp;                // 温度
  unsigned char MError;    // 错误码
  float T;                 // 当前实际电机输出力矩
  float W;                 // speed
  float Pos;  // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
  float footForce;  // 足端气压传感器数据 12bit (0-4095)
  float jointAngle;
} MOTOR_recv;

class Unitree_Motor {
 public:
  void Init(UART_HandleTypeDef* _p_huart, uint8_t _id, uint8_t _mode, float _ang_bias);
  int SendData();
  void SetMotorData(float _Pos, float _T, float _W, float _K_P, float _K_W);
  void SetMotorPos(float _Pos);
  void SetMotorT(float _T);
  int Update(uint8_t* pData);
  float GetAngle();
  float GetSpeed();
  float GetTor();
  void Ctrl();
  void Receive();

 private:
  UART_HandleTypeDef* p_huart_;
  uint8_t id_, mode_;
  GPIO_TypeDef* p_port_;
  uint16_t pin_;
  MOTOR_send motor_send_;
  MOTOR_recv motor_recv_;
  float ang_bias_;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif
#endif /* __UNITREE_H_ */
