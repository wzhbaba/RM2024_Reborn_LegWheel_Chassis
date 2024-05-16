/**
 *******************************************************************************
 * @file      : Client_UI.h
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
#ifndef __CLIENT_UI_H_
#define __CLIENT_UI_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "referee.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* 删除操作 */
typedef enum {
  UI_Data_Del_NoOperate = 0,
  UI_Data_Del_Layer = 1,
  UI_Data_Del_ALL = 2,  // 删除全部图层，后面的参数已经不重要了。
} UI_Delete_Operate_e;

/* 图形配置参数__图形操作 */
typedef enum {
  UI_Graph_ADD = 1,
  UI_Graph_Change = 2,
  UI_Graph_Del = 3,
} UI_Graph_Operate_e;

typedef enum {
  UI_Graph_Line = 0,       // 直线
  UI_Graph_Rectangle = 1,  // 矩形
  UI_Graph_Circle = 2,     // 整圆
  UI_Graph_Ellipse = 3,    // 椭圆
  UI_Graph_Arc = 4,        // 圆弧
  UI_Graph_Float = 5,      // 浮点型
  UI_Graph_Int = 6,        // 整形
  UI_Graph_String = 7,     // 字符型

} UI_Graph_Type_e;

/* 图形配置参数__图形颜色 */
typedef enum {
  UI_Color_Main = 0,  // 红蓝主色
  UI_Color_Yellow = 1,
  UI_Color_Green = 2,
  UI_Color_Orange = 3,
  UI_Color_Purplish_red = 4,  // 紫红色
  UI_Color_Pink = 5,
  UI_Color_Cyan = 6,  // 青色
  UI_Color_Black = 7,
  UI_Color_White = 8,

} UI_Graph_Color_e;

typedef struct  // 绘制UI UI图形数据
{
  uint8_t graphic_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t radius : 10;
  uint32_t end_x : 11;
  uint32_t end_y : 11;
} graphic_data_struct_t;

typedef struct  // 绘制UI UI字符串数据
{
  uint8_t string_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t null;
  uint8_t stringdata[30];
} string_data_struct_t;

typedef struct  // 绘制UI UI删除图形数据
{
  uint8_t operate_tpye;
  uint8_t layer;
} delete_data_struct_t;

typedef struct  // 绘制UI 绘制1个图形完整结构体
{
  xFrameHeader Referee_Transmit_Header;
  uint16_t CMD_ID;
  ext_student_interactive_header_data_t Interactive_Header;
  graphic_data_struct_t Graphic[1];
  uint16_t CRC16;
} UI_Graph1_t;

typedef struct  // 绘制UI 绘制2个图形完整结构体
{
  xFrameHeader Referee_Transmit_Header;
  uint16_t CMD_ID;
  ext_student_interactive_header_data_t Interactive_Header;
  graphic_data_struct_t Graphic[2];
  uint16_t CRC16;
} UI_Graph2_t;

typedef struct  // 绘制UI 绘制5个图形完整结构体
{
  xFrameHeader Referee_Transmit_Header;
  uint16_t CMD_ID;
  ext_student_interactive_header_data_t Interactive_Header;
  graphic_data_struct_t Graphic[5];
  uint16_t CRC16;
} UI_Graph5_t;

typedef struct  // 绘制UI 绘制7个图形完整结构体
{
  xFrameHeader Referee_Transmit_Header;
  uint16_t CMD_ID;
  ext_student_interactive_header_data_t Interactive_Header;
  graphic_data_struct_t Graphic[7];
  uint16_t CRC16;
} UI_Graph7_t;

typedef struct  // 绘制UI 绘制1字符串完整结构体
{
  xFrameHeader Referee_Transmit_Header;
  uint16_t CMD_ID;
  ext_student_interactive_header_data_t Interactive_Header;
  string_data_struct_t String;
  uint16_t CRC16;
} UI_String_t;

typedef struct  // 绘制UI UI删除图形完整结构体
{
  xFrameHeader Referee_Transmit_Header;
  uint16_t CMD_ID;
  ext_student_interactive_header_data_t Interactive_Header;
  delete_data_struct_t Delete;
  uint16_t CRC16;
} UI_Delete_t;

class UI_Def {
 public:
  UI_Graph1_t UI_Graph1[10];
  UI_Graph2_t UI_Graph2[5];
  UI_Graph5_t UI_Graph5[5];
  UI_Graph7_t UI_Graph7[2];
  UI_String_t UI_String[10];
  UI_Delete_t UI_Delete;
  void Draw_Line(graphic_data_struct_t* Graph, char GraphName[3],
                 uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                 uint16_t Width, uint16_t StartX, uint16_t StartY,
                 uint16_t EndX, uint16_t EndY);
  void Draw_Rectangle(graphic_data_struct_t* Graph, char GraphName[3],
                      uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                      uint16_t Width, uint16_t StartX, uint16_t StartY,
                      uint16_t EndX, uint16_t EndY);
  void Draw_Circle(graphic_data_struct_t* Graph, char GraphName[3],
                   uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                   uint16_t Width, uint16_t CenterX, uint16_t CenterY,
                   uint16_t Radius);
  void Draw_Ellipse(graphic_data_struct_t* Graph, char GraphName[3],
                    uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                    uint16_t Width, uint16_t CenterX, uint16_t CenterY,
                    uint16_t XHalfAxis, uint16_t YHalfAxis);
  void Draw_Arc(graphic_data_struct_t* Graph, char GraphName[3],
                uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                uint16_t StartAngle, uint16_t EndAngle, uint16_t Width,
                uint16_t CenterX, uint16_t CenterY, uint16_t XHalfAxis,
                uint16_t YHalfAxis);
  void Draw_Float(graphic_data_struct_t* Graph, char GraphName[3],
                  uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                  uint16_t NumberSize, uint16_t Significant, uint16_t Width,
                  uint16_t StartX, uint16_t StartY, float FloatData);
  void Draw_Int(graphic_data_struct_t* Graph, char GraphName[3],
                uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                uint16_t NumberSize, uint16_t Width, uint16_t StartX,
                uint16_t StartY, int32_t IntData);
  void Draw_String(string_data_struct_t* String, char StringName[3],
                   uint8_t StringOperate, uint8_t Layer, uint8_t Color,
                   uint16_t CharSize, uint16_t StringLength, uint16_t Width,
                   uint16_t StartX, uint16_t StartY, char* StringData);
  void PushUp_Graphs(uint8_t Counter, void* Graphs, uint8_t RobotID);
  void PushUp_String(UI_String_t* String, uint8_t RobotID);
  void PushUp_Delete(UI_Delete_t* Delete, uint8_t RobotID);
};

/* Exported variables --------------------------------------------------------*/
extern UI_Def UI;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __FILE_H_ */
