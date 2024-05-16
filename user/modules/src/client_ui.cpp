/**
 *******************************************************************************
 * @file      : Client_UI.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "client_ui.h"

#include "bsp_uart.h"
#include "crc_def.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
UI_Def UI;
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief   绘制直线
 * @param       *Graph: UI图形数据结构体指针
 * @param       GraphName: 图形名 作为客户端的索引
 * @param       GraphOperate: UI图形操作 对应UI_Graph_XXX的4种操作
 * @param       Layer: UI图形图层 [0,9]
 * @param       Color: UI图形颜色 对应UI_Color_XXX的9种颜色
 * @param       Width: 线宽
 * @param       StartX: 起始坐标X
 * @param       StartY: 起始坐标Y
 * @param       EndX: 截止坐标X
 * @param       EndY: 截止坐标Y
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::Draw_Line(graphic_data_struct_t* Graph, char GraphName[3],
                       uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                       uint16_t Width, uint16_t StartX, uint16_t StartY,
                       uint16_t EndX, uint16_t EndY) {
  Graph->graphic_name[0] = GraphName[0];
  Graph->graphic_name[1] = GraphName[1];
  Graph->graphic_name[2] = GraphName[2];
  Graph->operate_tpye = GraphOperate;
  Graph->graphic_tpye = UI_Graph_Line;
  Graph->layer = Layer;
  Graph->color = Color;
  Graph->width = Width;
  Graph->start_x = StartX;
  Graph->start_y = StartY;
  Graph->end_x = EndX;
  Graph->end_y = EndY;
}

/**
 * @brief   绘制矩形
 * @param       *Graph: UI图形数据结构体指针
 * @param       GraphName: 图形名 作为客户端的索引
 * @param       GraphOperate: UI图形操作 对应UI_Graph_XXX的4种操作
 * @param       Layer: UI图形图层 [0,9]
 * @param       Color: UI图形颜色 对应UI_Color_XXX的9种颜色
 * @param       Width: 线宽
 * @param       StartX: 起始坐标X
 * @param       StartY: 起始坐标Y
 * @param       EndX: 截止坐标X
 * @param       EndY: 截止坐标Y
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::Draw_Rectangle(graphic_data_struct_t* Graph, char GraphName[3],
                            uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                            uint16_t Width, uint16_t StartX, uint16_t StartY,
                            uint16_t EndX, uint16_t EndY) {
  Graph->graphic_name[0] = GraphName[0];
  Graph->graphic_name[1] = GraphName[1];
  Graph->graphic_name[2] = GraphName[2];
  Graph->operate_tpye = GraphOperate;
  Graph->graphic_tpye = UI_Graph_Rectangle;
  Graph->layer = Layer;
  Graph->color = Color;
  Graph->width = Width;
  Graph->start_x = StartX;
  Graph->start_y = StartY;
  Graph->end_x = EndX;
  Graph->end_y = EndY;
}

/**
 * @brief   绘制整圆
 * @param       *Graph: UI图形数据结构体指针
 * @param       GraphName: 图形名 作为客户端的索引
 * @param       GraphOperate: UI图形操作 对应UI_Graph_XXX的4种操作
 * @param       Layer: UI图形图层 [0,9]
 * @param       Color: UI图形颜色 对应UI_Color_XXX的9种颜色
 * @param       Width: 线宽
 * @param       CenterX:圆心坐标X
 * @param       CenterY:圆心坐标Y
 * @param       Radius:半径
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::Draw_Circle(graphic_data_struct_t* Graph, char GraphName[3],
                         uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                         uint16_t Width, uint16_t CenterX, uint16_t CenterY,
                         uint16_t Radius) {
  Graph->graphic_name[0] = GraphName[0];
  Graph->graphic_name[1] = GraphName[1];
  Graph->graphic_name[2] = GraphName[2];
  Graph->operate_tpye = GraphOperate;
  Graph->graphic_tpye = UI_Graph_Circle;
  Graph->layer = Layer;
  Graph->color = Color;
  Graph->width = Width;
  Graph->start_x = CenterX;
  Graph->start_y = CenterY;
  Graph->radius = Radius;
}

/**
 * @brief   绘制椭圆
 * @param       *Graph: UI图形数据结构体指针
 * @param       GraphName: 图形名 作为客户端的索引
 * @param       GraphOperate: UI图形操作 对应UI_Graph_XXX的4种操作
 * @param       Layer: UI图形图层 [0,9]
 * @param       Color: UI图形颜色 对应UI_Color_XXX的9种颜色
 * @param       Width: 线宽
 * @param       CenterX:圆心坐标X
 * @param       CenterY:圆心坐标Y
 * @param       XHalfAxis:X半轴长
 * @param       YHalfAxis:Y半轴长
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::Draw_Ellipse(graphic_data_struct_t* Graph, char GraphName[3],
                          uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                          uint16_t Width, uint16_t CenterX, uint16_t CenterY,
                          uint16_t XHalfAxis, uint16_t YHalfAxis) {
  Graph->graphic_name[0] = GraphName[0];
  Graph->graphic_name[1] = GraphName[1];
  Graph->graphic_name[2] = GraphName[2];
  Graph->operate_tpye = GraphOperate;
  Graph->graphic_tpye = UI_Graph_Ellipse;
  Graph->layer = Layer;
  Graph->color = Color;
  Graph->width = Width;
  Graph->start_x = CenterX;
  Graph->start_y = CenterY;
  Graph->end_x = XHalfAxis;
  Graph->end_y = YHalfAxis;
}

/**
 * @brief   绘制圆弧
 * @param       *Graph: UI图形数据结构体指针
 * @param       GraphName: 图形名 作为客户端的索引
 * @param       GraphOperate: UI图形操作 对应UI_Graph_XXX的4种操作
 * @param       Layer: UI图形图层 [0,9]
 * @param       Color: UI图形颜色 对应UI_Color_XXX的9种颜色
 * @param       StartAngle:起始角度 [0,360]
 * @param       EndAngle:截止角度 [0,360]
 * @param       Width:线宽
 * @param       CenterX:圆心坐标X
 * @param       CenterY:圆心坐标Y
 * @param       XHalfAxis:X半轴长
 * @param       YHalfAxis:Y半轴长
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::Draw_Arc(graphic_data_struct_t* Graph, char GraphName[3],
                      uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                      uint16_t StartAngle, uint16_t EndAngle, uint16_t Width,
                      uint16_t CenterX, uint16_t CenterY, uint16_t XHalfAxis,
                      uint16_t YHalfAxis) {
  Graph->graphic_name[0] = GraphName[0];
  Graph->graphic_name[1] = GraphName[1];
  Graph->graphic_name[2] = GraphName[2];
  Graph->operate_tpye = GraphOperate;
  Graph->graphic_tpye = UI_Graph_Arc;
  Graph->layer = Layer;
  Graph->color = Color;
  Graph->start_angle = StartAngle;
  Graph->end_angle = EndAngle;
  Graph->width = Width;
  Graph->start_x = CenterX;
  Graph->start_y = CenterY;
  Graph->end_x = XHalfAxis;
  Graph->end_y = YHalfAxis;
}

/**
 * @brief   绘制小数
 * @param       *Graph: UI图形数据结构体指针
 * @param       GraphName: 图形名 作为客户端的索引
 * @param       GraphOperate: UI图形操作 对应UI_Graph_XXX的4种操作
 * @param       Layer: UI图形图层 [0,9]
 * @param       Color: UI图形颜色 对应UI_Color_XXX的9种颜色
 * @param       NumberSize: 字体大小
 * @param       Significant: 有效位数
 * @param       Width: 线宽
 * @param       StartX: 起始坐标X
 * @param       StartY: 起始坐标Y
 * @param       FloatData: 数字内容
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::Draw_Float(graphic_data_struct_t* Graph, char GraphName[3],
                        uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                        uint16_t NumberSize, uint16_t Significant,
                        uint16_t Width, uint16_t StartX, uint16_t StartY,
                        float FloatData) {
  Graph->graphic_name[0] = GraphName[0];
  Graph->graphic_name[1] = GraphName[1];
  Graph->graphic_name[2] = GraphName[2];
  Graph->operate_tpye = GraphOperate;
  Graph->graphic_tpye = UI_Graph_Float;
  Graph->layer = Layer;
  Graph->color = Color;
  Graph->start_angle = NumberSize;
  Graph->end_angle = Significant;
  Graph->width = Width;
  Graph->start_x = StartX;
  Graph->start_y = StartY;
  int32_t IntData = FloatData * 1000;
  Graph->radius = (IntData & 0x000003ff) >> 0;
  Graph->end_x = (IntData & 0x001ffc00) >> 10;
  Graph->end_y = (IntData & 0xffe00000) >> 21;
}

/**
 * @brief   绘制整数
 * @param       *Graph: UI图形数据结构体指针
 * @param       GraphName: 图形名 作为客户端的索引
 * @param       GraphOperate: UI图形操作 对应UI_Graph_XXX的4种操作
 * @param       Layer:  UI图形图层 [0,9]
 * @param       Color: UI图形颜色 对应UI_Color_XXX的9种颜色
 * @param       NumberSize: 字体大小
 * @param       Width: 线宽
 * @param       StartX: 起始坐标X
 * @param       StartY: 起始坐标Y
 * @param       IntData: 数字内容
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::Draw_Int(graphic_data_struct_t* Graph, char GraphName[3],
                      uint8_t GraphOperate, uint8_t Layer, uint8_t Color,
                      uint16_t NumberSize, uint16_t Width, uint16_t StartX,
                      uint16_t StartY, int32_t IntData) {
  Graph->graphic_name[0] = GraphName[0];
  Graph->graphic_name[1] = GraphName[1];
  Graph->graphic_name[2] = GraphName[2];
  Graph->operate_tpye = GraphOperate;
  Graph->graphic_tpye = UI_Graph_Int;
  Graph->layer = Layer;
  Graph->color = Color;
  Graph->start_angle = NumberSize;
  Graph->width = Width;
  Graph->start_x = StartX;
  Graph->start_y = StartY;
  Graph->radius = (IntData & 0x000003ff) >> 0;
  Graph->end_x = (IntData & 0x001ffc00) >> 10;
  Graph->end_y = (IntData & 0xffe00000) >> 21;
}

/**
 * @brief   绘制字符
 * @param       *String: UI图形数据结构体指针
 * @param       StringName: 图形名 作为客户端的索引
 * @param       StringOperate: UI图形操作 对应UI_Graph_XXX的4种操作
 * @param       Layer: UI图形图层 [0,9]
 * @param       Color: UI图形颜色 对应UI_Color_XXX的9种颜色
 * @param       CharSize: 字体大小
 * @param       StringLength: 字符串长度
 * @param       Width: 线宽
 * @param       StartX: 起始坐标X
 * @param       StartY: 起始坐标Y
 * @param       *StringData: 字符串内容
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::Draw_String(string_data_struct_t* String, char StringName[3],
                         uint8_t StringOperate, uint8_t Layer, uint8_t Color,
                         uint16_t CharSize, uint16_t StringLength,
                         uint16_t Width, uint16_t StartX, uint16_t StartY,
                         char* StringData) {
  String->string_name[0] = StringName[0];
  String->string_name[1] = StringName[1];
  String->string_name[2] = StringName[2];
  String->operate_tpye = StringOperate;
  String->graphic_tpye = UI_Graph_String;
  String->layer = Layer;
  String->color = Color;
  String->start_angle = CharSize;
  String->end_angle = StringLength;
  String->width = Width;
  String->start_x = StartX;
  String->start_y = StartY;
  for (int i = 0; i < StringLength; i++)
    String->stringdata[i] = *StringData++;
}

/**
 * @brief
 * @param       Counter:
 * @param       *Graphs:
 * @param       RobotID:
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::PushUp_Graphs(uint8_t Counter, void* Graphs, uint8_t RobotID) {
  UI_Graph1_t* Graph = (UI_Graph1_t*)Graphs;  // 假设只发一个基本图形

  /* 填充 frame_header */
  Graph->Referee_Transmit_Header.SOF = JUDGE_FRAME_HEADER;
  if (Counter == 1)
    Graph->Referee_Transmit_Header.DataLength = 6 + 1 * 15;
  else if (Counter == 2)
    Graph->Referee_Transmit_Header.DataLength = 6 + 2 * 15;
  else if (Counter == 5)
    Graph->Referee_Transmit_Header.DataLength = 6 + 5 * 15;
  else if (Counter == 7)
    Graph->Referee_Transmit_Header.DataLength = 6 + 7 * 15;
  Graph->Referee_Transmit_Header.Seq = Graph->Referee_Transmit_Header.Seq + 1;
  Graph->Referee_Transmit_Header.CRC8 =
      Get_CRC8_Check_Sum((uint8_t*)Graph, CRC8, 0xFF);

  /* 填充 cmd_id */
  Graph->CMD_ID = ID_student_interactive;

  /* 填充 student_interactive_header */
  if (Counter == 1)
    Graph->Interactive_Header.data_cmd_id = UI_Data_ID_Draw1;
  else if (Counter == 2)
    Graph->Interactive_Header.data_cmd_id = UI_Data_ID_Draw2;
  else if (Counter == 5)
    Graph->Interactive_Header.data_cmd_id = UI_Data_ID_Draw5;
  else if (Counter == 7)
    Graph->Interactive_Header.data_cmd_id = UI_Data_ID_Draw7;
  Graph->Interactive_Header.send_ID = RobotID;            // 当前机器人ID
  Graph->Interactive_Header.receiver_ID = RobotID + 256;  // 对应操作手ID

  /* 填充 frame_tail 即CRC16 */
  if (Counter == 1) {
    UI_Graph1_t* Graph1 = (UI_Graph1_t*)Graphs;
    Graph1->CRC16 =
        Get_CRC16_Check_Sum((uint8_t*)Graph1, sizeof(UI_Graph1_t) - 2, 0xFFFF);
  } else if (Counter == 2) {
    UI_Graph2_t* Graph2 = (UI_Graph2_t*)Graphs;
    Graph2->CRC16 =
        Get_CRC16_Check_Sum((uint8_t*)Graph2, sizeof(UI_Graph2_t) - 2, 0xFFFF);
  } else if (Counter == 5) {
    UI_Graph5_t* Graph5 = (UI_Graph5_t*)Graphs;
    Graph5->CRC16 =
        Get_CRC16_Check_Sum((uint8_t*)Graph5, sizeof(UI_Graph5_t) - 2, 0xFFFF);
  } else if (Counter == 7) {
    UI_Graph7_t* Graph7 = (UI_Graph7_t*)Graphs;
    Graph7->CRC16 =
        Get_CRC16_Check_Sum((uint8_t*)Graph7, sizeof(UI_Graph7_t) - 2, 0xFFFF);
  }

  /* 使用串口PushUp到裁判系统 */
  if (Counter == 1)
    UartSendData(&huart5, (uint8_t*)Graph, sizeof(UI_Graph1_t),
                 UART_TRAMSMIT_DMA);
  else if (Counter == 2)
    UartSendData(&huart5, (uint8_t*)Graph, sizeof(UI_Graph2_t),
                 UART_TRAMSMIT_DMA);
  else if (Counter == 5)
    UartSendData(&huart5, (uint8_t*)Graph, sizeof(UI_Graph5_t),
                 UART_TRAMSMIT_DMA);
  else if (Counter == 7)
    UartSendData(&huart5, (uint8_t*)Graph, sizeof(UI_Graph7_t),
                 UART_TRAMSMIT_DMA);
}

/**
 * @brief
 * @param       *String:
 * @param       RobotID:
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::PushUp_String(UI_String_t* String, uint8_t RobotID) {
  /* 填充 frame_header */
  String->Referee_Transmit_Header.SOF = JUDGE_FRAME_HEADER;
  String->Referee_Transmit_Header.DataLength = 6 + 45;
  String->Referee_Transmit_Header.Seq = String->Referee_Transmit_Header.Seq + 1;
  String->Referee_Transmit_Header.CRC8 = Get_CRC8_Check_Sum(
      (uint8_t*)(&String->Referee_Transmit_Header), CRC8, 0xFF);

  /* 填充 cmd_id */
  String->CMD_ID = ID_student_interactive;

  /* 填充 student_interactive_header */
  String->Interactive_Header.data_cmd_id = UI_Data_ID_DrawChar;
  String->Interactive_Header.send_ID = RobotID;            // 当前机器人ID
  String->Interactive_Header.receiver_ID = RobotID + 256;  // 对应操作手ID

  /* 填充 frame_tail 即CRC16 */
  String->CRC16 =
      Get_CRC16_Check_Sum((uint8_t*)String, sizeof(UI_String_t) - 2, 0xFFFF);

  /* 使用串口PushUp到裁判系统 */
  UartSendData(&huart5, (uint8_t*)String, sizeof(UI_String_t),
               UART_TRAMSMIT_DMA);
}

/**
 * @brief
 * @param       *Delete:
 * @param       RobotID:
 *   @arg       None
 * @retval      None
 * @note        None
 */
void UI_Def::PushUp_Delete(UI_Delete_t* Delete, uint8_t RobotID) {
  /* 填充 frame_header */
  Delete->Referee_Transmit_Header.SOF = JUDGE_FRAME_HEADER;
  Delete->Referee_Transmit_Header.DataLength = 6 + 2;
  Delete->Referee_Transmit_Header.Seq = Delete->Referee_Transmit_Header.Seq + 1;
  Delete->Referee_Transmit_Header.CRC8 = Get_CRC8_Check_Sum(
      (uint8_t*)(&Delete->Referee_Transmit_Header), CRC8, 0xFF);

  /* 填充 cmd_id */
  Delete->CMD_ID = ID_student_interactive;

  /* 填充 student_interactive_header */
  Delete->Interactive_Header.data_cmd_id = UI_Data_ID_Del;
  Delete->Interactive_Header.send_ID = RobotID;            // 当前机器人ID
  Delete->Interactive_Header.receiver_ID = RobotID + 256;  // 对应操作手ID

  /* 填充 frame_tail 即CRC16 */
  Delete->CRC16 =
      Get_CRC16_Check_Sum((uint8_t*)Delete, sizeof(UI_Delete_t) - 2, 0xFFFF);

  /* 使用串口PushUp到裁判系统 */
  UartSendData(&huart5, (uint8_t*)Delete, sizeof(UI_Delete_t),
               UART_TRAMSMIT_DMA);
}
