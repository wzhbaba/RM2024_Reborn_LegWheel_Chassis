/**
 *******************************************************************************
 * @file      : referee.h
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
#ifndef __REFEREE_H_
#define __REFEREE_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#define FALSE 0
#define TRUE 1

#define JUDGE_DATA_ERROR 0
#define JUDGE_DATA_CORRECT 1

#define LEN_HEADER 5  // 帧头长
#define LEN_CMDID 2   // 命令码长度
#define LEN_TAIL 2    // 帧尾CRC16

// 起始字节,协议固定为0xA5
#define JUDGE_FRAME_HEADER (0xA5)

#pragma pack(1)

/***************裁判系统数据接收********************/

typedef enum {
    FRAME_HEADER = 0,
    CMD_ID = 5,
    DATA = 7,
} JudgeFrameOffset;

// 5字节帧头,偏移位置
typedef enum {
    SOF = 0,          // 起始位
    DATA_LENGTH = 1,  // 帧内数据长度,根据这个来获取数据长度
    SEQ = 3,          // 包序号
    CRC8 = 4          // CRC8

} FrameHeaderOffset;

// 命令码ID,用来判断接收的是什么数据
typedef enum {
    ID_game_state = 0x0001,                // 比赛状态数据
    ID_game_result = 0x0002,               // 比赛结果数据
    ID_game_robot_survivors = 0x0003,      // 比赛机器人血量数据
    ID_event_data = 0x0101,                // 场地事件数据
    ID_supply_projectile_action = 0x0102,  // 场地补给站动作标识数据
    ID_referee_warning = 0x0104,           // 裁判警告数据
    ID_dart_info = 0x0105,                 // 飞镖发射相关数据
    ID_game_robot_state = 0x0201,          // 机器人状态数据
    ID_power_heat_data = 0x0202,           // 实时功率热量数据
    ID_game_robot_pos = 0x0203,            // 机器人位置数据
    ID_buff_musk = 0x0204,                 // 机器人增益数据
    ID_aerial_robot_energy = 0x0205,       // 空中机器人能量状态数据
    ID_robot_hurt = 0x0206,                // 伤害状态数据
    ID_shoot_data = 0x0207,                // 实时射击数据
    ID_projectile_allowance = 0x0208,      // 允许发弹量
    ID_rfid_status = 0x0209,               // RFID状态
    ID_dart_client_cmd = 0x020A,           // 飞镖选手端指令数据
    ID_ground_robot_position = 0x020B,     // 地面机器人位置数据
    ID_radar_mark_data = 0x020C,           // 雷达标记进度数据
    ID_sentry_info = 0x020D,               // 哨兵自主决策信息同步
    ID_radar_info = 0x020E,                // 雷达自主决策信息同步
    ID_student_interactive = 0x0301,       // 机器人交互数据
    ID_custom_robot_data = 0x0302,         // 自定义控制器与机器人交互数据
    ID_map_command = 0x0303,               // 选手端小地图交互数据
    ID_command_data = 0x0304,              // 键鼠遥控数据
    ID_map_robot_data = 0x0305,            // 选手端小地图接收雷达数据
    ID_custom_client_data = 0x0306,        // 自定义控制器与选手端交互数据
    ID_map_data = 0x0307,                  // 选手端小地图接收哨兵数据
    ID_custom_info = 0x0308,               // 选手端小地图接收机器人数据
} CmdID;

typedef enum {
    UI_Data_ID_Del = 0x100,
    UI_Data_ID_Draw1 = 0x101,
    UI_Data_ID_Draw2 = 0x102,
    UI_Data_ID_Draw5 = 0x103,
    UI_Data_ID_Draw7 = 0x104,
    UI_Data_ID_DrawChar = 0x110,

    /* 自定义交互数据部分 */
    Communicate_Data_ID = 0x0200,

} Interactive_Data_ID_e;

// 命令码数据段长,根据官方协议来定义长度
typedef enum {
    LEN_game_state = 11,               // 0x0001
    LEN_game_result = 1,               // 0x0002
    LEN_game_robot_HP = 32,            // 0x0003
    LEN_event_data = 4,                // 0x0101
    LEN_supply_projectile_action = 4,  // 0x0102
    LEN_game_robot_state = 13,         // 0x0201
    LEN_power_heat_data = 16,          // 0x0202
    LEN_game_robot_pos = 16,           // 0x0203
    LEN_buff_musk = 6,                 // 0x0204
    LEN_aerial_robot_energy = 2,       // 0x0205
    LEN_robot_hurt = 1,                // 0x0206
    LEN_shoot_data = 7,                // 0x0207
    LEN_projectile_allowance = 6,      // 0x0208
    LEN_rfid_status = 4,               // 0x0209
    LEN_dart_client_cmd = 6,           // 0x020A
    LEN_ground_robot_position = 40,    // 0x020B
    LEN_radar_mark_data = 6,           // 0x020C
    LEN_sentry_info = 4,               // 0x020D
    LEN_radar_info = 1,                // 0x020E
    LEN_student_interactive = 128,     // 0x0301
    LEN_custom_robot_data = 30,        // 0x0302
    LEN_map_command = 15,              // 0x0303
    LEN_command_data = 12,             // 0x0304
    LEN_map_robot_data = 10,           // 0x0305
    LEN_custom_client_data = 8,        // 0x0306
    LEN_map_data = 103,                // 0x0307
    LEN_custom_info = 34,              // 0x0308
} JudgeDataLength;

/* 自定义帧头 */
typedef struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;

} xFrameHeader;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_state_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct
{
    uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct
{
    uint32_t event_type;
} ext_event_data_t;

/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef struct
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} ext_referee_warning_t;

/* ID: 0X0201  Byte: 27    机器人状态数据 */
typedef struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_game_robot_state_t;

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef struct
{
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef struct
{
    float x;
    float y;
    float yaw;
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef struct
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} ext_buff_musk_t;

/* ID: 0x0205  Byte:  1    空中机器人能量状态数据 */
typedef struct
{
    uint8_t airforce_status;
    uint8_t time_remain;
} aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

/***************裁判系统数据********************/
/*

    交互数据，包括一个统一的数据段头结构，
    包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
    整个交互数据的包总共长最大为 128 个字节，
    减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
    故而发送的内容数据段最大为 113。
    整个交互数据 0x0301 的包上行频率为 10Hz。

    机器人 ID：
    1，英雄(红)；
    2，工程(红)；
    3/4/5，步兵(红)；
    6，空中(红)；
    7，哨兵(红)；
    11，英雄(蓝)；
    12，工程(蓝)；
    13/14/15，步兵(蓝)；
    16，空中(蓝)；
    17，哨兵(蓝)。
    客户端 ID：
    0x0101 为英雄操作手客户端( 红) ；
    0x0102 ，工程操作手客户端 ((红 )；
    0x0103/0x0104/0x0105，步兵操作手客户端(红)；
    0x0106，空中操作手客户端((红)；
    0x0111，英雄操作手客户端(蓝)；
    0x0112，工程操作手客户端(蓝)；
    0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
    0x0116，空中操作手客户端(蓝)。
*/
/* 交互数据接收信息：0x0301  */
typedef struct
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/*
    客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
    发送频率：上限 10Hz
    1.	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz
    字节偏移量 	大小 	说明 				备注
    0 			2 		数据的内容 ID 		0xD180
    2 			2 		送者的 ID 			需要校验发送者机器人的 ID 正确性
    4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端
    6 			4 		自定义浮点数据 1
    10 			4 		自定义浮点数据 2
    14 			4 		自定义浮点数据 3
    18 			1 		自定义 8 位数据 4

*/
typedef struct
{
    float data1;
    float data2;
    float data3;
    uint8_t masks;
} client_custom_data_t;

/*
    学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
    交互数据 机器人间通信：0x0301。
    发送频率：上限 10Hz

    字节偏移量 	大小 	说明 			备注
    0 			2 		数据的内容 ID 	0x0200~0x02FF
                                        可以在以上 ID 段选取，具体 ID 含义由参赛队自定义

    2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性，
    4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
                                        例如不能发送到敌对机器人的ID
    6 			n 		数据段 			n 需要小于 113

*/
typedef struct
{
    uint8_t data[10];  // 数据段,n需要小于113
} robot_interactive_data_t;

// 发送给客户端的信息
// 帧头  命令码   数据段头结构  数据段   帧尾
typedef struct
{
    xFrameHeader txFrameHeader;                             // 帧头
    uint16_t CmdID;                                         // 命令码
    ext_student_interactive_header_data_t dataFrameHeader;  // 数据段头结构
    client_custom_data_t clientData;                        // 数据段
    uint16_t FrameTail;                                     // 帧尾
} ext_SendClientData_t;

// 机器人交互信息
typedef struct
{
    xFrameHeader txFrameHeader;                             // 帧头
    uint16_t CmdID;                                         // 命令码
    ext_student_interactive_header_data_t dataFrameHeader;  // 数据段头结构
    robot_interactive_data_t interactData;                  // 数据段
    uint16_t FrameTail;                                     // 帧尾
} ext_CommunatianData_t;

typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t left_button_down;
    uint8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} ext_robot_command_t;

class Referee
{
   public:
    UartInstance *p_instance_;
    void Update(uint8_t *_p_data);
    void Init(UART_HandleTypeDef *_p_huart);

    xFrameHeader FrameHeader;  // 接收到的帧头信息
    uint16_t CmdID;
    ext_game_state_t game_state_;                              // 0x0001
    ext_game_result_t game_result_;                            // 0x0002
    ext_game_robot_HP_t game_robot_HP_;                        // 0x0003
    ext_event_data_t event_data_;                              // 0x0101
    ext_supply_projectile_action_t supply_projectile_action_;  // 0x0102
    ext_game_robot_state_t game_robot_state_;                  // 0x0201
    ext_power_heat_data_t power_heat_data_;                    // 0x0202
    ext_game_robot_pos_t game_robot_pos_;                      // 0x0203
    ext_buff_musk_t buff_musk_;                                // 0x0204
    aerial_robot_energy_t aerial_robot_energy_;                // 0x0205
    ext_robot_hurt_t robot_hurt_;                              // 0x0206
    ext_shoot_data_t shoot_data_;                              // 0x0207
    ext_SendClientData_t show_data_;                           // 客户端信息
    ext_CommunatianData_t commu_data;                          // 队友通信信息
    ext_robot_command_t comma_data;                            // 图传链路
};

/* Exported variables --------------------------------------------------------*/
extern Referee referee;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __REFEREE_H_ */
