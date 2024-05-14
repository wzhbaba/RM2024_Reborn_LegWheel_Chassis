/**
 *******************************************************************************
 * @file      : PM01.h
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
#ifndef __CAP_H_
#define __CAP_H_

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef union {
    uint16_t all;
    struct {
        uint16_t rdy : 1;  /*!< bit0    就绪     */
        uint16_t run : 1;  /*!< bit1    运行     */
        uint16_t alm : 1;  /*!< bit2    报警     */
        uint16_t pwr : 1;  /*!< bit3    电源开关 */
        uint16_t load : 1; /*!< bit4    负载开关 */
        uint16_t cc : 1;   /*!< bit5    恒流     */
        uint16_t cv : 1;   /*!< bit6    恒压     */
        uint16_t cw : 1;   /*!< bit7    恒功率   */
        uint16_t revd : 7; /*!< bit8-14 保留     */
        uint16_t flt : 1;  /*!< bit15   故障     */
    } bit;

} csr_t;

typedef struct mb_reg_type {
    uint16_t ccr;        /*!< 8000H 控制寄存器     */
    uint16_t p_set;      /*!< 8001H 输入功率限制   */
    uint16_t v_set;      /*!< 8002H 输出电压设置   */
    uint16_t i_set;      /*!< 8003H 输出电流限制   */
    csr_t sta_code;      /*!< 8100H 状态标志位     */
    uint16_t err_code;   /*!< 8101H 故障代码       */
    int16_t v_in;        /*!< 8102H 输入电压       */
    int16_t i_in;        /*!< 8103H 输入电流       */
    int16_t p_in;        /*!< 8104H 输入功率       */
    int16_t v_out;       /*!< 8105H 输出电压       */
    int16_t i_out;       /*!< 8106H 输出电流       */
    int16_t p_out;       /*!< 8107H 输出功率       */
    int16_t temp;        /*!< 8108H 温度           */
    uint16_t total_time; /*!< 8109H 累计时间       */
    uint16_t run_time;   /*!< 810AH 运行时间       */

} pm01_od_t;

class PM01_t
{
   public:
    uint16_t access_id_;
    uint16_t response_flg_;
    uint16_t cmd_set_ = 2;
    uint16_t power_set_ = 4000;
    uint16_t vout_set_ = 2600;
    uint16_t iout_set_ = 1500;
    pm01_od_t data;
    void Init(CAN_HandleTypeDef* _p_hcan, uint16_t _id);
    void Update();
    void AccessPoll();
    void SetCapState();
    void SetPower(uint16_t _p_in)
    {
        power_set_ = _p_in * 100;
    };
    void SetIOut(uint16_t _iout)
    {
        iout_set_ = _iout * 100;
    }
    float GetVolt()
    {
        return data.v_out / 100.0f;
    }
    uint8_t GetCapState()
    {
        return cap_state_;
    }

   private:
    CanInstance* p_instance_;
    void SendRemote();
    void SendData(uint16_t _cmd);
    uint16_t SetRxID();
    uint16_t SetTxID(uint16_t _id);
    uint8_t cap_state_;
};
/* Exported variables --------------------------------------------------------*/
extern PM01_t cap;
/* Exported function prototypes ----------------------------------------------*/

#endif /* __FILE_H_ */
