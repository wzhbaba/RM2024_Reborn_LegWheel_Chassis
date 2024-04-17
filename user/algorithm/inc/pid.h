/**
 *******************************************************************************
 * @file      : pid.h
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
#ifndef __PID_H_
#define __PID_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include <stm32f446xx.h>

#include "arm_math.h"
#include "stdint.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

typedef enum {
  PID_IMPROVE_NONE = 0b00000000,                 // 0000 0000
  PID_INTEGRAL_LIMIT = 0b00000001,               // 0000 0001
  PID_DERIVATIVE_ON_MEASUREMENT = 0b00000010,    // 0000 0010
  PID_TRAPEZOID_INTEGRAL = 0b00000100,           // 0000 0100
  PID_PROPORTIONAL_ON_MEASUREMENT = 0b00001000,  // 0000 1000
  PID_OUTPUT_FILTER = 0b00010000,                // 0001 0000
  PID_CHANGING_INTEGRATION_RATE = 0b00100000,    // 0010 0000
  PID_DERIVATIVE_FILTER = 0b01000000,            // 0100 0000
  PID_ERROR_HANDLE = 0b10000000,                 // 1000 0000
} PidImprovement;

typedef enum errorType_e {
  PID_ERROR_NONE = 0x00U,
  PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType;

typedef struct {
  uint64_t ERRORCount;
  ErrorType ERRORType;
} PidErrorHandler;

/* Exported types ------------------------------------------------------------*/
class Pid {
 public:
  void Init(float _kp, float _ki, float _kd, float _max_out, float _dead_band) {
    kp_ = _kp;
    ki_ = _ki;
    kd_ = _kd;
    max_out_ = _max_out;
    dead_band_ = _dead_band;
  };
  void Inprovement(uint8_t _improve, float _integral_limit, float _coef_a,
                   float _coef_b, float _output_lpf_rc,
                   float _derivative_lpf_rc) {
    improve_ = _improve;
    integral_limit_ = _integral_limit;
    coef_a_ = _coef_a;
    coef_b_ = _coef_b;
    output_lpf_rc_ = _output_lpf_rc;
    derivative_lpf_rc_ = _derivative_lpf_rc;
  };

  void SetRef(float _ref) { ref_ = _ref; };

  void SetMeasure(float _measure) { measure_ = _measure; };

  float GetOutput() { return output_; }
  float Calculate();
  void TrapezoidIntergral();
  void ChangingIntegratioRate();
  void IntegralLimit();
  void DerivativeOnMeasurement();
  void DerivativeFilter();
  void OutputFilter();
  void OutputLimit();
  void ErrorHandle();

 private:
  float kp_;
  float ki_;
  float kd_;
  float max_out_;
  float dead_band_;

  uint8_t improve_;
  PidErrorHandler error_handle;

  float integral_limit_;
  float coef_a_;
  float coef_b_;
  float output_lpf_rc_;
  float derivative_lpf_rc_;

  float ref_, measure_, last_measure_;
  float err_, last_err_;
  float p_out_, i_out_, d_out_, last_d_out_, i_term_, last_i_term_;
  float output_, last_output_;

  uint32_t DWT_CNT;
  float dt;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

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

#ifdef __cplusplus
}
#endif

#endif /* __PID_H_ */
