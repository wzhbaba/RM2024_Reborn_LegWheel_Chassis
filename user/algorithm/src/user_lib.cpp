/**
 *******************************************************************************
 * @file      : user_lib.cpp
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
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

namespace Math
{
float Sqrt(float _x)
{
    float y;
    float delta;
    float maxError;

    if (_x <= 0) {
        return 0;
    }

    // initial guess
    y = _x / 2;

    // refine
    maxError = _x * 0.001f;

    do {
        delta = (y * y) - _x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

float AbsLimit(float _num, float _limit)
{
    if (_num > _limit) {
        _num = _limit;
    } else if (_num < -_limit) {
        _num = -_limit;
    }
    return _num;
}

float Sign(float _value)
{
    if (_value >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

float FloatDeadband(float _value, float _min_value, float _max_value)
{
    if (_value < _max_value && _value > _min_value) {
        _value = 0.0f;
    }
    return _value;
}

float FloatConstrain(float _value, float _min_value, float _max_value)
{
    if (_value < _min_value)
        return _min_value;
    else if (_value > _max_value)
        return _max_value;
    else
        return _value;
}

int16_t Int16Constrain(int16_t _value, int16_t _min_value, int16_t _max_value)
{
    if (_value < _min_value)
        return _min_value;
    else if (_value > _max_value)
        return _max_value;
    else
        return _value;
}

float LoopFloatConstrain(float _value, float _min_value, float _max_value)
{
    if (_max_value < _min_value) {
        return _value;
    }

    if (_value > _max_value) {
        float len = _max_value - _min_value;
        while (_value > _max_value) {
            _value -= len;
        }
    } else if (_value < _min_value) {
        float len = _max_value - _min_value;
        while (_value < _min_value) {
            _value += len;
        }
    }
    return _value;
}

float ThetaFormat(float _ang)
{
    return LoopFloatConstrain(_ang, -180.0f, 180.0f);
}

int FloatRounding(float _raw)
{
    static int integer;
    static float decimal;
    integer = (int)_raw;
    decimal = _raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

float NormOf3d(float *v)
{
    return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

}  // namespace Math
