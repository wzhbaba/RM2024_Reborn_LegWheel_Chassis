/**
 *******************************************************************************
 * @file      : user_lib.h
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
#ifndef __USER_LIB_H_
#define __USER_LIB_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

#include "stdint.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define VAL_LIMIT(val, min, max)     \
    do {                             \
        if ((val) <= (min)) {        \
            (val) = (min);           \
        } else if ((val) >= (max)) { \
            (val) = (max);           \
        }                            \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do {                                \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do {                            \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/* Exported types ------------------------------------------------------------*/
/**
 * @brief Namespace containing mathematical functions.
 */
namespace Math
{
    /**
     * @brief Calculates the square root of a given number.
     * @param _x The number to calculate the square root of.
     * @return The square root of the given number.
     */
    float Sqrt(float _x);

    /**
     * @brief Limits the absolute value of a number.
     * @param _num The number to limit.
     * @param _limit The absolute value limit.
     * @return The limited number.
     */
    float AbsLimit(float _num, float _limit);

    /**
     * @brief Returns the sign of a number.
     * @param _value The number to determine the sign of.
     * @return 1 if the number is positive, -1 if the number is negative, 0 if the number is zero.
     */
    float Sign(float _value);

    /**
     * @brief Applies a deadband to a floating-point value.
     * @param _value The value to apply the deadband to.
     * @param _min_value The minimum value of the deadband range.
     * @param _max_value The maximum value of the deadband range.
     * @return The value after applying the deadband.
     */
    float FloatDeadband(float _value, float _min_value, float _max_value);

    /**
     * @brief Constrains a floating-point value within a specified range.
     * @param _value The value to constrain.
     * @param _min_value The minimum value of the range.
     * @param _max_value The maximum value of the range.
     * @return The constrained value.
     */
    float FloatConstrain(float _value, float _min_value, float _max_value);

    /**
     * @brief Constrains a 16-bit integer value within a specified range.
     * @param _value The value to constrain.
     * @param _min_value The minimum value of the range.
     * @param _max_value The maximum value of the range.
     * @return The constrained value.
     */
    int16_t Int16Constrain(int16_t _value, int16_t _min_value, int16_t _max_value);

    /**
     * @brief Constrains a floating-point value within a specified range, with loop-around behavior.
     * @param _value The value to constrain.
     * @param _min_value The minimum value of the range.
     * @param _max_value The maximum value of the range.
     * @return The constrained value.
     */
    float LoopFloatConstrain(float _value, float _min_value, float _max_value);

    /**
     * @brief Formats an angle value to be within the range of -180 to 180 degrees.
     * @param _ang The angle value to format.
     * @return The formatted angle value.
     */
    float ThetaFormat(float _ang);

    /**
     * @brief Rounds a floating-point number to the nearest integer.
     * @param _raw The raw floating-point number.
     * @return The rounded integer value.
     */
    int FloatRounding(float _raw);

    /**
     * @brief Normalizes a 3D vector.
     * @param v The 3D vector to normalize.
     * @return The normalized vector.
     */
    float *Norm3d(float *v);

    /**
     * @brief Calculates the norm (magnitude) of a 3D vector.
     * @param v The 3D vector.
     * @return The norm of the vector.
     */
    float NormOf3d(float *v);

    /**
     * @brief Calculates the cross product of two 3D vectors.
     * @param v1 The first 3D vector.
     * @param v2 The second 3D vector.
     * @param res The resulting cross product vector.
     */
    void Cross3d(float *v1, float *v2, float *res);

    /**
     * @brief Calculates the dot product of two 3D vectors.
     * @param v1 The first 3D vector.
     * @param v2 The second 3D vector.
     * @return The dot product of the two vectors.
     */
    float Dot3d(float *v1, float *v2);
};  // namespace Math
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __USER_LIB_H_ */
