/**
 * @file algo_sf_types.h
 * @brief Core type definitions and constants for sensor fusion algorithms
 *
 * This header defines fundamental constants, conversion factors, and sampling
 * configuration used throughout the sensor fusion implementation. It includes
 * mathematical constants, unit conversions, and timing parameters for the
 * Kalman filter updates.
 *
 * @author Vikas Yadav
 * @date 2020
 */

#ifndef _ALGO_SF_TYPES_H_
#define _ALGO_SF_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "math.h"
#include "stdlib.h"

#include "algo_sf_interface.h"

/******************************************************************************
 *                         VECTOR COMPONENT INDICES
 ******************************************************************************/

/** @brief X-axis component index */
#define CHX 0

/** @brief Y-axis component index */
#define CHY 1

/** @brief Z-axis component index */
#define CHZ 2

/******************************************************************************
 *                         BOOLEAN DEFINITIONS
 ******************************************************************************/

#ifndef true
/** @brief Boolean true value */
#define true 1
#endif

#ifndef false
/** @brief Boolean false value */
#define false 0
#endif

/******************************************************************************
 *                         MATHEMATICAL CONSTANTS
 ******************************************************************************/

/** @brief Pi constant */
#define PI               3.14159265358979323846

/** @brief Pi/2 (90 degrees in radians) */
#define PI_OVER_2        (PI/2.0)

/** @brief Pi/4 (45 degrees in radians) */
#define PI_OVER_4        (PI/4.0)

/** @brief Pi/6 (30 degrees in radians) */
#define PI_OVER_6        (PI/6.0)

/** @brief Pi/12 (15 degrees in radians) */
#define PI_OVER_12       (PI/12.0)

/** @brief 2*Pi (360 degrees in radians) */
#define PI_TIMES_2       (PI*2.0)

/** @brief 1/sqrt(2) - frequently used in quaternion calculations */
#define ONEOVERSQRT2     0.70710678118654752440

/** @brief 1/3 - used in numerical approximations */
#define ONETHIRD         0.333333333333333

/** @brief 1/6 - used in Taylor series expansions */
#define ONESIXTH         0.166666666666667

/** @brief 1/24 - used in Taylor series expansions */
#define ONEOVER24        0.0416666666666667

/** @brief 1/48 - used in Taylor series expansions */
#define ONEOVER48        0.0208333333333333

/** @brief 1/120 - used in Taylor series expansions */
#define ONEOVER120       0.00833333333333333

/** @brief 1/3840 - used in high-order approximations */
#define ONEOVER3840      0.000260416666666667

/******************************************************************************
 *                         UNIT CONVERSION CONSTANTS
 ******************************************************************************/

/** @brief Standard gravity in m/s² */
#define GTOMSEC2         9.8

/** @brief Degrees to radians conversion factor (π/180) */
#define DEG2RAD          0.017453292519943295

/** @brief Radians to degrees conversion factor (180/π) */
#define RAD2DEG          (1.0/DEG2RAD)

/** @brief Nanoseconds to milliseconds conversion factor */
#define NSEC2MSEC        1000000

/******************************************************************************
 *                         NUMERICAL PRECISION CONSTANTS
 ******************************************************************************/

/** @brief Double precision minimum value */
#define DBL_MIN          2.2250738585072014E-308

/** @brief Small epsilon for zero comparisons */
#define EPSILON          1e-9

/** @brief Epsilon squared */
#define EPSILON_SQ       (EPSILON*EPSILON)

/** @brief Tiny value for numerical stability */
#define TINY             2.2250738585072014E-30

/******************************************************************************
 *                         SAMPLING RATE CONFIGURATION
 ******************************************************************************/

/**
 * @brief Gyroscope sampling frequency (Hz)
 * @note This sets the base rate for time updates in the Kalman filter
 */
#define SF_GYRO_FS           100

/**
 * @brief Gyroscope sampling interval (seconds)
 * @note Calculated as 1/SF_GYRO_FS = 0.01 seconds (10ms)
 */
#define SF_GYRO_SAMP_INTVL   (1.0/(double)SF_GYRO_FS)

/**
 * @brief Oversampling ratio between gyro and accelerometer
 * @note Gyro runs 4x faster than accel. Fusion updates at accel rate.
 */
#define SF_OVERSAMPLE_RATIO  4

/**
 * @brief Time interval between Kalman filter updates (seconds)
 * @note SF_DELTA_T = 4 * 0.01s = 0.04s (40ms, 25Hz fusion rate)
 */
#define SF_DELTA_T           (SF_OVERSAMPLE_RATIO * SF_GYRO_SAMP_INTVL)

/**
 * @brief Square of the Kalman filter update interval
 * @note Used in covariance propagation equations
 */
#define SF_DELTA_T_SQ        (SF_DELTA_T*SF_DELTA_T)

/******************************************************************************
 *                         EXTERNAL VARIABLES
 ******************************************************************************/

/**
 * @brief Start time offset for timestamp synchronization
 * @note Used to match PC time with recorded sensor data time
 */
extern int64_t start_time_offset_ns;

#endif /* _ALGO_SF_TYPES_H_ */
