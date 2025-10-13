/*******************************************************************************
 * @file    algo_sf_approx.h
 * @brief   Fast approximation functions for trigonometric operations
 * @author  Vikas Yadav
 * @date    2020
 *
 * @note These functions provide fast approximations for inverse trigonometric
 *       functions with controlled error bounds, suitable for real-time sensor
 *       fusion applications.
 ******************************************************************************/

#ifndef _APPROXIMATIONS_H_
#define _APPROXIMATIONS_H_

#include "algo_sf_types.h"

/****************************************************************************
 * Trigonometric Approximation Constants
 ****************************************************************************/

#define APPROX_MAX_ERROR_DEG  15.0E-6F  /* Maximum approximation error in degrees */

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

/**
 * @brief Fast approximation of arcsine in degrees
 *
 * Returns angle in degrees for -1 <= x <= 1
 * Maximum error: 10.29E-6 deg
 *
 * @param[in] x - Input value in range [-1, 1]
 * @return Angle in degrees, range [-90, 90]
 */
float fasin_deg(float x);

/**
 * @brief Fast approximation of arccosine in degrees
 *
 * Returns angle in degrees for -1 <= x <= 1
 * Maximum error: 14.67E-6 deg
 *
 * @param[in] x - Input value in range [-1, 1]
 * @return Angle in degrees, range [0, 180]
 */
float facos_deg(float x);

/**
 * @brief Fast approximation of arctangent in degrees
 *
 * Returns angle in degrees for any x
 * Maximum error: 9.84E-6 deg
 *
 * @param[in] x - Input value (any real number)
 * @return Angle in degrees, range [-90, 90]
 */
float fatan_deg(float x);

/**
 * @brief Fast approximation of atan2 in degrees
 *
 * Returns angle of point (x,y) from positive x-axis
 * Maximum error: 14.58E-6 deg
 *
 * @param[in] y - Y coordinate
 * @param[in] x - X coordinate
 * @return Angle in degrees, range [-180, 180]
 */
float fatan2_deg(float y, float x);

/**
 * @brief Fast approximation of arctangent for small angles
 *
 * Optimized for input range corresponding to angles -15 to 15 degrees
 * Uses modified Pade[3/2] approximation
 *
 * @param[in] x - Input value in range [-tan(15deg), tan(15deg)]
 * @return Angle in degrees, range [-15, 15]
 *
 * @note This is an internal helper function for fatan_deg
 */
float fatan_15deg(float x);

#endif /* _APPROXIMATIONS_H_ */
