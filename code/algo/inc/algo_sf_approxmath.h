/*******************************************************************************
 * @file    algo_sf_approxmath.h
 * @brief   Fast approximation functions for trigonometric operations
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#ifndef __ALGO_SF_APPROXMATH_H__
#define __ALGO_SF_APPROXMATH_H__

#include "algo_sf_types.h"

/****************************************************************************
 * Trigonometric Approximation Constants
 ****************************************************************************/

#define TAN15DEG   0.267949192431123    /* tan(15 degrees) for piecewise approximation */
#define TAN30DEG   0.577350269189626    /* tan(30 degrees) for piecewise approximation */

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

/**
 * @brief Computes sine and cosine approximations for any angle
 *
 * Uses polynomial approximations with input angle reduction to first quadrant.
 * Efficient alternative to standard library sin/cos for embedded systems.
 *
 * @param[in] x_rad - Input angle in radians (-Inf < x_rad < Inf)
 * @param[out] psinx - Pointer to output sine value (-1.0 <= sinx <= 1.0)
 * @param[out] pcosx - Pointer to output cosine value (-1.0 <= cosx <= 1.0)
 * @return None
 *
 * @note Uses 5th order polynomial approximation with max error ~1e-7
 */
void sincos_approx( float x_rad, float *psinx, float *pcosx );

/**
 * @brief Computes arcsine approximation
 *
 * Calculates inverse sine using polynomial approximations and atan_approx.
 * Handles domain boundaries explicitly for robustness.
 *
 * @param[in] x - Input value (-1.0 <= x <= 1.0)
 * @return Arcsine in degrees (-90 <= asinx <= 90 deg)
 *
 * @note Clamps input to valid domain [-1.0, 1.0]
 */
float asin_approx( float x );

/**
 * @brief Computes arccosine approximation
 *
 * Calculates inverse cosine using atan_approx with identity:
 * acos(x) = atan(sqrt(1-x²)/x) for x>0, plus offset for x<0.
 *
 * @param[in] x - Input value (-1.0 <= x <= 1.0)
 * @return Arccosine in degrees (0 <= acosx <= 180 deg)
 *
 * @note Handles special cases: x=0, x=1, x=-1 explicitly
 */
float acos_approx( float x );

/**
 * @brief Computes arctangent approximation
 *
 * Uses piecewise polynomial approximation with range reduction.
 * Divides input into regions: |x|<tan(15°), tan(15°)<|x|<1, |x|>1
 * to maintain accuracy across full range.
 *
 * @param[in] x - Input value (-Inf < x < Inf)
 * @return Arctangent in degrees (-90 < atanx < 90 deg)
 *
 * @note Max error approximately 2e-7 degrees across full range
 */
float atan_approx( float x );

/**
 * @brief Computes two-argument arctangent approximation
 *
 * Calculates atan2(y,x) using atan_approx with quadrant handling.
 * More robust than atan(y/x) as it handles x=0 and maintains
 * correct signs in all quadrants.
 *
 * @param[in] y - Y coordinate (-Inf < y < Inf)
 * @param[in] x - X coordinate (-Inf < x < Inf)
 * @return Two-argument arctangent in degrees (-180 < atan2x <= 180 deg)
 *
 * @note Handles all quadrants and special cases (x=0, y=0)
 */
float atan2_approx( float y, float x );


#endif /* __ALGO_SF_APPROXMATH_H__ */
