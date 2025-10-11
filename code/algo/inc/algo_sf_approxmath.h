#ifndef __ALGO_SF_APPROXMATH_H__
#define __ALGO_SF_APPROXMATH_H__

/*****
* Author: Vikas Yadav
* Date: 2020
*/
#include "algo_sf_types.h"

#define TAN15DEG   0.267949192431123
#define TAN30DEG   0.577350269189626

// Input: -Inf < x_rad < Inf
// Output: -1.0 <= sinx, cosx <= 1.0
void sincos_approx( float x_rad, float *psinx, float *pcosx );

// Input: -1.0 <= x <= 1.0
// Output: -90 <= asinx <= 90 deg
float asin_approx( float x );

// Input: -1.0 <= x <= 1.0
// Output: 0 <= acosx <= 180 deg
float acos_approx( float x );

// Input: -Inf < x < Inf
// Output: -90 < atanx < 90 deg
float atan_approx( float x );

// Input: -Inf < x, y < Inf
// Output: -180 < atan2x <= 180 deg
float atan2_approx( float y, float x );


#endif /* __ALGO_SF_APPROXMATH_H__ */
