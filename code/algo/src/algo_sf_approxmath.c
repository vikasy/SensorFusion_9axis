/*******************************************************************************
 * @file    algo_sf_approxmath.c
 * @brief   Implementation of fast trigonometric approximation functions
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#include "algo_sf_approxmath.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static float cos_1stquad(float x_rad);
static float sin_1stquad(float x_rad);
static float atan_below15(float x);

/****************************************************************************
 * Private Constants
 ****************************************************************************/

/* Floating point constants */
#define FLOAT_ZERO               0.0F     /* Zero value */
#define FLOAT_ONE                1.0F     /* Unity value */
#define FLOAT_NEG_ONE            -1.0F    /* Negative one */

/* Angle constants in degrees */
#define ANGLE_ZERO_DEG           0.0F     /* Zero degrees */
#define ANGLE_90_DEG             90.0F    /* 90 degrees (right angle) */
#define ANGLE_NEG_90_DEG         -90.0F   /* Negative 90 degrees */
#define ANGLE_180_DEG            180.0F   /* 180 degrees (straight angle) */
#define ANGLE_NEG_180_DEG        -180.0F  /* Negative 180 degrees */

/****************************************************************************
 * Cosine Approximation for First Quadrant
 ****************************************************************************/

/**
 * @brief Computes cosine approximation for first quadrant angles
 *
 * Uses 5th order polynomial approximation for 0 <= x <= Pi/2.
 * Coefficients optimized for minimal error in first quadrant.
 *
 * @param[in] x_rad - Input angle in radians (0 <= x_rad <= Pi/2)
 * @return Cosine value (0.0 <= cosx <= 1.0)
 *
 * @note Max error approximately 1e-7 in first quadrant
 */
static float cos_1stquad(float x_rad)
{
#define c1  0.999999953464
#define c2 -0.4999999053455
#define c3  0.0416635846769
#define c4 -0.0013853704264
#define c5  0.000023233
//#define c1  0.99999999999925182
//#define c2 -0.49999999997024012
//#define c3  0.041666666473384543
//#define c4 -0.001388888418000423
//#define c5  0.0000248010406484558
//#define c6  0.0000002752469638432
//#define c7  0.0000000019907856854

    float y, x2;

    x2 = x_rad*x_rad;
    y = (c1+x2*(c2+x2*(c3+x2*(c4+c5*x2))));

	return y;

#undef c1
#undef c2
#undef c3
#undef c4
#undef c5
}

/****************************************************************************
 * Sine Approximation for First Quadrant
 ****************************************************************************/

/**
 * @brief Computes sine approximation for first quadrant angles
 *
 * Calculates sine from cosine using identity: sin²(x) + cos²(x) = 1
 * Valid for 0 <= x <= Pi/2 where both sin and cos are non-negative.
 *
 * @param[in] x_rad - Input angle in radians (0 <= x_rad <= Pi/2)
 * @return Sine value (0.0 <= sinx <= 1.0)
 */
static float sin_1stquad(float x_rad)
{
    float y;
	y = cos_1stquad(x_rad);

	return sqrt(FLOAT_ONE - y*y);

}

/****************************************************************************
 * Combined Sine and Cosine Approximation
 ****************************************************************************/

/**
 * @brief Computes both sine and cosine approximations for any angle
 *
 * Reduces input angle to first quadrant using modulo and symmetry.
 * Applies appropriate sign changes based on quadrant. More efficient
 * than calling sin and cos separately.
 *
 * @param[in] x_rad - Input angle in radians (-Inf < x_rad < Inf)
 * @param[out] psinx - Pointer to output sine value (-1.0 <= sinx <= 1.0)
 * @param[out] pcosx - Pointer to output cosine value (-1.0 <= cosx <= 1.0)
 * @return None
 */
void sincos_approx( float x_rad, float *psinx, float *pcosx )
{

	float xin;
	float sign_c, sign_s;

	sign_c = FLOAT_ONE;
	sign_s = FLOAT_ONE;

	// adjust input angle to bring it within 1st quadrant
	xin = fmodf(x_rad, PI_TIMES_2);
	if (xin < FLOAT_ZERO) {
		xin = -xin;
		sign_s = FLOAT_NEG_ONE;
	}
    if( xin >= PI_OVER_2 && xin < PI ) {
		xin = PI - xin;
		sign_c = FLOAT_NEG_ONE*sign_c;
	}
    else if( xin >= PI && xin < (PI + PI_OVER_2) ) {
		xin = xin - PI;
		sign_c = FLOAT_NEG_ONE*sign_c;
		sign_s = FLOAT_NEG_ONE*sign_s;
	}
    else if( xin >= (PI + PI_OVER_2) && xin < PI_TIMES_2 ) {
		xin = PI_TIMES_2 - xin;
		sign_s = FLOAT_NEG_ONE*sign_s;
	}

	*pcosx = sign_c*cos_1stquad( xin );
	*psinx = sign_s*sin_1stquad( xin );

}

/****************************************************************************
 * Arctangent Approximation for Small Angles
 ****************************************************************************/

/**
 * @brief Computes arctangent approximation for small angles
 *
 * Uses 5th order polynomial approximation valid for -tan(15°) <= x <= tan(15°).
 * Coefficients optimized for this restricted range to achieve high accuracy.
 *
 * @param[in] x - Input value (-TAN15DEG <= x <= TAN15DEG)
 * @return Arctangent in radians (-15 <= atanx <= 15 deg equivalent)
 *
 * @note Max error approximately 2e-7 degrees in valid range
 */
static float atan_below15(float x)
{
#define c1  -0.3333333333297
#define c2   0.1999999963100
#define c3  -0.1428565386578
#define c4   0.1110747892853
#define c5  -0.0899143448436

    float y, x2;

    x2 = x*x;
    y = x*(FLOAT_ONE + x2*(c1 + x2*(c2 + x2*(c3 + x2*(c4 + x2*c5)))));

	return y;

#undef c1
#undef c2
#undef c3
#undef c4
#undef c5
}

/****************************************************************************
 * Inverse Trigonometric Functions
 ****************************************************************************/

/**
 * @brief Computes arcsine approximation
 *
 * Uses identity: asin(x) = atan(x/sqrt(1-x²)) for |x| < 1.
 * Handles boundary cases x = ±1 explicitly to avoid division issues.
 *
 * @param[in] x - Input value (-1.0 <= x <= 1.0)
 * @return Arcsine in degrees (-90 <= asinx <= 90 deg)
 */
float asin_approx( float x )
{
	float asinx_deg = FLOAT_ZERO;

	if (x >= FLOAT_ONE)
		asinx_deg = ANGLE_90_DEG;
	else if (x <= FLOAT_NEG_ONE)
		asinx_deg = ANGLE_NEG_90_DEG;
	else
		asinx_deg = atan_approx( x/sqrt(FLOAT_ONE - x*x) );

	return asinx_deg;

}

/**
 * @brief Computes arccosine approximation
 *
 * Uses identity: acos(x) = atan(sqrt(1-x²)/x) + offset based on sign of x.
 * Handles special cases to avoid numerical issues and maintain accuracy.
 *
 * @param[in] x - Input value (-1.0 <= x <= 1.0)
 * @return Arccosine in degrees (0 <= acosx <= 180 deg)
 */
float acos_approx( float x )
{
	float acosx_deg = FLOAT_ZERO;

	if (x == FLOAT_ZERO) {
		acosx_deg = ANGLE_90_DEG;
	}
	else if (x >= FLOAT_ONE) {
		acosx_deg = FLOAT_ZERO;
	}
	else if (x <= FLOAT_NEG_ONE) {
		acosx_deg = ANGLE_180_DEG;
	}
	else if (x > FLOAT_ZERO) {
		acosx_deg = atan_approx( sqrt(FLOAT_ONE - x*x)/x );
	}
	else {
		acosx_deg = atan_approx( sqrt(FLOAT_ONE - x*x)/x ) + ANGLE_180_DEG;
	}

	return acosx_deg;

}

/**
 * @brief Computes arctangent approximation with range reduction
 *
 * Uses piecewise approximation strategy:
 * 1. For |x| > 1: use identity atan(x) = π/2 - atan(1/x)
 * 2. For tan(15°) < |x| <= 1: shift by tan(30°) and use addition formula
 * 3. For |x| <= tan(15°): use direct polynomial approximation
 *
 * @param[in] x - Input value (-Inf < x < Inf)
 * @return Arctangent in degrees (-90 < atanx < 90 deg)
 */
float atan_approx( float x )
{
	float xin;
	float sign;
	float shift1, shift2;
	float y, atanx_deg;

	atanx_deg = FLOAT_ZERO;
	xin = x;
	sign = FLOAT_ONE;
	shift1 = FLOAT_ZERO;
	shift2 = FLOAT_ZERO;

	if( xin == FLOAT_ZERO ) {
		atanx_deg = FLOAT_ZERO;
		return atanx_deg;
	}
	else if( xin < FLOAT_ZERO ) {
	    xin = -xin;
		sign = FLOAT_NEG_ONE;
	}

	if( xin > FLOAT_ONE ) {
		xin = FLOAT_ONE/xin;
		shift1 = sign*PI_OVER_2;
		sign = FLOAT_NEG_ONE*sign;
	}

	if( xin > TAN15DEG ) {
		xin = (xin - TAN30DEG)/(FLOAT_ONE + TAN30DEG*x);
		shift2 = PI_OVER_6;
	}
	y = atan_below15( xin );
	y = sign*(y + shift2) + shift1;
	atanx_deg = y*RAD2DEG;

	return atanx_deg;

}

/**
 * @brief Computes two-argument arctangent approximation
 *
 * Determines quadrant based on signs of x and y, then calls atan_approx.
 * Returns angle in range (-180, 180] degrees. Handles all special cases
 * including x=0, y=0, and both zero.
 *
 * @param[in] y - Y coordinate (-Inf < y < Inf)
 * @param[in] x - X coordinate (-Inf < x < Inf)
 * @return Two-argument arctangent in degrees (-180 < atan2x <= 180 deg)
 */
float atan2_approx( float y, float x )
{
	float atan2_deg = FLOAT_ZERO;

	if( x > FLOAT_ZERO ) {
		atan2_deg = atan_approx(y/x);
	}
	else if( x == FLOAT_ZERO ) {
		if( y < FLOAT_ZERO ) {
		    atan2_deg = ANGLE_NEG_90_DEG;
		}
		else if( y > FLOAT_ZERO ) {
		    atan2_deg = ANGLE_90_DEG;
	    }
		else {
		    atan2_deg = FLOAT_ZERO;
		}
	}
	else if( y >= FLOAT_ZERO ) {
		atan2_deg = atan_approx(y/x) + ANGLE_180_DEG;
	}
	else if( y < FLOAT_ZERO ) {
		atan2_deg = atan_approx(y/x) - ANGLE_180_DEG;
	}

	return atan2_deg;

}
