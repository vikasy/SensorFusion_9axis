/*******************************************************************************
 * @file    algo_sf_approx.c
 * @brief   Fast approximation function implementations for trigonometric operations
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#include "algo_sf_approx.h"

/****************************************************************************
 * Private Constants
 ****************************************************************************/

/* Angle constants in degrees */
#define ANGLE_ZERO_DEG      0.0F
#define ANGLE_90_DEG        90.0F
#define ANGLE_180_DEG       180.0F
#define ANGLE_NEG_90_DEG    -90.0F
#define ANGLE_NEG_180_DEG   -180.0F
#define ANGLE_15_DEG        15.0F
#define ANGLE_30_DEG        30.0F

/* Floating point constants */
#define FLOAT_ZERO          0.0F
#define FLOAT_ONE           1.0F
#define FLOAT_NEG_ONE       -1.0F

/* Tangent values for angle mapping */
#define TAN15DEG            0.26794919243F    /* tan(15 deg) = 2 - sqrt(3) */
#define TAN30DEG            0.57735026919F    /* tan(30 deg) = 1/sqrt(3) */

/* Pade approximation coefficients for fatan_15deg */
#define PADE_A              96.644395816F     /* Optimized Pade[3/2] coefficient (theoretical: 5/3*180/PI=95.49296) */
#define PADE_B              25.086941612F     /* Optimized Pade[3/2] coefficient (theoretical: 4/9*180/PI=25.46479) */
#define PADE_C              1.6867633134F     /* Optimized Pade[3/2] coefficient (theoretical: 5/3=1.66667) */

/****************************************************************************
 * Inverse Trigonometric Functions
 ****************************************************************************/

/**
 * @brief Fast approximation of arcsine in degrees
 *
 * Provides fast computation of arcsin(x) with maximum error of 10.29E-6 degrees.
 * Uses arctangent approximation internally: asin(x) = atan(x/sqrt(1-x^2))
 *
 * @param[in] x - Input value in range [-1, 1]
 * @return Angle in degrees, range [-90, 90]
 *
 * @note Input values outside [-1, 1] are clamped to boundaries
 */
float fasin_deg(float x)
{
	// for robustness, check for invalid argument
	if (x >= FLOAT_ONE) return ANGLE_90_DEG;
	if (x <= FLOAT_NEG_ONE) return ANGLE_NEG_90_DEG;

	// call the atan which will return an angle in the correct range -90 to 90 deg
	// this line cannot fail from division by zero or negative square root since |x| < 1
	return (fatan_deg(x / sqrtf(FLOAT_ONE - x * x)));
}

/**
 * @brief Fast approximation of arccosine in degrees
 *
 * Provides fast computation of arccos(x) with maximum error of 14.67E-6 degrees.
 * Uses arctangent approximation internally: acos(x) = atan(sqrt(1-x^2)/x)
 *
 * @param[in] x - Input value in range [-1, 1]
 * @return Angle in degrees, range [0, 180]
 *
 * @note Input values outside [-1, 1] are clamped to boundaries
 */
float facos_deg(float x)
{
	// for robustness, check for invalid arguments
	if (x >= FLOAT_ONE) return ANGLE_ZERO_DEG;
	if (x <= FLOAT_NEG_ONE) return ANGLE_180_DEG;

	// call the atan which will return an angle in the incorrect range -90 to 90 deg
	// these lines cannot fail from division by zero or negative square root
	if (x == FLOAT_ZERO) return ANGLE_90_DEG;
	if (x > FLOAT_ZERO) return fatan_deg((sqrtf(FLOAT_ONE - x * x) / x));
	return ANGLE_180_DEG + fatan_deg((sqrtf(FLOAT_ONE - x * x) / x));
}

/**
 * @brief Fast approximation of arctangent in degrees
 *
 * Provides fast computation of arctan(x) with maximum error of 9.84E-6 degrees.
 * Uses range reduction and mapping to optimize for small angle approximation.
 *
 * @param[in] x - Input value (any real number)
 * @return Angle in degrees, range [-90, 90]
 *
 * @note Uses multiple range mapping transformations for optimal accuracy
 */
float fatan_deg(float x)
{
	float fangledeg;			// computed angle (deg)
	uint32_t ixisnegative;		// argument x is negative
	uint32_t ixexceeds1;		// argument x is greater than 1.0
	uint32_t ixmapped;			// argument in range tan(15 deg) to tan(45 deg)=1.0

	// reset all flags
	ixisnegative = ixexceeds1 = ixmapped = 0;

	// test for negative argument to allow use of tan(-x)=-tan(x)
	if (x < FLOAT_ZERO)
	{
		x = -x;
		ixisnegative = 1;
	}

	// test for argument above 1 to allow use of atan(x)=pi/2-atan(1/x)
	if (x > FLOAT_ONE)
	{
		x = FLOAT_ONE / x;
		ixexceeds1 = 1;
	}

	// at this point, x is in the range 0 to 1 inclusive
	// map argument onto range -tan(15 deg) to tan(15 deg)
	// using tan(angle-30deg) = (tan(angle)-tan(30deg)) / (1 + tan(angle)tan(30deg))
	// tan(15deg) maps to tan(-15 deg) = -tan(15 deg)
	// 1. maps to (sqrt(3) - 1) / (sqrt(3) + 1) = 2 - sqrt(3) = tan(15 deg)
	if (x > TAN15DEG)
	{
		x = (x - TAN30DEG)/(FLOAT_ONE + TAN30DEG * x);
		ixmapped = 1;
	}

	// call the atan estimator to obtain -15 deg <= angle <= 15 deg
	fangledeg = fatan_15deg(x);

	// undo the distortions applied earlier to obtain -90 deg <= angle <= 90 deg
	if (ixmapped) fangledeg += ANGLE_30_DEG;
	if (ixexceeds1) fangledeg = ANGLE_90_DEG - fangledeg;
	if (ixisnegative) fangledeg = -fangledeg;
	
	return (fangledeg);
}

/**
 * @brief Fast approximation of atan2 in degrees
 *
 * Provides fast computation of atan2(y,x) with maximum error of 14.58E-6 degrees.
 * Returns the angle of the point (x,y) from the positive x-axis.
 *
 * @param[in] y - Y coordinate
 * @param[in] x - X coordinate
 * @return Angle in degrees, range [-180, 180]
 *
 * @note Handles all quadrants and special cases (x=0, y=0)
 */
float fatan2_deg(float y, float x)
{
	// check for zero x to avoid division by zero
	if (x == FLOAT_ZERO)
	{
		// return 90 deg for positive y
		if (y > FLOAT_ZERO) return ANGLE_90_DEG;
		// return -90 deg for negative y
		if (y < FLOAT_ZERO) return ANGLE_NEG_90_DEG;
		// otherwise y = 0.0 and return 0 deg (invalid arguments)
		return ANGLE_ZERO_DEG;
	}

	// from here onwards, x is guaranteed to be non-zero
	// compute atan2 for quadrant 1 (0 to 90 deg) and quadrant 4 (-90 to 0 deg)
	if (x > FLOAT_ZERO) return (fatan_deg(y / x));
	// compute atan2 for quadrant 2 (90 to 180 deg)
	if ((x < FLOAT_ZERO) && (y > FLOAT_ZERO)) return (ANGLE_180_DEG + fatan_deg(y / x));
	// compute atan2 for quadrant 3 (-180 to -90 deg)
	return (ANGLE_NEG_180_DEG + fatan_deg(y / x));

}

/**
 * @brief Fast approximation of arctangent for small angles (±15 degrees)
 *
 * Uses modified Pade[3/2] rational approximation optimized for the range
 * corresponding to angles between -15 and +15 degrees.
 *
 * @param[in] x - Input value in range [-tan(15deg), tan(15deg)]
 * @return Angle in degrees, range [-15, 15]
 *
 * @note This is an internal helper function called by fatan_deg
 * @note Coefficients are empirically optimized for minimum error
 */
float fatan_15deg(float x)
{
	float x2;			// x^2

	// compute the approximation to the inverse tangent
	// the function is anti-symmetric as required for positive and negative arguments
	x2 = x * x;
	return (x * (PADE_A + x2 * PADE_B) / (PADE_C + x2));
}
