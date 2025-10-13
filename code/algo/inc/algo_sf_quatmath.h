/*******************************************************************************
 * @file    algo_sf_quatmath.h
 * @brief   Quaternion mathematics functions for 3D orientation representation
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#ifndef _ALGO_SF_QUATMATH_H_
#define _ALGO_SF_QUATMATH_H_

#include "algo_sf_types.h"

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/**
 * @brief Quaternion structure using double precision
 *
 * Represents orientation as q = q0 + q1*i + q2*j + q3*k where:
 * - q0 is the scalar (real) component
 * - q1, q2, q3 are the vector (imaginary) components
 */
typedef struct quaternion_double
{
	double q0;	// scalar component
	double q1;	// x vector component
	double q2;	// y vector component
	double q3;	// z vector component
} quaternion_double_t;

/****************************************************************************
 * Quaternion Constants
 ****************************************************************************/

#define MAX_POS_PITCH_DEG    179.9999    /* Maximum positive pitch angle (deg) */

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

/**
 * @brief Converts a quaternion orientation to a rotation matrix
 *
 * RotMtx is actually coordination frame rotation matrix such that
 * w = RotMtx*v is same as w = Quat'*v*Quat
 *
 * @param[in] Quat - Normalized quaternion representing orientation
 * @param[out] RotMtx - 3x3 rotation matrix output
 * @return None
 */
void Quat2RotMtx(const quaternion_double_t *Quat, double RotMtx[3][3]);

/**
 * @brief Converts a rotation matrix orientation to a quaternion
 *
 * Extracts quaternion representation from a 3x3 rotation matrix using
 * square root decomposition method.
 *
 * @param[in] RotMtx - 3x3 rotation matrix input
 * @param[out] Quat - Output quaternion representing the same orientation
 * @return None
 */
void RotMtx2Quat(const double        RotMtx[3][3],
                 quaternion_double_t *Quat);

/**
 * @brief Normalize a quaternion to unit magnitude
 *
 * Normalizes quaternion to unit magnitude (qmag = 1). If q0 is negative,
 * it makes it positive without changing the rotation represented.
 *
 * @param[in] Quat - Input quaternion to be normalized
 * @param[out] NormQuat - Output normalized quaternion with |q| = 1
 * @return None
 *
 * @note If input magnitude is below EPSILON, returns identity quaternion
 */
void QuatNormal(const quaternion_double_t *Quat, quaternion_double_t *NormQuat);

/**
 * @brief Calculates product of two quaternions
 *
 * Computes quaternion multiplication: pq = p * q
 * pq = [p0q0-P.Q, PxQ + p0Q + q0P]
 * where p = [p0, P] and q = [q0, Q]
 *
 * @param[in] pQuat - First quaternion (left operand)
 * @param[in] qQuat - Second quaternion (right operand)
 * @param[out] pqQuat - Product quaternion pq = p * q
 * @return None
 *
 * @note Quaternion multiplication is non-commutative
 */
void QuatProduct(const quaternion_double_t *pQuat, const quaternion_double_t *qQuat, quaternion_double_t *pqQuat);

/**
 * @brief Time integrate quaternion using zeroth order method
 *
 * Rotates given quaternion by the provided angular rate over one time interval.
 * Uses zeroth order (Euler) integration method.
 *
 * @param[in] QuatPre - Previous quaternion at time t
 * @param[in] ang_rate_dps - Instantaneous angular rate vector (deg/sec)
 * @param[in] deltaT - Time interval between samples (seconds)
 * @param[out] QuatInt - Integrated quaternion at time t + deltaT
 * @return None
 *
 * @note Uses small angle approximation for angles < sqrt(EPSILON) rad
 */
void QuatIntegrate(const quaternion_double_t *QuatPre, const double ang_rate_dps[3], const double deltaT, quaternion_double_t *QuatInt);

/**
 * @brief Time integrate quaternion using first order method
 *
 * Rotates given quaternion using first order (trapezoidal) integration.
 * Accounts for coriolis cross-product term for improved accuracy.
 *
 * @param[in] QuatPre - Previous quaternion at time t
 * @param[in] ang_rate1_dps - Current instantaneous angular rate (deg/sec)
 * @param[in] ang_rate2_dps - Previous instantaneous angular rate (deg/sec)
 * @param[in] deltaT - Time interval between this and previous samples (sec)
 * @param[out] QuatInt - Integrated quaternion at time t + deltaT
 * @return None
 *
 * @note More accurate than QuatIntegrate() for high rotation rates
 */
void QuatIntegrate1st(const quaternion_double_t *QuatPre,
	const double              ang_rate1_dps[3],
	const double              ang_rate2_dps[3],
	const double              deltaT,
	quaternion_double_t       *QuatInt);

/**
 * @brief Convert rotation matrix to Euler angles (pitch, yaw, roll)
 *
 * Extracts Euler angles from rotation matrix with gimbal lock handling.
 * Output ranges: -90 <= phi <= 90, -180 <= theta < 180, 0 <= psi < 360
 *
 * @param[in] RotMtx - 3x3 rotation matrix
 * @param[out] theta - Pitch angle (degrees)
 * @param[out] phi - Roll angle (degrees)
 * @param[out] psi - Yaw angle (degrees)
 * @return None
 *
 * @note Handles gimbal lock at phi = ±90 degrees using previous values
 */
void RotMtx2Angles(const double RotMtx[3][3],
	double       *theta,
	double       *phi,
	double       *psi);

/**
 * @brief Safe arctangent function with input validation
 *
 * Computes atan2(y,x) with checks for infinities and near-zero values.
 * Returns validity flag to indicate if result is reliable.
 *
 * @param[in] y - Y coordinate value
 * @param[in] x - X coordinate value
 * @param[out] vld - Validity flag: 1 if valid, 0 if invalid
 * @return Arctangent value in radians, or 0.0 if invalid
 *
 * @note Handles special cases: x≈0, y≈0, and infinity
 */
double atan2_safe(double    y,
	double    x,
	uint32_t  *vld);

#endif /* _ALGO_SF_QUATMATH_H_ */
