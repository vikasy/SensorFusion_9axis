/*******************************************************************************
 * @file    algo_sf_orientation.c
 * @brief   Implementation of orientation calculation functions from sensor data
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#include "algo_sf_orientation.h"
#include "algo_sf_matrix.h"
#include "algo_sf_approx.h"

/****************************************************************************
 * Private Constants
 ****************************************************************************/

/* Quaternion threshold constants (already defined at top - kept for compatibility) */
#define SMALLQ0 0.01F		// limit of quaternion scalar component requiring special algorithm
#define CORRUPTQUAT 0.001F	// threshold for deciding rotation quaternion is corrupt
#define SMALLMODULUS 0.01F	// limit where rounding errors may appear

/* Rotation angle threshold constants */
#define ROT_ANGLE_THRESHOLD_02   0.02F    /* MacLaurin 3rd order threshold (rad^2) */
#define ROT_ANGLE_THRESHOLD_06   0.06F    /* MacLaurin 5th order threshold (rad^2) */

/* Rotation matrix trace constants */
#define ROT_TRACE_MIN            -1.0F    /* Minimum rotation matrix trace */
#define ROT_TRACE_MAX            3.0F     /* Maximum rotation matrix trace */

/* Quaternion scaling constants */
#define QUAT_SCALE_FACTOR_2      2.0F     /* Factor 2 for quaternion to matrix conversion */
#define QUAT_HALF                0.5F     /* Half value for quaternion computations */
#define QUAT_QUARTER             0.25F    /* Quarter value for quaternion matrix decomposition */

/* Floating point constants */
#define FLOAT_ZERO               0.0F     /* Zero value for floats */
#define FLOAT_ONE                1.0F     /* Unity value for floats */
#define FLOAT_NEG_ONE            -1.0F    /* Negative one for floats */

/* Angle constants in degrees */
#define ANGLE_ZERO_DEG           0.0F     /* Zero degrees */
#define ANGLE_90_DEG             90.0F    /* 90 degrees (right angle) */
#define ANGLE_NEG_90_DEG         -90.0F   /* Negative 90 degrees */
#define ANGLE_180_DEG            180.0F   /* 180 degrees (straight angle) */
#define ANGLE_NEG_180_DEG        -180.0F  /* Negative 180 degrees */
#define ANGLE_360_DEG            360.0F   /* 360 degrees (full circle) */

/****************************************************************************
 * 3DOF Tilt Functions
 ****************************************************************************/

/**
 * @brief Computes 3DOF tilt rotation matrix for NED (Aerospace) frame
 *
 * Calculates rotation matrix from accelerometer readings assuming gravity
 * is the only acceleration. Uses NED (North-East-Down) coordinate frame.
 * Handles gimbal lock at 90 deg pitch and freefall conditions.
 *
 * @param[out] fR - Output 3x3 rotation matrix
 * @param[in] fGp - Accelerometer readings [Gx, Gy, Gz] in g's
 * @return None
 *
 * @note Self-consistency twist occurs at 90 deg pitch in NED frame
 */
void f3DOFTiltNED(float fR[][3], float fGp[])
{
	// the NED self-consistency twist occurs at 90 deg pitch

	// local variables
	int16_t i;				// counter
	float fmodGxyz;			// modulus of the x, y, z accelerometer readings
	float fmodGyz;			// modulus of the y, z accelerometer readings
	float frecipmodGxyz;	// reciprocal of modulus
	float ftmp;				// scratch variable

							// compute the accelerometer squared magnitudes
	fmodGyz = fGp[CHY] * fGp[CHY] + fGp[CHZ] * fGp[CHZ];
	fmodGxyz = fmodGyz + fGp[CHX] * fGp[CHX];

	// check for freefall special case where no solution is possible
	if (fmodGxyz == FLOAT_ZERO)
	{
		f3x3matrixAeqI(fR);
		return;
	}

	// check for vertical up or down gimbal lock case
	if (fmodGyz == FLOAT_ZERO)
	{
		f3x3matrixAeqScalar(fR, FLOAT_ZERO);
		fR[CHY][CHY] = FLOAT_ONE;
		if (fGp[CHX] >= FLOAT_ZERO)
		{
			fR[CHX][CHZ] = FLOAT_ONE;
			fR[CHZ][CHX] = FLOAT_NEG_ONE;
		}
		else
		{
			fR[CHX][CHZ] = FLOAT_NEG_ONE;
			fR[CHZ][CHX] = FLOAT_ONE;
		}
		return;
	}

	// compute moduli for the general case
	fmodGyz = sqrtf(fmodGyz);
	fmodGxyz = sqrtf(fmodGxyz);
	frecipmodGxyz = FLOAT_ONE / fmodGxyz;
	ftmp = fmodGxyz / fmodGyz;

	// normalize the accelerometer reading into the z column
	for (i = CHX; i <= CHZ; i++)
	{
		fR[i][CHZ] = fGp[i] * frecipmodGxyz;
	}

	// construct x column of orientation matrix
	fR[CHX][CHX] = fmodGyz * frecipmodGxyz;
	fR[CHY][CHX] = -fR[CHX][CHZ] * fR[CHY][CHZ] * ftmp;
	fR[CHZ][CHX] = -fR[CHX][CHZ] * fR[CHZ][CHZ] * ftmp;

	// // construct y column of orientation matrix
	fR[CHX][CHY] = FLOAT_ZERO;
	fR[CHY][CHY] = fR[CHZ][CHZ] * ftmp;
	fR[CHZ][CHY] = -fR[CHY][CHZ] * ftmp;

	return;
}

/**
 * @brief Computes 3DOF tilt rotation matrix for Android frame
 *
 * The Android tilt matrix is mathematically identical to the NED tilt matrix.
 * Self-consistency twist occurs at 90 deg roll in Android frame.
 *
 * @param[out] fR - Output 3x3 rotation matrix
 * @param[in] fGp - Accelerometer readings [Gx, Gy, Gz] in g's
 * @return None
 */
void f3DOFTiltAndroid(float fR[][3], float fGp[])
{
	// the Android tilt matrix is mathematically identical to the NED tilt matrix
	// the Android self-consistency twist occurs at 90 deg roll
	f3DOFTiltNED(fR, fGp);
	return;
}

/****************************************************************************
 * Angle Extraction Functions
 ****************************************************************************/

/**
 * @brief Extracts Android angles in degrees from rotation matrix
 *
 * Computes roll (Phi), pitch (The), yaw (Psi), compass heading (Rho),
 * and tilt from vertical (Chi) from rotation matrix. Handles gimbal lock
 * at roll = ±90 degrees.
 *
 * @param[in] R - Input 3x3 rotation matrix
 * @param[out] pfPhiDeg - Roll angle -90 <= Phi <= 90 deg
 * @param[out] pfTheDeg - Pitch angle -180 <= The < 180 deg
 * @param[out] pfPsiDeg - Yaw angle 0 <= Psi < 360 deg
 * @param[out] pfRhoDeg - Compass heading 0 <= Rho < 360 deg (equals Psi)
 * @param[out] pfChiDeg - Tilt from vertical 0 <= Chi <= 180 deg
 * @return None
 */
void fAndroidAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg, float *pfTheDeg, float *pfPsiDeg,
	float *pfRhoDeg, float *pfChiDeg)
{
	// calculate the roll angle -90.0 <= Phi <= 90.0 deg
	*pfPhiDeg = fasin_deg(R[CHX][CHZ]);

	// calculate the pitch angle -180.0 <= The < 180.0 deg
	*pfTheDeg = fatan2_deg(-R[CHY][CHZ], R[CHZ][CHZ]);

	// map +180 pitch onto the functionally equivalent -180 deg pitch
	if (*pfTheDeg == ANGLE_180_DEG)
	{
		*pfTheDeg = ANGLE_NEG_180_DEG;
	}

	// calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
	if (*pfPhiDeg == ANGLE_90_DEG)
	{
		// vertical downwards gimbal lock case
		*pfPsiDeg = fatan2_deg(R[CHY][CHX], R[CHY][CHY]) - *pfTheDeg;
	}
	else if (*pfPhiDeg == ANGLE_NEG_90_DEG)
	{
		// vertical upwards gimbal lock case
		*pfPsiDeg = fatan2_deg(R[CHY][CHX], R[CHY][CHY]) + *pfTheDeg;
	}
	else
	{
		// // general case
		*pfPsiDeg = fatan2_deg(-R[CHX][CHY], R[CHX][CHX]);
	}

	// map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
	if (*pfPsiDeg < FLOAT_ZERO)
	{
		*pfPsiDeg += ANGLE_360_DEG;
	}

	// check for rounding errors mapping small negative angle to 360 deg
	if (*pfPsiDeg >= ANGLE_360_DEG)
	{
		*pfPsiDeg = FLOAT_ZERO;
	}

	// the compass heading angle Rho equals the yaw angle Psi
	// this definition is compliant with Motorola Xoom tablet behavior
	*pfRhoDeg = *pfPsiDeg;

	// calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg)
	*pfChiDeg = facos_deg(R[CHZ][CHZ]);

	return;
}

/****************************************************************************
 * Quaternion Conversion Functions
 ****************************************************************************/

/**
 * @brief Computes normalized rotation quaternion from rotation vector (deg)
 *
 * Converts rotation vector (axis*angle in degrees) to unit quaternion.
 * Uses small angle approximations for efficiency when angle < 0.245 rad.
 *
 * @param[out] pq - Output normalized quaternion
 * @param[in] rvecdeg - Rotation vector [rx, ry, rz] in degrees
 * @param[in] fscaling - Scaling factor (typically 1.0 or -1.0)
 * @return None
 *
 * @note Uses MacLaurin series up to 5th order for small angles
 */
void fQuaternionFromRotationVectorDeg(quaternion_t *pq, const float rvecdeg[], float fscaling)
{
	float fetadeg;			// rotation angle (deg)
	float fetarad;			// rotation angle (rad)
	float fetarad2;			// eta (rad)^2
	float fetarad4;			// eta (rad)^4
	float sinhalfeta;		// sin(eta/2)
	float fvecsq;			// q1^2+q2^2+q3^2
	float ftmp;				// scratch variable

							// compute the scaled rotation angle eta (deg) which can be both positve or negative
	fetadeg = fscaling * sqrtf(rvecdeg[CHX] * rvecdeg[CHX] + rvecdeg[CHY] * rvecdeg[CHY] + rvecdeg[CHZ] * rvecdeg[CHZ]);
	fetarad = fetadeg * DEG2RAD;
	fetarad2 = fetarad * fetarad;

	// calculate the sine and cosine using small angle approximations or exact
	// angles under sqrt(0.02)=0.141 rad is 8.1 deg and 1620 deg/s (=936deg/s in 3 axes) at 200Hz and 405 deg/s at 50Hz
	if (fetarad2 <= ROT_ANGLE_THRESHOLD_02)
	{
		// use MacLaurin series up to and including third order
		sinhalfeta = fetarad * (QUAT_HALF - ONEOVER48 * fetarad2);
	}
	else if (fetarad2 <= ROT_ANGLE_THRESHOLD_06)
	{
		// use MacLaurin series up to and including fifth order
		// angles under sqrt(0.06)=0.245 rad is 14.0 deg and 2807 deg/s (=1623deg/s in 3 axes) at 200Hz and 703 deg/s at 50Hz
		fetarad4 = fetarad2 * fetarad2;
		sinhalfeta = fetarad * (QUAT_HALF - ONEOVER48 * fetarad2 + ONEOVER3840 * fetarad4);
	}
	else
	{
		// use exact calculation
		sinhalfeta = (float)sinf(QUAT_HALF * fetarad);
	}

	// compute the vector quaternion components q1, q2, q3
	if (fetadeg != FLOAT_ZERO)
	{
		// general case with non-zero rotation angle
		ftmp = fscaling * sinhalfeta / fetadeg;
		pq->q1 = rvecdeg[CHX] * ftmp;		// q1 = nx * sin(eta/2)
		pq->q2 = rvecdeg[CHY] * ftmp;		// q2 = ny * sin(eta/2)
		pq->q3 = rvecdeg[CHZ] * ftmp;		// q3 = nz * sin(eta/2)
	}
	else
	{
		// zero rotation angle giving zero vector component
		pq->q1 = pq->q2 = pq->q3 = FLOAT_ZERO;
	}

	// compute the scalar quaternion component q0 by explicit normalization
	// taking care to avoid rounding errors giving negative operand to sqrt
	fvecsq = pq->q1 * pq->q1 + pq->q2 * pq->q2 + pq->q3 * pq->q3;
	if (fvecsq <= FLOAT_ONE)
	{
		// normal case
		pq->q0 = sqrtf(FLOAT_ONE - fvecsq);
	}
	else
	{
		// rounding errors are present
		pq->q0 = FLOAT_ZERO;
	}

	return;
}

/**
 * @brief Computes orientation quaternion from 3x3 rotation matrix
 *
 * Extracts quaternion from rotation matrix. Handles both general case
 * (q0 > SMALLQ0) and special case near 180 deg rotation (q0 small).
 *
 * @param[in] R - Input 3x3 rotation matrix (assumed normalized)
 * @param[out] pq - Output orientation quaternion
 * @return None
 *
 * @note Assumes rotation matrix is normalized (no explicit normalization)
 */
void fQuaternionFromRotationMatrix(float R[][3], quaternion_t *pq)
{
	float fq0sq;			// q0^2
	float recip4q0;			// 1/4q0

							// the quaternion is not explicitly normalized in this function on the assumption that it
							// is supplied with a normalized rotation matrix. if the rotation matrix is normalized then
							// the quaternion will also be normalized even if the case of small q0

							// get q0^2 and q0
	fq0sq = QUAT_QUARTER * (FLOAT_ONE + R[CHX][CHX] + R[CHY][CHY] + R[CHZ][CHZ]);
	pq->q0 = sqrtf(fabs(fq0sq));

	// normal case when q0 is not small meaning rotation angle not near 180 deg
	if (pq->q0 > SMALLQ0)
	{
		// calculate q1 to q3
		recip4q0 = QUAT_QUARTER / pq->q0;
		pq->q1 = recip4q0 * (R[CHY][CHZ] - R[CHZ][CHY]);
		pq->q2 = recip4q0 * (R[CHZ][CHX] - R[CHX][CHZ]);
		pq->q3 = recip4q0 * (R[CHX][CHY] - R[CHY][CHX]);
	} // end of general case
	else
	{
		// special case of near 180 deg corresponds to nearly symmetric matrix
		// which is not numerically well conditioned for division by small q0
		// instead get absolute values of q1 to q3 from leading diagonal
		pq->q1 = sqrtf(fabs(QUAT_HALF * (FLOAT_ONE + R[CHX][CHX]) - fq0sq));
		pq->q2 = sqrtf(fabs(QUAT_HALF * (FLOAT_ONE + R[CHY][CHY]) - fq0sq));
		pq->q3 = sqrtf(fabs(QUAT_HALF * (FLOAT_ONE + R[CHZ][CHZ]) - fq0sq));

		// correct the signs of q1 to q3 by examining the signs of differenced off-diagonal terms
		if ((R[CHY][CHZ] - R[CHZ][CHY]) < FLOAT_ZERO) pq->q1 = -pq->q1;
		if ((R[CHZ][CHX] - R[CHX][CHZ]) < FLOAT_ZERO) pq->q2 = -pq->q2;
		if ((R[CHX][CHY] - R[CHY][CHX]) < FLOAT_ZERO) pq->q3 = -pq->q3;
	} // end of special case

	return;
}

/**
 * @brief Computes rotation matrix from orientation quaternion
 *
 * Converts unit quaternion to 3x3 rotation matrix using optimized formula
 * with pre-computed products to minimize multiplications.
 *
 * @param[out] R - Output 3x3 rotation matrix
 * @param[in] pq - Input normalized quaternion
 * @return None
 *
 * @note Assumes input quaternion is normalized
 */
void fRotationMatrixFromQuaternion(float R[][3], const quaternion_t *pq)
{
	float f2q;
	float f2q0q0, f2q0q1, f2q0q2, f2q0q3;
	float f2q1q1, f2q1q2, f2q1q3;
	float f2q2q2, f2q2q3;
	float f2q3q3;

	// calculate products
	f2q = QUAT_SCALE_FACTOR_2 * pq->q0;
	f2q0q0 = f2q * pq->q0;
	f2q0q1 = f2q * pq->q1;
	f2q0q2 = f2q * pq->q2;
	f2q0q3 = f2q * pq->q3;
	f2q = QUAT_SCALE_FACTOR_2 * pq->q1;
	f2q1q1 = f2q * pq->q1;
	f2q1q2 = f2q * pq->q2;
	f2q1q3 = f2q * pq->q3;
	f2q = QUAT_SCALE_FACTOR_2 * pq->q2;
	f2q2q2 = f2q * pq->q2;
	f2q2q3 = f2q * pq->q3;
	f2q3q3 = QUAT_SCALE_FACTOR_2 * pq->q3 * pq->q3;

	// calculate the rotation matrix assuming the quaternion is normalized
	R[CHX][CHX] = f2q0q0 + f2q1q1 - FLOAT_ONE;
	R[CHX][CHY] = f2q1q2 + f2q0q3;
	R[CHX][CHZ] = f2q1q3 - f2q0q2;
	R[CHY][CHX] = f2q1q2 - f2q0q3;
	R[CHY][CHY] = f2q0q0 + f2q2q2 - FLOAT_ONE;
	R[CHY][CHZ] = f2q2q3 + f2q0q1;
	R[CHZ][CHX] = f2q1q3 + f2q0q2;
	R[CHZ][CHY] = f2q2q3 - f2q0q1;
	R[CHZ][CHZ] = f2q0q0 + f2q3q3 - FLOAT_ONE;

	return;
}

/****************************************************************************
 * Rotation Vector Functions
 ****************************************************************************/

/**
 * @brief Calculates rotation vector from rotation matrix
 *
 * Extracts axis-angle representation (rotation vector in degrees) from
 * rotation matrix. Handles 0 deg, 180 deg, and general rotation cases.
 *
 * @param[in] R - Input 3x3 rotation matrix
 * @param[out] rvecdeg - Output rotation vector [rx, ry, rz] in degrees
 * @return None
 *
 * @note Handles numerical issues near 0 and 180 degree rotations
 */
void fRotationVectorDegFromRotationMatrix(float R[][3], float rvecdeg[])
{
	float ftrace;			// trace of the rotation matrix
	float fetadeg;			// rotation angle eta (deg)
	float fmodulus;			// modulus of axis * angle vector = 2|sin(eta)|
	float ftmp;				// scratch variable

							// calculate the trace of the rotation matrix = 1+2cos(eta) in range -1 to +3
							// and eta (deg) in range 0 to 180 deg inclusive
							// checking for rounding errors that might take the trace outside this range
	ftrace = R[CHX][CHX] + R[CHY][CHY] + R[CHZ][CHZ];
	if (ftrace >= ROT_TRACE_MAX)
	{
		fetadeg = FLOAT_ZERO;
	}
	else if (ftrace <= ROT_TRACE_MIN)
	{
		fetadeg = ANGLE_180_DEG;
	}
	else
	{
		fetadeg = acosf(QUAT_HALF * (ftrace - FLOAT_ONE)) * RAD2DEG;
	}

	// set the rvecdeg vector to differences across the diagonal = 2*n*sin(eta)
	// and calculate its modulus equal to 2|sin(eta)|
	// the modulus approaches zero near 0 and 180 deg (when sin(eta) approaches zero)
	rvecdeg[CHX] = R[CHY][CHZ] - R[CHZ][CHY];
	rvecdeg[CHY] = R[CHZ][CHX] - R[CHX][CHZ];
	rvecdeg[CHZ] = R[CHX][CHY] - R[CHY][CHX];
	fmodulus = sqrtf(rvecdeg[CHX] * rvecdeg[CHX] + rvecdeg[CHY] * rvecdeg[CHY] + rvecdeg[CHZ] * rvecdeg[CHZ]);

	// normalize the rotation vector for general, 0 deg and 180 deg rotation cases
	if (fmodulus > SMALLMODULUS)
	{
		// general case away from 0 and 180 deg rotation
		ftmp = fetadeg / fmodulus;
		rvecdeg[CHX] *= ftmp;	// set x component to eta(deg) * nx
		rvecdeg[CHY] *= ftmp;	// set y component to eta(deg) * ny
		rvecdeg[CHZ] *= ftmp;	// set z component to eta(deg) * nz
	} // end of general case
	else if (ftrace >= FLOAT_ZERO)
	{
		// near 0 deg rotation (trace = 3): matrix is nearly identity matrix
		// R[CHY][CHZ]-R[CHZ][CHY]=2*nx*eta(rad) and similarly for other components
		ftmp = QUAT_HALF * RAD2DEG;
		rvecdeg[CHX] *= ftmp;
		rvecdeg[CHY] *= ftmp;
		rvecdeg[CHZ] *= ftmp;
	} // end of zero deg case
	else
	{
		// near 180 deg (trace = -1): matrix is nearly symmetric
		// calculate the absolute value of the components of the axis-angle vector
		rvecdeg[CHX] = ANGLE_180_DEG * sqrtf(fabs(QUAT_HALF * (R[CHX][CHX] + FLOAT_ONE)));
		rvecdeg[CHY] = ANGLE_180_DEG * sqrtf(fabs(QUAT_HALF * (R[CHY][CHY] + FLOAT_ONE)));
		rvecdeg[CHZ] = ANGLE_180_DEG * sqrtf(fabs(QUAT_HALF * (R[CHZ][CHZ] + FLOAT_ONE)));

		// correct the signs of the three components by examining the signs of differenced off-diagonal terms
		if ((R[CHY][CHZ] - R[CHZ][CHY]) < FLOAT_ZERO) rvecdeg[CHX] = -rvecdeg[CHX];
		if ((R[CHZ][CHX] - R[CHX][CHZ]) < FLOAT_ZERO) rvecdeg[CHY] = -rvecdeg[CHY];
		if ((R[CHX][CHY] - R[CHY][CHX]) < FLOAT_ZERO) rvecdeg[CHZ] = -rvecdeg[CHZ];

	} // end of 180 deg case

	return;
}

/**
 * @brief Computes rotation vector (deg) from rotation quaternion
 *
 * Converts unit quaternion to axis-angle representation. Maps rotation
 * angle onto range -180 deg <= eta < 180 deg.
 *
 * @param[in,out] pq - Input normalized quaternion (may be modified)
 * @param[out] rvecdeg - Output rotation vector [rx, ry, rz] in degrees
 * @return None
 */
void fRotationVectorDegFromQuaternion(quaternion_t *pq, float rvecdeg[])
{
	float fetarad;			// rotation angle (rad)
	float fetadeg;			// rotation angle (deg)
	float sinhalfeta;		// sin(eta/2)
	float ftmp;				// scratch variable

							// calculate the rotation angle in the range 0 <= eta < 360 deg
	if ((pq->q0 >= FLOAT_ONE) || (pq->q0 <= FLOAT_NEG_ONE))
	{
		// rotation angle is 0 deg or 2*180 deg = 360 deg = 0 deg
		fetarad = FLOAT_ZERO;
		fetadeg = FLOAT_ZERO;
	}
	else
	{
		// general case returning 0 < eta < 360 deg
		fetarad = QUAT_SCALE_FACTOR_2 * acosf(pq->q0);
		fetadeg = fetarad * RAD2DEG;
	}

	// map the rotation angle onto the range -180 deg <= eta < 180 deg
	if (fetadeg >= ANGLE_180_DEG)
	{
		fetadeg -= ANGLE_360_DEG;
		fetarad = fetadeg * DEG2RAD;
	}

	// calculate sin(eta/2) which will be in the range -1 to +1
	sinhalfeta = (float)sinf(QUAT_HALF * fetarad);

	// calculate the rotation vector (deg)
	if (sinhalfeta == FLOAT_ZERO)
	{
		// the rotation angle eta is zero and the axis is irrelevant
		rvecdeg[CHX] = rvecdeg[CHY] = rvecdeg[CHZ] = FLOAT_ZERO;
	}
	else
	{
		// general case with non-zero rotation angle
		ftmp = fetadeg / sinhalfeta;
		rvecdeg[CHX] = pq->q1 * ftmp;
		rvecdeg[CHY] = pq->q2 * ftmp;
		rvecdeg[CHZ] = pq->q3 * ftmp;
	}

	return;
}

/****************************************************************************
 * Quaternion Arithmetic Functions
 ****************************************************************************/

/**
 * @brief Computes quaternion product qA = qB * qC
 *
 * Calculates quaternion multiplication using standard formula.
 * Result stored in qA.
 *
 * @param[out] pqA - Output product quaternion qA = qB * qC
 * @param[in] pqB - First input quaternion (left operand)
 * @param[in] pqC - Second input quaternion (right operand)
 * @return None
 *
 * @note Quaternion multiplication is non-commutative: qB*qC ≠ qC*qB
 */
void qAeqBxC(quaternion_t *pqA, const quaternion_t *pqB, const quaternion_t *pqC)
{
	pqA->q0 = pqB->q0 * pqC->q0 - pqB->q1 * pqC->q1 - pqB->q2 * pqC->q2 - pqB->q3 * pqC->q3;
	pqA->q1 = pqB->q0 * pqC->q1 + pqB->q1 * pqC->q0 + pqB->q2 * pqC->q3 - pqB->q3 * pqC->q2;
	pqA->q2 = pqB->q0 * pqC->q2 - pqB->q1 * pqC->q3 + pqB->q2 * pqC->q0 + pqB->q3 * pqC->q1;
	pqA->q3 = pqB->q0 * pqC->q3 + pqB->q1 * pqC->q2 - pqB->q2 * pqC->q1 + pqB->q3 * pqC->q0;

	return;
}

/**
 * @brief Computes quaternion product qA = qA * qB (in-place)
 *
 * Multiplies qA by qB and stores result back in qA. Uses temporary
 * storage to handle in-place operation correctly.
 *
 * @param[in,out] pqA - Input/output quaternion, replaced by qA * qB
 * @param[in] pqB - Second input quaternion (right operand)
 * @return None
 */
void qAeqAxB(quaternion_t *pqA, const quaternion_t *pqB)
{
	quaternion_t qProd;

	// perform the quaternion product
	qProd.q0 = pqA->q0 * pqB->q0 - pqA->q1 * pqB->q1 - pqA->q2 * pqB->q2 - pqA->q3 * pqB->q3;
	qProd.q1 = pqA->q0 * pqB->q1 + pqA->q1 * pqB->q0 + pqA->q2 * pqB->q3 - pqA->q3 * pqB->q2;
	qProd.q2 = pqA->q0 * pqB->q2 - pqA->q1 * pqB->q3 + pqA->q2 * pqB->q0 + pqA->q3 * pqB->q1;
	qProd.q3 = pqA->q0 * pqB->q3 + pqA->q1 * pqB->q2 - pqA->q2 * pqB->q1 + pqA->q3 * pqB->q0;

	// copy the result back into qA
	*pqA = qProd;

	return;
}

/**
 * @brief Computes quaternion product conjg(qA) * qB
 *
 * Multiplies conjugate of qA by qB. Conjugate inverts rotation direction.
 * Used for relative rotations and coordinate frame transformations.
 *
 * @param[in] pqA - First input quaternion (conjugated before multiplication)
 * @param[in] pqB - Second input quaternion
 * @return Product quaternion conjg(qA) * qB
 */
quaternion_t qconjgAxB(const quaternion_t *pqA, const quaternion_t *pqB)
{
	quaternion_t qProd;

	qProd.q0 = pqA->q0 * pqB->q0 + pqA->q1 * pqB->q1 + pqA->q2 * pqB->q2 + pqA->q3 * pqB->q3;
	qProd.q1 = pqA->q0 * pqB->q1 - pqA->q1 * pqB->q0 - pqA->q2 * pqB->q3 + pqA->q3 * pqB->q2;
	qProd.q2 = pqA->q0 * pqB->q2 + pqA->q1 * pqB->q3 - pqA->q2 * pqB->q0 - pqA->q3 * pqB->q1;
	qProd.q3 = pqA->q0 * pqB->q3 - pqA->q1 * pqB->q2 + pqA->q2 * pqB->q1 - pqA->q3 * pqB->q0;

	return qProd;
}

/**
 * @brief Normalizes rotation quaternion and ensures q0 is non-negative
 *
 * Normalizes quaternion to unit magnitude. If magnitude is below
 * CORRUPTQUAT threshold, returns identity quaternion. Flips signs if
 * q0 is negative to maintain positive scalar component convention.
 *
 * @param[in,out] pqA - Input quaternion, normalized on output
 * @return None
 */
void fqAeqNormqA(quaternion_t *pqA)
{
	float fNorm;					// quaternion Norm

									// calculate the quaternion Norm
	fNorm = sqrtf(pqA->q0 * pqA->q0 + pqA->q1 * pqA->q1 + pqA->q2 * pqA->q2 + pqA->q3 * pqA->q3);
	if (fNorm > CORRUPTQUAT)
	{
		// general case
		fNorm = FLOAT_ONE / fNorm;
		pqA->q0 *= fNorm;
		pqA->q1 *= fNorm;
		pqA->q2 *= fNorm;
		pqA->q3 *= fNorm;
	}
	else
	{
		// return with identity quaternion since the quaternion is corrupted
		pqA->q0 = FLOAT_ONE;
		pqA->q1 = pqA->q2 = pqA->q3 = FLOAT_ZERO;
	}

	// correct a negative scalar component if the function was called with negative q0
	if (pqA->q0 < FLOAT_ZERO)
	{
		pqA->q0 = -pqA->q0;
		pqA->q1 = -pqA->q1;
		pqA->q2 = -pqA->q2;
		pqA->q3 = -pqA->q3;
	}

	return;
}

/**
 * @brief Sets quaternion to unit (identity) quaternion
 *
 * Initializes quaternion to identity: q = [1, 0, 0, 0] representing
 * zero rotation.
 *
 * @param[out] pqA - Output quaternion set to identity
 * @return None
 */
void fqAeq1(quaternion_t *pqA)
{
	pqA->q0 = FLOAT_ONE;
	pqA->q1 = pqA->q2 = pqA->q3 = FLOAT_ZERO;

	return;
}
