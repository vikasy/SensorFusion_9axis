/*******************************************************************************
 * @file    algo_sf_quatmath.c
 * @brief   Implementation of quaternion mathematics functions
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#include "algo_sf_quatmath.h"

/****************************************************************************
 * Private Constants
 ****************************************************************************/

/* Quaternion scaling constants */
#define QUAT_SCALE_TWO           2.0      /* Factor for quaternion to rotation matrix conversion */
#define QUAT_HALF_ANGLE          0.5      /* Half angle divisor for sin/cos computations */
#define QUAT_QUARTER             0.25     /* Quarter value for matrix decomposition */
#define QUAT_EIGHTH              0.125    /* One-eighth for small angle approximation */

/* Quaternion identity values */
#define QUAT_IDENTITY_SCALAR     1.0      /* Identity quaternion scalar part q0 = 1 */
#define QUAT_IDENTITY_VECTOR     0.0      /* Identity quaternion vector parts q1=q2=q3 = 0 */

/* Angle constants in degrees */
#define ANGLE_ZERO_DEG           0.0      /* Zero degrees */
#define ANGLE_NEG_180_DEG        -180.0   /* Negative 180 degrees */
#define ANGLE_180_DEG            180.0    /* 180 degrees (straight angle) */
#define ANGLE_360_DEG            360.0    /* 360 degrees (full circle) */
#define ANGLE_INVALID            -999.999 /* Invalid angle marker */

/* Array dimension constants */
#define QUAT_ARRAY_SIZE          4        /* Quaternion array size (q0, q1, q2, q3) */

/****************************************************************************
 * Quaternion to Rotation Matrix Conversion
 ****************************************************************************/

/**
 * @brief Converts a quaternion orientation to a rotation matrix
 *
 * RotMtx is actually coordination frame rotation matrix such that
 * w = RotMtx*v is same as w = Quat'*v*Quat. Uses optimized computation
 * with pre-calculated sub-products to avoid redundant multiplications.
 *
 * @param[in] NormQuat - Normalized quaternion representing orientation
 * @param[out] RotMtx - 3x3 rotation matrix output
 * @return None
 */
void Quat2RotMtx(const quaternion_double_t *NormQuat,
	             double                     RotMtx[3][3])
{
	double sub_prod_q0, sub_prod_q0q0, sub_prod_q0q1, sub_prod_q0q2, sub_prod_q0q3;
	double sub_prod_q1, sub_prod_q1q1, sub_prod_q1q2, sub_prod_q1q3;
	double sub_prod_q2, sub_prod_q2q2, sub_prod_q2q3;
	double sub_prod_q3, sub_prod_q3q3;

	sub_prod_q0 = QUAT_SCALE_TWO * NormQuat->q0;
	sub_prod_q0q0 = sub_prod_q0 * NormQuat->q0;
	sub_prod_q0q1 = sub_prod_q0 * NormQuat->q1;
	sub_prod_q0q2 = sub_prod_q0 * NormQuat->q2;
	sub_prod_q0q3 = sub_prod_q0 * NormQuat->q3;

	sub_prod_q1 = QUAT_SCALE_TWO * NormQuat->q1;
	sub_prod_q1q1 = sub_prod_q1 * NormQuat->q1;
	sub_prod_q1q2 = sub_prod_q1 * NormQuat->q2;
	sub_prod_q1q3 = sub_prod_q1 * NormQuat->q3;

	sub_prod_q2 = QUAT_SCALE_TWO * NormQuat->q2;
	sub_prod_q2q2 = sub_prod_q2 * NormQuat->q2;
	sub_prod_q2q3 = sub_prod_q2 * NormQuat->q3;

	sub_prod_q3 = QUAT_SCALE_TWO * NormQuat->q3;
	sub_prod_q3q3 = sub_prod_q3 * NormQuat->q3;

	RotMtx[0][0] = (sub_prod_q0q0 + sub_prod_q1q1) - QUAT_IDENTITY_SCALAR;
	RotMtx[0][1] = (sub_prod_q1q2 + sub_prod_q0q3);
	RotMtx[0][2] = (sub_prod_q1q3 - sub_prod_q0q2);
	RotMtx[1][0] = (sub_prod_q1q2 - sub_prod_q0q3);
	RotMtx[1][1] = (sub_prod_q0q0 + sub_prod_q2q2) - QUAT_IDENTITY_SCALAR;
	RotMtx[1][2] = (sub_prod_q2q3 + sub_prod_q0q1);
	RotMtx[2][0] = (sub_prod_q1q3 + sub_prod_q0q2);
	RotMtx[2][1] = (sub_prod_q2q3 - sub_prod_q0q1);
	RotMtx[2][2] = (sub_prod_q0q0 + sub_prod_q3q3) - QUAT_IDENTITY_SCALAR;

}

/****************************************************************************
 * Rotation Matrix to Quaternion Conversion
 ****************************************************************************/

/**
 * @brief Converts a rotation matrix orientation to a quaternion
 *
 * Uses the Shepperd method with matrix A = [[0.25, 0.25, 0.25, 0.25], ...]
 * to extract quaternion from rotation matrix trace and diagonal elements.
 * Handles sign ambiguities using off-diagonal differences.
 *
 * @param[in] RotMtx - 3x3 rotation matrix input
 * @param[out] Quat - Output quaternion representing the same orientation
 * @return None
 */
void RotMtx2Quat(const double        RotMtx[3][3],
	             quaternion_double_t *Quat)
{

	double   b[4];
	uint32_t k;
	uint32_t indx;

	static const double A[4][4] = {{QUAT_QUARTER,  QUAT_QUARTER,  QUAT_QUARTER,  QUAT_QUARTER},
		                           {QUAT_QUARTER, -QUAT_QUARTER, -QUAT_QUARTER,  QUAT_QUARTER},
		                           {-QUAT_QUARTER,  QUAT_QUARTER, -QUAT_QUARTER,  QUAT_QUARTER},
		                           {-QUAT_QUARTER, -QUAT_QUARTER,  QUAT_QUARTER,  QUAT_QUARTER}};

	double q[4] = { QUAT_IDENTITY_SCALAR, QUAT_IDENTITY_VECTOR, QUAT_IDENTITY_VECTOR, QUAT_IDENTITY_VECTOR};

	b[0] = RotMtx[0][0];
	b[1] = RotMtx[1][1];
	b[2] = RotMtx[2][2];
	b[3] = QUAT_IDENTITY_SCALAR;
	for (k = 0; k < QUAT_ARRAY_SIZE; k++) {
		q[k] = QUAT_IDENTITY_VECTOR;
		for (indx = 0; indx < QUAT_ARRAY_SIZE; indx++) {
			q[k] += A[k][indx] * b[indx];
		}
		if (q[k] < QUAT_IDENTITY_VECTOR) {
			q[k] = QUAT_IDENTITY_VECTOR;
		}
		q[k] = sqrt(q[k]);
	}

	if (RotMtx[1][2] < RotMtx[2][1]) {
		q[1] = -q[1];
	}
	if (RotMtx[2][0] < RotMtx[0][2]) {
		q[2] = -q[2];
	}
	if (RotMtx[0][1] < RotMtx[1][0]) {
		q[3] = -q[3];
	}

	Quat->q0 = q[0];
	Quat->q1 = q[1];
	Quat->q2 = q[2];
	Quat->q3 = q[3];

}

/****************************************************************************
 * Axis-Angle to Quaternion Conversion
 ****************************************************************************/

/**
 * @brief Converts an axis-angle orientation to a rotation quaternion
 *
 * Input: rotation axis with unit magnitude and rotation angle in radians.
 * Computes q = [cos(ang/2), sin(ang/2)*axis] representing coordinate
 * frame rotation (opposite of vector rotation).
 *
 * @param[in] unit_axis - Unit vector representing rotation axis [nx, ny, nz]
 * @param[in] ang_rad - Rotation angle in radians
 * @param[out] QuatRotNorm - Output normalized rotation quaternion
 * @return None
 *
 * @note This is coordinate frame rotation (opposite of vector rotation)
 */
void UnitAxisAngle2Quat(const double        unit_axis[3],
	                    const double        ang_rad,
	                    quaternion_double_t *QuatRotNorm)
{

	QuatRotNorm->q0 = cos(ang_rad / QUAT_SCALE_TWO);
	QuatRotNorm->q1 = unit_axis[0] * sin(ang_rad / QUAT_SCALE_TWO);
	QuatRotNorm->q2 = unit_axis[1] * sin(ang_rad / QUAT_SCALE_TWO);
	QuatRotNorm->q3 = unit_axis[2] * sin(ang_rad / QUAT_SCALE_TWO);

}

/****************************************************************************
 * Quaternion Normalization
 ****************************************************************************/

/**
 * @brief Normalize a quaternion to unit magnitude
 *
 * Normalizes quaternion: qnorm = Quat/|Quat|. If Quat.q0 is negative,
 * it makes it positive without changing the rotation represented. Returns
 * identity quaternion if magnitude is below EPSILON threshold.
 *
 * @param[in] Quat - Input quaternion to be normalized
 * @param[out] NormQuat - Output normalized quaternion with |q| = 1
 * @return None
 *
 * @note Ensures q0 >= 0 by flipping all signs if necessary
 */
void QuatNormal(const quaternion_double_t *Quat,
	            quaternion_double_t       *NormQuat)
{

	double   qmag;
	double   qnorm[4] = { QUAT_IDENTITY_SCALAR, QUAT_IDENTITY_VECTOR, QUAT_IDENTITY_VECTOR, QUAT_IDENTITY_VECTOR };
	double   q[4];
	uint32_t k;

	q[0] = Quat->q0;
	q[1] = Quat->q1;
	q[2] = Quat->q2;
	q[3] = Quat->q3;

	qmag = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
	qnorm[0] = qmag;

	if(qmag > EPSILON) {
		for(k = 0; k < QUAT_ARRAY_SIZE; k++) {
			qnorm[k] = q[k] / qmag;
		}
	}
	if (qnorm[0] < QUAT_IDENTITY_VECTOR) {
		for(k = 0; k < QUAT_ARRAY_SIZE; k++) {
			qnorm[k] = -qnorm[k];
		}
	}

	NormQuat->q0 = qnorm[0];
	NormQuat->q1 = qnorm[1];
	NormQuat->q2 = qnorm[2];
	NormQuat->q3 = qnorm[3];

}

/****************************************************************************
 * Quaternion Multiplication
 ****************************************************************************/

/**
 * @brief Calculates product of two quaternions
 *
 * Computes quaternion multiplication: pq = p * q using optimized
 * computation with 8 sub-products. Formula:
 * pq = [p0q0-P.Q, PxQ + p0Q + q0P] where p = [p0, P] and q = [q0, Q]
 *
 * @param[in] pQuat - First quaternion (left operand)
 * @param[in] qQuat - Second quaternion (right operand)
 * @param[out] pqQuat - Product quaternion pq = p * q
 * @return None
 *
 * @note Uses 8 sub-products method for efficient computation
 */
void QuatProduct(const quaternion_double_t *pQuat,
	             const quaternion_double_t *qQuat,
	             quaternion_double_t       *pqQuat)
{

	double sub_prod1 = (pQuat->q0 + pQuat->q1) * (qQuat->q0 + qQuat->q1);
	double sub_prod2 = (pQuat->q3 - pQuat->q2) * (qQuat->q2 - qQuat->q3);
	double sub_prod3 = (pQuat->q0 - pQuat->q1) * (qQuat->q2 + qQuat->q3);
	double sub_prod4 = (pQuat->q2 + pQuat->q3) * (qQuat->q0 - qQuat->q1);
	double sub_prod5 = (pQuat->q1 + pQuat->q3) * (qQuat->q1 + qQuat->q2);
	double sub_prod6 = (pQuat->q1 - pQuat->q3) * (qQuat->q1 - qQuat->q2);
	double sub_prod7 = (pQuat->q0 + pQuat->q2) * (qQuat->q0 - qQuat->q3);
	double sub_prod8 = (pQuat->q0 - pQuat->q2) * (qQuat->q0 + qQuat->q3);

	pqQuat->q0 = sub_prod2 - QUAT_HALF_ANGLE*(sub_prod5 + sub_prod6 - sub_prod7 - sub_prod8);
	pqQuat->q1 = sub_prod1 - QUAT_HALF_ANGLE*(sub_prod5 + sub_prod6 + sub_prod7 + sub_prod8);
	pqQuat->q2 = sub_prod3 + QUAT_HALF_ANGLE*(sub_prod5 - sub_prod6 + sub_prod7 - sub_prod8);
	pqQuat->q3 = sub_prod4 + QUAT_HALF_ANGLE*(sub_prod5 - sub_prod6 - sub_prod7 + sub_prod8);

}

/****************************************************************************
 * Quaternion Integration Functions
 ****************************************************************************/

/**
 * @brief Time integrate (zeroth order) the quaternion based on angular rate
 *
 * Input angular rate is instantaneous (in degrees/sec) and delta T is time
 * interval between two samples in sec. Uses small angle approximation for
 * small rotations (mag < EPSILON). Otherwise converts to axis-angle form.
 *
 * @param[in] QuatPre - Previous quaternion at time t
 * @param[in] ang_rate_dps - Instantaneous angular rate vector (deg/sec)
 * @param[in] deltaT - Time interval between samples (seconds)
 * @param[out] QuatInt - Integrated quaternion at time t + deltaT
 * @return None
 *
 * @note Uses MacLaurin series for angles < sqrt(EPSILON) rad
 */
void QuatIntegrate(const quaternion_double_t *QuatPre,
	               const double              ang_rate_dps[3],
	               const double              deltaT,
	               quaternion_double_t       *QuatInt)
{

	double     ang_deg[3];
	double     ang_mag_deg;
	double     rot_axis_unit[3];
	uint32_t   i;

	quaternion_double_t QuatAng;

	for(i = 0; i < 3; i++) {
		ang_deg[i] = ang_rate_dps[i] * deltaT;
	}

	ang_mag_deg = sqrt(ang_deg[0]* ang_deg[0] + ang_deg[1] * ang_deg[1] + ang_deg[2] * ang_deg[2]);

	if (ang_mag_deg > EPSILON) {
		for( i = 0; i < 3; i++) {
			rot_axis_unit[i] = ang_deg[i] / ang_mag_deg;
		}
		UnitAxisAngle2Quat(rot_axis_unit, ang_mag_deg*DEG2RAD, &QuatAng);
	}
	else {
		//QuatAng.q0 = sqrt(1.0 - 0.25*ang_mag_deg*ang_mag_deg);
		QuatAng.q0 = QUAT_IDENTITY_SCALAR - QUAT_EIGHTH*ang_mag_deg*ang_mag_deg;
		QuatAng.q1 = QUAT_HALF_ANGLE * ang_deg[0];
		QuatAng.q2 = QUAT_HALF_ANGLE * ang_deg[1];
		QuatAng.q3 = QUAT_HALF_ANGLE * ang_deg[2];
	}
	QuatProduct(QuatPre, &QuatAng, QuatInt);

}

/**
 * @brief Time integrate (first order) the quaternion based on angular rates
 *
 * Input angular rates are current and previous (in degrees/sec) and delta T
 * is time interval between this and previous samples in sec. Uses trapezoidal
 * integration plus coriolis correction term for improved accuracy.
 *
 * @param[in] QuatPre - Previous quaternion at time t
 * @param[in] ang_rate1_dps - Current instantaneous angular rate (deg/sec)
 * @param[in] ang_rate2_dps - Previous instantaneous angular rate (deg/sec)
 * @param[in] deltaT - Time interval between this and previous samples (sec)
 * @param[out] QuatInt - Integrated quaternion at time t + deltaT
 * @return None
 *
 * @note Adds cross-product correction term scaled by 2*dT^2/48
 */
void QuatIntegrate1st(const quaternion_double_t *QuatPre,
	                  const double              ang_rate1_dps[3],
	                  const double              ang_rate2_dps[3],
	                  const double              deltaT,
	                  quaternion_double_t       *QuatInt)
{

	double    avg_angrate_deg[3];
	double    cp_ang_rate[3];
	double    cp_ang_rate_mag;
	double    scale;
	uint32_t  i;

	quaternion_double_t QuatInt1st, QuatInt2nd, QuatAngCP;

	for(i = 0; i < 3; i++) {
		avg_angrate_deg[i] = QUAT_HALF_ANGLE*(ang_rate1_dps[i] + ang_rate2_dps[i]);
	}
	QuatIntegrate(QuatPre, avg_angrate_deg, deltaT, &QuatInt1st);

	cp_ang_rate[0] = (ang_rate1_dps[1] * ang_rate2_dps[2]) - (ang_rate1_dps[2] * ang_rate2_dps[1]);
	cp_ang_rate[1] = (ang_rate1_dps[2] * ang_rate2_dps[0]) - (ang_rate1_dps[0] * ang_rate2_dps[2]);
	cp_ang_rate[2] = (ang_rate1_dps[0] * ang_rate2_dps[1]) - (ang_rate1_dps[1] * ang_rate2_dps[0]);

	cp_ang_rate_mag = sqrt(cp_ang_rate[0] * cp_ang_rate[0] + cp_ang_rate[1] * cp_ang_rate[1] + cp_ang_rate[2] * cp_ang_rate[2]);

	scale = QUAT_SCALE_TWO*deltaT*deltaT*ONEOVER48;

	if (cp_ang_rate_mag > EPSILON) {
		QuatAngCP.q0 = QUAT_IDENTITY_VECTOR;
		QuatAngCP.q1 = scale*cp_ang_rate[0];
		QuatAngCP.q2 = scale*cp_ang_rate[1];
		QuatAngCP.q3 = scale*cp_ang_rate[2];
		QuatProduct(QuatPre, &QuatAngCP, &QuatInt2nd);
	}
	else {
		QuatInt2nd.q0 = QUAT_IDENTITY_VECTOR;
		QuatInt2nd.q1 = QUAT_IDENTITY_VECTOR;
		QuatInt2nd.q2 = QUAT_IDENTITY_VECTOR;
		QuatInt2nd.q3 = QUAT_IDENTITY_VECTOR;
	}

	QuatInt->q0 = QuatInt1st.q0 + QuatInt2nd.q0;
	QuatInt->q1 = QuatInt1st.q1 + QuatInt2nd.q1;
	QuatInt->q2 = QuatInt1st.q2 + QuatInt2nd.q2;
	QuatInt->q3 = QuatInt1st.q3 + QuatInt2nd.q3;

}

/****************************************************************************
 * Rotation Matrix to Euler Angles Conversion
 ****************************************************************************/

/**
 * @brief Convert rotation matrix to Euler angles (pitch, yaw, roll)
 *
 * Extracts Euler angles from rotation matrix with gimbal lock handling.
 * Output ranges: -90 <= phi <= 90, -180 <= theta < 180, 0 <= psi < 360.
 * Uses previous values to resolve gimbal lock at roll = ±90 degrees.
 *
 * @param[in] RotMtx - 3x3 rotation matrix
 * @param[out] theta - Pitch angle (degrees)
 * @param[out] phi - Roll angle (degrees)
 * @param[out] psi - Yaw angle (degrees)
 * @return None
 *
 * @note Alternates between using previous theta and psi for gimbal lock
 */
void RotMtx2Angles(const double RotMtx[3][3],
	               double       *theta,
	               double       *phi,
	               double       *psi)
{

	static uint32_t prev_theta_used = 0;
	double          angle_val;
	uint32_t        angle_vld;

	/*  roll angle [-90,90) */
	*phi = asin(RotMtx[0][2]) * RAD2DEG;

	/*  pitch angle [-180,180)  and yaw angle [0, 360)    */
	*theta = ANGLE_INVALID;
	*psi = ANGLE_INVALID;
	if ((RotMtx[0][2] < (QUAT_IDENTITY_SCALAR - EPSILON)) && (RotMtx[0][2] > -(QUAT_IDENTITY_SCALAR - EPSILON))) {
		angle_val = atan2_safe(-RotMtx[1][2], RotMtx[2][2], &angle_vld);
		if (angle_vld == 1) {
			*theta = angle_val * RAD2DEG;
		}
		angle_val = atan2_safe(-RotMtx[0][1], RotMtx[0][0], &angle_vld);
		if (angle_vld == 1) {
			*psi = angle_val * RAD2DEG;
		}
	}
	/*  and if roll = 90 or -90 , resolve gimbal lock first using prev values */
	else {
		angle_val = atan2_safe(RotMtx[1][0], RotMtx[1][1], &angle_vld);
		if (angle_vld == 1) {
			if (prev_theta_used == 0) {
				if ((RotMtx[0][2] >= (QUAT_IDENTITY_SCALAR - EPSILON))) {
					*psi = (angle_val * RAD2DEG) - *theta;
				}
				else {
					*psi = (angle_val * RAD2DEG) + *theta;
				}
				prev_theta_used = 1;
			}
			else {
				if ((RotMtx[0][2] >= (QUAT_IDENTITY_SCALAR - EPSILON))) {
					*theta = (angle_val * RAD2DEG) - *psi;
				}
				else {
					*theta = (angle_val * RAD2DEG) + *psi;
				}
				prev_theta_used = 0;
			}
		}
	}

	if (*theta > MAX_POS_PITCH_DEG) {
		*theta = ANGLE_NEG_180_DEG;
	}
	if (*psi < ANGLE_ZERO_DEG) {
		*psi += ANGLE_360_DEG;
	}
	else if (*psi >= ANGLE_360_DEG) {
		*psi = ANGLE_ZERO_DEG;
	}

}

/****************************************************************************
 * Safe Arctangent Function
 ****************************************************************************/

/**
 * @brief Safe arctangent function with input validation
 *
 * Computes atan2(y,x) with checks for infinities and near-zero values.
 * Handles special cases to avoid numerical issues:
 * - Both x and y near zero: returns 0, invalid
 * - x near zero, y<0: returns -PI/2
 * - x near zero, y>0: returns +PI/2
 * - y near zero, x<0: returns PI
 * - y near zero, x>0: returns 0
 *
 * @param[in] y - Y coordinate value
 * @param[in] x - X coordinate value
 * @param[out] vld - Validity flag: 1 if valid, 0 if invalid
 * @return Arctangent value in radians, or 0.0 if invalid
 *
 * @note Uses TINY and INFINITY thresholds for comparisons
 */
double atan2_safe(double    y,
                  double    x,
	              uint32_t  *vld)
{
	double   retval = QUAT_IDENTITY_VECTOR;

	*vld = true;

	// Check before atan2 use
	if ((fabs(x) < INFINITY) && (fabs(y) < INFINITY)) {
		if ((fabs(x) < TINY) && (fabs(y) < TINY)) {
			retval = QUAT_IDENTITY_VECTOR;
			*vld = false;
		}
		else if ((fabs(x) < TINY) && (y < QUAT_IDENTITY_VECTOR)) {
			retval = (double)(-PI / QUAT_SCALE_TWO);
		}
		else if ((fabs(x) < TINY) && (y > QUAT_IDENTITY_VECTOR)) {
			retval = (double)(PI / QUAT_SCALE_TWO);
		}
		else if ((fabs(y) < TINY) && (x < 0.0f)) {
			retval = (double)PI;
		}
		else if ((fabs(y) < TINY) && (x > 0.0f)) {
			retval = QUAT_IDENTITY_VECTOR;
		}
		else {
			retval = atan2(y, x);
		}
	}
	else {
		retval = QUAT_IDENTITY_VECTOR;
		*vld = false;
	}

	return(retval);

}   /* atan2_safe() */

