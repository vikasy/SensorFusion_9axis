#ifndef _ALGO_SF_QUATMATH_H_
#define _ALGO_SF_QUATMATH_H_

/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include "algo_sf_types.h"

// quaternion double precision structure definition
typedef struct quaternion_double
{
	double q0;	// scalar component
	double q1;	// x vector component
	double q2;	// y vector component
	double q3;	// z vector component
} quaternion_double_t;


/* Quat2RotMtx Converts a quaternion orientation to a rotation matrix
*    RotMtx is actually cordination frame rotation matrix.
*    such that w = RotMtx*v is same as w = Quat'*v*Quat
*    Converts a quaternion orientation to a rotation matrix.
*/
void Quat2RotMtx(const quaternion_double_t *Quat, double RotMtx[3][3]);

/* RotMtx2Quat Converts a rotation matrix orientation to a quaternion
*
*    Converts a rotation matrix orientation to a quaternion.
*/
void RotMtx2Quat(const double        RotMtx[3][3],
                 quaternion_double_t *Quat);

/* QuatNormalize Normalize a quaternion so that its mag = 1
*    qnorm = QuatNormal(Quat)
*    qnorm = Quat/qmag
*
*    If Quat(0) is negative, it makes it positive without changing the
*    functionality of quaternion
*/
void QuatNormal(const quaternion_double_t *Quat, quaternion_double_t *NormQuat);

/* QuatProduct Calculates product of two quaternions
*    pq = QuatProduct(p, q)
*    pq = [p0q0-P.Q, PxQ + p0Q + q0P]
*    where q = [p0, P] and q = [q0, Q]
*/
void QuatProduct(const quaternion_double_t *pQuat, const quaternion_double_t *qQuat, quaternion_double_t *pqQuat);

/*  Rotate given quaternion by provided angle */
/*  Angle is in degree */
/*  */
void QuatIntegrate(const quaternion_double_t *QuatPre, const double ang_rate_dps[3], const double deltaT, quaternion_double_t *QuatInt);

void QuatIntegrate1st(const quaternion_double_t *QuatPre,
	const double              ang_rate1_dps[3],
	const double              ang_rate2_dps[3],
	const double              deltaT,
	quaternion_double_t       *QuatInt);



void RotMtx2Angles(const double RotMtx[3][3],
	double       *theta,
	double       *phi,
	double       *psi);

/* Atan2 with check on inputs   */
double atan2_safe(double    y,
	double    x,
	uint32_t  *vld);

#endif /* _ALGO_SF_QUATMATH_H_ */
