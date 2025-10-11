
/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include "algo_sf_quatmath.h"

/* Quat2RotMtx Converts a quaternion orientation to a rotation matrix 
*    RotMtx is actually cordination frame rotation matrix. 
*    such that w = RotMtx*v is same as w = Quat'*v*Quat 
*    Converts a quaternion orientation to a rotation matrix. 
*/
void Quat2RotMtx(const quaternion_double_t *NormQuat,
	             double                     RotMtx[3][3])
{
	double sub_prod_q0, sub_prod_q0q0, sub_prod_q0q1, sub_prod_q0q2, sub_prod_q0q3;
	double sub_prod_q1, sub_prod_q1q1, sub_prod_q1q2, sub_prod_q1q3;
	double sub_prod_q2, sub_prod_q2q2, sub_prod_q2q3;
	double sub_prod_q3, sub_prod_q3q3;
	
	sub_prod_q0 = 2.0 * NormQuat->q0;
	sub_prod_q0q0 = sub_prod_q0 * NormQuat->q0;
	sub_prod_q0q1 = sub_prod_q0 * NormQuat->q1;
	sub_prod_q0q2 = sub_prod_q0 * NormQuat->q2;
	sub_prod_q0q3 = sub_prod_q0 * NormQuat->q3;
	
	sub_prod_q1 = 2.0 * NormQuat->q1;
	sub_prod_q1q1 = sub_prod_q1 * NormQuat->q1;
	sub_prod_q1q2 = sub_prod_q1 * NormQuat->q2;
	sub_prod_q1q3 = sub_prod_q1 * NormQuat->q3;
	
	sub_prod_q2 = 2.0 * NormQuat->q2;
	sub_prod_q2q2 = sub_prod_q2 * NormQuat->q2;
	sub_prod_q2q3 = sub_prod_q2 * NormQuat->q3;
	
	sub_prod_q3 = 2.0 * NormQuat->q3;
	sub_prod_q3q3 = sub_prod_q3 * NormQuat->q3;
	
	RotMtx[0][0] = (sub_prod_q0q0 + sub_prod_q1q1) - 1.0;
	RotMtx[0][1] = (sub_prod_q1q2 + sub_prod_q0q3);
	RotMtx[0][2] = (sub_prod_q1q3 - sub_prod_q0q2);
	RotMtx[1][0] = (sub_prod_q1q2 - sub_prod_q0q3);
	RotMtx[1][1] = (sub_prod_q0q0 + sub_prod_q2q2) - 1.0;
	RotMtx[1][2] = (sub_prod_q2q3 + sub_prod_q0q1);
	RotMtx[2][0] = (sub_prod_q1q3 + sub_prod_q0q2);
	RotMtx[2][1] = (sub_prod_q2q3 - sub_prod_q0q1);
	RotMtx[2][2] = (sub_prod_q0q0 + sub_prod_q3q3) - 1.0;

}


/* RotMtx2Quat Converts a rotation matrix orientation to a quaternion 
* 
*    Converts a rotation matrix orientation to a quaternion. 
*/
void RotMtx2Quat(const double        RotMtx[3][3], 
	             quaternion_double_t *Quat)
{

	double   b[4];
	uint32_t k;
	uint32_t indx;

	static const double A[4][4] = {{0.25,  0.25,  0.25,  0.25}, 
		                           {0.25, -0.25, -0.25,  0.25}, 
		                           {-0.25,  0.25, -0.25,  0.25}, 
		                           {-0.25, -0.25,  0.25,  0.25}};

	double q[4] = { 1.0, 0.0, 0.0, 0.0};

	b[0] = RotMtx[0][0];
	b[1] = RotMtx[1][1];
	b[2] = RotMtx[2][2];
	b[3] = 1.0;
	for (k = 0; k < 4; k++) {
		q[k] = 0.0;
		for (indx = 0; indx < 4; indx++) {
			q[k] += A[k][indx] * b[indx];
		}
		if (q[k] < 0.0) {
			q[k] = 0.0;
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


/* UnitAxisAngle2Quat Converts an axis-angle orientation to a rotation quaternion
*    Input: rotation axis with unit magnitude and rotation angle in radians
*    Converts an axis angle orientation pair to a quaternion rotation
*    NOTE: This is cordinate frame rotation (opposite of vector rotation)
*/
void UnitAxisAngle2Quat(const double        unit_axis[3],
	                    const double        ang_rad,
	                    quaternion_double_t *QuatRotNorm)
{

	QuatRotNorm->q0 = cos(ang_rad / 2.0);
	QuatRotNorm->q1 = unit_axis[0] * sin(ang_rad / 2.0);
	QuatRotNorm->q2 = unit_axis[1] * sin(ang_rad / 2.0);
	QuatRotNorm->q3 = unit_axis[2] * sin(ang_rad / 2.0);

}


/* QuatNormalize Normalize a quaternion so that its mag = 1 
*    qnorm = QuatNormal(Quat) 
*    qnorm = Quat/qmag 
*  
*    If Quat(0) is negative, it makes it positive without changing the 
*    functionality of quaternion 
*/
void QuatNormal(const quaternion_double_t *Quat, 
	            quaternion_double_t       *NormQuat)
{

	double   qmag;
	double   qnorm[4] = { 1.0, 0.0, 0.0, 0.0 };
	double   q[4];
	uint32_t k;

	q[0] = Quat->q0;
	q[1] = Quat->q1;
	q[2] = Quat->q2;
	q[3] = Quat->q3;

	qmag = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
	qnorm[0] = qmag;

	if(qmag > EPSILON) {
		for(k = 0; k < 4; k++) {
			qnorm[k] = q[k] / qmag;
		}
	}
	if (qnorm[0] < 0.0) {
		for(k = 0; k < 4; k++) {
			qnorm[k] = -qnorm[k];
		}
	}

	NormQuat->q0 = qnorm[0];
	NormQuat->q1 = qnorm[1];
	NormQuat->q2 = qnorm[2];
	NormQuat->q3 = qnorm[3];

}


/* QuatProduct Calculates product of two quaternions 
*    pq = QuatProduct(p, q) 
*    pq = [p0q0-P.Q, PxQ + p0Q + q0P]  
*    where q = [p0, P] and q = [q0, Q] 
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

	pqQuat->q0 = sub_prod2 - 0.5*(sub_prod5 + sub_prod6 - sub_prod7 - sub_prod8);
	pqQuat->q1 = sub_prod1 - 0.5*(sub_prod5 + sub_prod6 + sub_prod7 + sub_prod8);
	pqQuat->q2 = sub_prod3 + 0.5*(sub_prod5 - sub_prod6 + sub_prod7 - sub_prod8);
	pqQuat->q3 = sub_prod4 + 0.5*(sub_prod5 - sub_prod6 - sub_prod7 + sub_prod8);

}


/* Time integrate (zeroth order) the given quaternion based on the provided angle change over one time interval
*  Input Anglur rate is instantaneous (in degrees/sec) 
*  and dleta T is time interval between two samples in sec. 
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
		QuatAng.q0 = 1.0 - 0.125*ang_mag_deg*ang_mag_deg;
		QuatAng.q1 = 0.5 * ang_deg[0];
		QuatAng.q2 = 0.5 * ang_deg[1];
		QuatAng.q3 = 0.5 * ang_deg[2];
	}
	QuatProduct(QuatPre, &QuatAng, QuatInt);

}


/* Time integrate (first order) the given quaternion based on the provided angle change over one time interval
*  Input Anglur rates are instantaneous and previous (in degrees/sec)
*  and dleta T is time interval between this and previous samples in sec.
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
		avg_angrate_deg[i] = 0.5*(ang_rate1_dps[i] + ang_rate2_dps[i]);
	}
	QuatIntegrate(QuatPre, avg_angrate_deg, deltaT, &QuatInt1st);

	cp_ang_rate[0] = (ang_rate1_dps[1] * ang_rate2_dps[2]) - (ang_rate1_dps[2] * ang_rate2_dps[1]);
	cp_ang_rate[1] = (ang_rate1_dps[2] * ang_rate2_dps[0]) - (ang_rate1_dps[0] * ang_rate2_dps[2]);
	cp_ang_rate[2] = (ang_rate1_dps[0] * ang_rate2_dps[1]) - (ang_rate1_dps[1] * ang_rate2_dps[0]);

	cp_ang_rate_mag = sqrt(cp_ang_rate[0] * cp_ang_rate[0] + cp_ang_rate[1] * cp_ang_rate[1] + cp_ang_rate[2] * cp_ang_rate[2]);

	scale = 2*deltaT*deltaT*ONEOVER48;

	if (cp_ang_rate_mag > EPSILON) {
		QuatAngCP.q0 = 0.0;
		QuatAngCP.q1 = scale*cp_ang_rate[0];
		QuatAngCP.q2 = scale*cp_ang_rate[1];
		QuatAngCP.q3 = scale*cp_ang_rate[2];
		QuatProduct(QuatPre, &QuatAngCP, &QuatInt2nd);
	}
	else {
		QuatInt2nd.q0 = 0.0;
		QuatInt2nd.q1 = 0.0;
		QuatInt2nd.q2 = 0.0;
		QuatInt2nd.q3 = 0.0;
	}

	QuatInt->q0 = QuatInt1st.q0 + QuatInt2nd.q0;
	QuatInt->q1 = QuatInt1st.q1 + QuatInt2nd.q1;
	QuatInt->q2 = QuatInt1st.q2 + QuatInt2nd.q2;
	QuatInt->q3 = QuatInt1st.q3 + QuatInt2nd.q3;

}


/*  Convert rotation matrix to Euler angles pitch yaw and roll
*   Input: Rotation Matrix
*   Output: -90 <= phi <= 90
*          -180 <= theta < 180
*             0 <= psi < 360
*/
void RotMtx2Angles(const double RotMtx[3][3],
	               double       *theta,
	               double       *phi,
	               double       *psi)
{
#define MAX_POS_PITCH_DEG (179.9999)

	static uint32_t prev_theta_used = 0;
	double          angle_val;
	uint32_t        angle_vld;

	/*  roll angle [-90,90) */
	*phi = asin(RotMtx[0][2]) * RAD2DEG;

	/*  pitch angle [-180,180)  and yaw angle [0, 360)    */
	*theta = -999.999;
	*psi = -999.999;
	if ((RotMtx[0][2] < (1 - EPSILON)) && (RotMtx[0][2] > -(1 - EPSILON))) {
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
				if ((RotMtx[0][2] >= (1 - EPSILON))) {
					*psi = (angle_val * RAD2DEG) - *theta;
				}
				else {
					*psi = (angle_val * RAD2DEG) + *theta;
				}
				prev_theta_used = 1;
			}
			else {
				if ((RotMtx[0][2] >= (1 - EPSILON))) {
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
		*theta = -180.0;
	}
	if (*psi < 0.0) {
		*psi += 360.0;
	}
	else if (*psi >= 360.0) {
		*psi = 0.0;
	}

}


/* Atan2 with check on inputs   */
double atan2_safe(double    y,
                  double    x,
	              uint32_t  *vld)
{
	double   retval = 0.0;

	*vld = true;

	// Check before atan2 use
	if ((fabs(x) < INFINITY) && (fabs(y) < INFINITY)) {
		if ((fabs(x) < TINY) && (fabs(y) < TINY)) {
			retval = 0.0;
			*vld = false;
		}
		else if ((fabs(x) < TINY) && (y < 0.0)) {
			retval = (double)(-PI / 2.0);
		}
		else if ((fabs(x) < TINY) && (y > 0.0)) {
			retval = (double)(PI / 2.0);
		}
		else if ((fabs(y) < TINY) && (x < 0.0f)) {
			retval = (double)PI;
		}
		else if ((fabs(y) < TINY) && (x > 0.0f)) {
			retval = 0.0;
		}
		else {
			retval = atan2(y, x);
		}
	}
	else {
		retval = 0.0;
		*vld = false;
	}

	return(retval);

}   /* atan2_safe() */


