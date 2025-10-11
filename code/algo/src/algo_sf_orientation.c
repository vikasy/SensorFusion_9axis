
#include "algo_sf_orientation.h"
#include "algo_sf_matrix.h"
#include "algo_sf_approx.h"

// compile time constants that are private to this file
#define SMALLQ0 0.01F		// limit of quaternion scalar component requiring special algorithm
#define CORRUPTQUAT 0.001F	// threshold for deciding rotation quaternion is corrupt
#define SMALLMODULUS 0.01F	// limit where rounding errors may appear

// Aerospace NED accelerometer 3DOF tilt function computing rotation matrix fR
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
	if (fmodGxyz == 0.0F)
	{
		f3x3matrixAeqI(fR);
		return;
	}

	// check for vertical up or down gimbal lock case
	if (fmodGyz == 0.0F)
	{
		f3x3matrixAeqScalar(fR, 0.0F);
		fR[CHY][CHY] = 1.0F;
		if (fGp[CHX] >= 0.0F)
		{
			fR[CHX][CHZ] = 1.0F;
			fR[CHZ][CHX] = -1.0F;
		}
		else
		{
			fR[CHX][CHZ] = -1.0F;
			fR[CHZ][CHX] = 1.0F;
		}
		return;
	}

	// compute moduli for the general case
	fmodGyz = sqrtf(fmodGyz);
	fmodGxyz = sqrtf(fmodGxyz);
	frecipmodGxyz = 1.0F / fmodGxyz;
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
	fR[CHX][CHY] = 0.0F;
	fR[CHY][CHY] = fR[CHZ][CHZ] * ftmp;
	fR[CHZ][CHY] = -fR[CHY][CHZ] * ftmp;

	return;
}

// Android accelerometer 3DOF tilt function computing rotation matrix fR
void f3DOFTiltAndroid(float fR[][3], float fGp[])
{
	// the Android tilt matrix is mathematically identical to the NED tilt matrix
	// the Android self-consistency twist occurs at 90 deg roll
	f3DOFTiltNED(fR, fGp);
	return;
}


// extract the Android angles in degrees from the Android rotation matrix
void fAndroidAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg, float *pfTheDeg, float *pfPsiDeg,
	float *pfRhoDeg, float *pfChiDeg)
{
	// calculate the roll angle -90.0 <= Phi <= 90.0 deg
	*pfPhiDeg = fasin_deg(R[CHX][CHZ]);

	// calculate the pitch angle -180.0 <= The < 180.0 deg
	*pfTheDeg = fatan2_deg(-R[CHY][CHZ], R[CHZ][CHZ]);

	// map +180 pitch onto the functionally equivalent -180 deg pitch
	if (*pfTheDeg == 180.0F)
	{
		*pfTheDeg = -180.0F;
	}

	// calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
	if (*pfPhiDeg == 90.0F)
	{
		// vertical downwards gimbal lock case
		*pfPsiDeg = fatan2_deg(R[CHY][CHX], R[CHY][CHY]) - *pfTheDeg;
	}
	else if (*pfPhiDeg == -90.0F)
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
	if (*pfPsiDeg < 0.0F)
	{
		*pfPsiDeg += 360.0F;
	}

	// check for rounding errors mapping small negative angle to 360 deg
	if (*pfPsiDeg >= 360.0F)
	{
		*pfPsiDeg = 0.0F;
	}

	// the compass heading angle Rho equals the yaw angle Psi
	// this definition is compliant with Motorola Xoom tablet behavior
	*pfRhoDeg = *pfPsiDeg;

	// calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg) 
	*pfChiDeg = facos_deg(R[CHZ][CHZ]);

	return;
}


// computes normalized rotation quaternion from a rotation vector (deg)
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
	if (fetarad2 <= 0.02F)
	{
		// use MacLaurin series up to and including third order
		sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2);
	}
	else if (fetarad2 <= 0.06F)
	{
		// use MacLaurin series up to and including fifth order
		// angles under sqrt(0.06)=0.245 rad is 14.0 deg and 2807 deg/s (=1623deg/s in 3 axes) at 200Hz and 703 deg/s at 50Hz
		fetarad4 = fetarad2 * fetarad2;
		sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2 + ONEOVER3840 * fetarad4);
	}
	else
	{
		// use exact calculation
		sinhalfeta = (float)sinf(0.5F * fetarad);
	}

	// compute the vector quaternion components q1, q2, q3
	if (fetadeg != 0.0F)
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
		pq->q1 = pq->q2 = pq->q3 = 0.0F;
	}

	// compute the scalar quaternion component q0 by explicit normalization
	// taking care to avoid rounding errors giving negative operand to sqrt
	fvecsq = pq->q1 * pq->q1 + pq->q2 * pq->q2 + pq->q3 * pq->q3;
	if (fvecsq <= 1.0F)
	{
		// normal case
		pq->q0 = sqrtf(1.0F - fvecsq);
	}
	else
	{
		// rounding errors are present
		pq->q0 = 0.0F;
	}

	return;
}

// compute the orientation quaternion from a 3x3 rotation matrix
void fQuaternionFromRotationMatrix(float R[][3], quaternion_t *pq)
{
	float fq0sq;			// q0^2
	float recip4q0;			// 1/4q0

							// the quaternion is not explicitly normalized in this function on the assumption that it
							// is supplied with a normalized rotation matrix. if the rotation matrix is normalized then
							// the quaternion will also be normalized even if the case of small q0

							// get q0^2 and q0
	fq0sq = 0.25F * (1.0F + R[CHX][CHX] + R[CHY][CHY] + R[CHZ][CHZ]);
	pq->q0 = sqrtf(fabs(fq0sq));

	// normal case when q0 is not small meaning rotation angle not near 180 deg
	if (pq->q0 > SMALLQ0)
	{
		// calculate q1 to q3
		recip4q0 = 0.25F / pq->q0;
		pq->q1 = recip4q0 * (R[CHY][CHZ] - R[CHZ][CHY]);
		pq->q2 = recip4q0 * (R[CHZ][CHX] - R[CHX][CHZ]);
		pq->q3 = recip4q0 * (R[CHX][CHY] - R[CHY][CHX]);
	} // end of general case
	else
	{
		// special case of near 180 deg corresponds to nearly symmetric matrix
		// which is not numerically well conditioned for division by small q0
		// instead get absolute values of q1 to q3 from leading diagonal
		pq->q1 = sqrtf(fabs(0.5F * (1.0F + R[CHX][CHX]) - fq0sq));
		pq->q2 = sqrtf(fabs(0.5F * (1.0F + R[CHY][CHY]) - fq0sq));
		pq->q3 = sqrtf(fabs(0.5F * (1.0F + R[CHZ][CHZ]) - fq0sq));

		// correct the signs of q1 to q3 by examining the signs of differenced off-diagonal terms
		if ((R[CHY][CHZ] - R[CHZ][CHY]) < 0.0F) pq->q1 = -pq->q1;
		if ((R[CHZ][CHX] - R[CHX][CHZ]) < 0.0F) pq->q2 = -pq->q2;
		if ((R[CHX][CHY] - R[CHY][CHX]) < 0.0F) pq->q3 = -pq->q3;
	} // end of special case

	return;
}

// compute the rotation matrix from an orientation quaternion
void fRotationMatrixFromQuaternion(float R[][3], const quaternion_t *pq)
{
	float f2q;
	float f2q0q0, f2q0q1, f2q0q2, f2q0q3;
	float f2q1q1, f2q1q2, f2q1q3;
	float f2q2q2, f2q2q3;
	float f2q3q3;

	// calculate products
	f2q = 2.0F * pq->q0;
	f2q0q0 = f2q * pq->q0;
	f2q0q1 = f2q * pq->q1;
	f2q0q2 = f2q * pq->q2;
	f2q0q3 = f2q * pq->q3;
	f2q = 2.0F * pq->q1;
	f2q1q1 = f2q * pq->q1;
	f2q1q2 = f2q * pq->q2;
	f2q1q3 = f2q * pq->q3;
	f2q = 2.0F * pq->q2;
	f2q2q2 = f2q * pq->q2;
	f2q2q3 = f2q * pq->q3;
	f2q3q3 = 2.0F * pq->q3 * pq->q3;

	// calculate the rotation matrix assuming the quaternion is normalized
	R[CHX][CHX] = f2q0q0 + f2q1q1 - 1.0F;
	R[CHX][CHY] = f2q1q2 + f2q0q3;
	R[CHX][CHZ] = f2q1q3 - f2q0q2;
	R[CHY][CHX] = f2q1q2 - f2q0q3;
	R[CHY][CHY] = f2q0q0 + f2q2q2 - 1.0F;
	R[CHY][CHZ] = f2q2q3 + f2q0q1;
	R[CHZ][CHX] = f2q1q3 + f2q0q2;
	R[CHZ][CHY] = f2q2q3 - f2q0q1;
	R[CHZ][CHZ] = f2q0q0 + f2q3q3 - 1.0F;

	return;
}

// function calculate the rotation vector from a rotation matrix
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
	if (ftrace >= 3.0F)
	{
		fetadeg = 0.0F;
	}
	else if (ftrace <= -1.0F)
	{
		fetadeg = 180.0F;
	}
	else
	{
		fetadeg = acosf(0.5F * (ftrace - 1.0F)) * RAD2DEG;
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
	else if (ftrace >= 0.0F)
	{
		// near 0 deg rotation (trace = 3): matrix is nearly identity matrix
		// R[CHY][CHZ]-R[CHZ][CHY]=2*nx*eta(rad) and similarly for other components
		ftmp = 0.5F * RAD2DEG;
		rvecdeg[CHX] *= ftmp;
		rvecdeg[CHY] *= ftmp;
		rvecdeg[CHZ] *= ftmp;
	} // end of zero deg case
	else
	{
		// near 180 deg (trace = -1): matrix is nearly symmetric
		// calculate the absolute value of the components of the axis-angle vector
		rvecdeg[CHX] = 180.0F * sqrtf(fabs(0.5F * (R[CHX][CHX] + 1.0F)));
		rvecdeg[CHY] = 180.0F * sqrtf(fabs(0.5F * (R[CHY][CHY] + 1.0F)));
		rvecdeg[CHZ] = 180.0F * sqrtf(fabs(0.5F * (R[CHZ][CHZ] + 1.0F)));

		// correct the signs of the three components by examining the signs of differenced off-diagonal terms
		if ((R[CHY][CHZ] - R[CHZ][CHY]) < 0.0F) rvecdeg[CHX] = -rvecdeg[CHX];
		if ((R[CHZ][CHX] - R[CHX][CHZ]) < 0.0F) rvecdeg[CHY] = -rvecdeg[CHY];
		if ((R[CHX][CHY] - R[CHY][CHX]) < 0.0F) rvecdeg[CHZ] = -rvecdeg[CHZ];

	} // end of 180 deg case

	return;
}

// computes rotation vector (deg) from rotation quaternion
void fRotationVectorDegFromQuaternion(quaternion_t *pq, float rvecdeg[])
{
	float fetarad;			// rotation angle (rad)
	float fetadeg;			// rotation angle (deg)
	float sinhalfeta;		// sin(eta/2)
	float ftmp;				// scratch variable

							// calculate the rotation angle in the range 0 <= eta < 360 deg
	if ((pq->q0 >= 1.0F) || (pq->q0 <= -1.0F))
	{
		// rotation angle is 0 deg or 2*180 deg = 360 deg = 0 deg
		fetarad = 0.0F;
		fetadeg = 0.0F;
	}
	else
	{
		// general case returning 0 < eta < 360 deg 
		fetarad = 2.0F * acosf(pq->q0);
		fetadeg = fetarad * RAD2DEG;
	}

	// map the rotation angle onto the range -180 deg <= eta < 180 deg 
	if (fetadeg >= 180.0F)
	{
		fetadeg -= 360.0F;
		fetarad = fetadeg * DEG2RAD;
	}

	// calculate sin(eta/2) which will be in the range -1 to +1
	sinhalfeta = (float)sinf(0.5F * fetarad);

	// calculate the rotation vector (deg)
	if (sinhalfeta == 0.0F)
	{
		// the rotation angle eta is zero and the axis is irrelevant 
		rvecdeg[CHX] = rvecdeg[CHY] = rvecdeg[CHZ] = 0.0F;
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


// function compute the quaternion product qA * qB
void qAeqBxC(quaternion_t *pqA, const quaternion_t *pqB, const quaternion_t *pqC)
{
	pqA->q0 = pqB->q0 * pqC->q0 - pqB->q1 * pqC->q1 - pqB->q2 * pqC->q2 - pqB->q3 * pqC->q3;
	pqA->q1 = pqB->q0 * pqC->q1 + pqB->q1 * pqC->q0 + pqB->q2 * pqC->q3 - pqB->q3 * pqC->q2;
	pqA->q2 = pqB->q0 * pqC->q2 - pqB->q1 * pqC->q3 + pqB->q2 * pqC->q0 + pqB->q3 * pqC->q1;
	pqA->q3 = pqB->q0 * pqC->q3 + pqB->q1 * pqC->q2 - pqB->q2 * pqC->q1 + pqB->q3 * pqC->q0;

	return;
}

// function compute the quaternion product qA = qA * qB
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

// function compute the quaternion product conjg(qA) * qB
quaternion_t qconjgAxB(const quaternion_t *pqA, const quaternion_t *pqB)
{
	quaternion_t qProd;

	qProd.q0 = pqA->q0 * pqB->q0 + pqA->q1 * pqB->q1 + pqA->q2 * pqB->q2 + pqA->q3 * pqB->q3;
	qProd.q1 = pqA->q0 * pqB->q1 - pqA->q1 * pqB->q0 - pqA->q2 * pqB->q3 + pqA->q3 * pqB->q2;
	qProd.q2 = pqA->q0 * pqB->q2 + pqA->q1 * pqB->q3 - pqA->q2 * pqB->q0 - pqA->q3 * pqB->q1;
	qProd.q3 = pqA->q0 * pqB->q3 - pqA->q1 * pqB->q2 + pqA->q2 * pqB->q1 - pqA->q3 * pqB->q0;

	return qProd;
}

// function normalizes a rotation quaternion and ensures q0 is non-negative
void fqAeqNormqA(quaternion_t *pqA)
{
	float fNorm;					// quaternion Norm

									// calculate the quaternion Norm
	fNorm = sqrtf(pqA->q0 * pqA->q0 + pqA->q1 * pqA->q1 + pqA->q2 * pqA->q2 + pqA->q3 * pqA->q3);
	if (fNorm > CORRUPTQUAT)
	{
		// general case
		fNorm = 1.0F / fNorm;
		pqA->q0 *= fNorm;
		pqA->q1 *= fNorm;
		pqA->q2 *= fNorm;
		pqA->q3 *= fNorm;
	}
	else
	{
		// return with identity quaternion since the quaternion is corrupted
		pqA->q0 = 1.0F;
		pqA->q1 = pqA->q2 = pqA->q3 = 0.0F;
	}

	// correct a negative scalar component if the function was called with negative q0
	if (pqA->q0 < 0.0F)
	{
		pqA->q0 = -pqA->q0;
		pqA->q1 = -pqA->q1;
		pqA->q2 = -pqA->q2;
		pqA->q3 = -pqA->q3;
	}

	return;
}

// set a quaternion to the unit quaternion
void fqAeq1(quaternion_t *pqA)
{
	pqA->q0 = 1.0F;
	pqA->q1 = pqA->q2 = pqA->q3 = 0.0F;

	return;
}
