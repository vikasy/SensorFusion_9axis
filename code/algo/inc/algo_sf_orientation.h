#ifndef _ORIENTATION_H_
#define _ORIENTATION_H_

#include "algo_sf_types.h"

// function prototypes
void f3DOFTiltNED(float fR[][3], float fGp[]);
void f3DOFTiltAndroid(float fR[][3], float fGp[]);
void fAndroidAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg, float *pfTheDeg, float *pfPsiDeg,
	float *pfRhoDeg, float *pfChiDeg);
void fQuaternionFromRotationMatrix(float R[][3], quaternion_t *pq);
void fRotationMatrixFromQuaternion(float R[][3], const quaternion_t *pq);
void fRotationVectorDegFromRotationMatrix(float R[][3], float rvecdeg[]);
void fQuaternionFromRotationVectorDeg(quaternion_t *pq, const float rvecdeg[], float fscaling);
void fRotationVectorDegFromQuaternion(quaternion_t *pq, float rvecdeg[]);
void qAeqBxC(quaternion_t *pqA, const quaternion_t *pqB, const quaternion_t *pqC);
void qAeqAxB(quaternion_t *pqA, const quaternion_t *pqB);
quaternion_t qconjgAxB(const quaternion_t *pqA, const quaternion_t *pqB);
void fqAeqNormqA(quaternion_t *pqA);
void fqAeq1(quaternion_t *pqA);

#endif /* _ORIENTATION_H_ */
