#ifndef _ALGO_SF_MATRIX_H_
#define _ALGO_SF_MATRIX_H_

#include "algo_sf_types.h"

// function prototypes
void f3x3matrixAeqI(float A[][3]);
void fmatrixAeqI(float *A[], uint32_t rc);
void f3x3matrixAeqScalar(float A[][3], float Scalar);
void fmatrixAeqInvA(float *A[], uint32_t iColInd[], uint32_t iRowInd[], uint32_t iPivot[], uint32_t isize, uint32_t *pierror);

#endif   /* _ALGO_SF_MATRIX_H_ */