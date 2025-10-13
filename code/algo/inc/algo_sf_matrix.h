/*******************************************************************************
 * @file    algo_sf_matrix.h
 * @brief   Matrix operation functions for sensor fusion algorithms
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#ifndef _ALGO_SF_MATRIX_H_
#define _ALGO_SF_MATRIX_H_

#include "algo_sf_types.h"

/****************************************************************************
 * Matrix Constants
 ****************************************************************************/

#define MATRIX_3X3_SIZE            3
#define MATRIX_CORRUPT_THRESHOLD   0.001F

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

/**
 * @brief Set 3x3 matrix to identity matrix
 *
 * @param[in,out] A - 3x3 matrix to be set to identity
 * @return None
 */
void f3x3matrixAeqI(float A[][3]);

/**
 * @brief Set square matrix to identity matrix
 *
 * @param[in,out] A - Pointer array to matrix rows
 * @param[in] rc - Number of rows and columns
 * @return None
 */
void fmatrixAeqI(float *A[], uint32_t rc);

/**
 * @brief Set all elements of 3x3 matrix to a scalar value
 *
 * @param[in,out] A - 3x3 matrix to be set
 * @param[in] Scalar - Value to set all matrix elements to
 * @return None
 */
void f3x3matrixAeqScalar(float A[][3], float Scalar);

/**
 * @brief Compute matrix inverse using Gauss-Jordan elimination
 *
 * @param[in,out] A - Matrix to invert (replaced with inverse on output)
 * @param[in,out] iColInd - Column index array for pivoting
 * @param[in,out] iRowInd - Row index array for pivoting
 * @param[in,out] iPivot - Pivot tracking array
 * @param[in] isize - Matrix size (number of rows/columns)
 * @param[out] pierror - Error flag (0=success, 1=singular matrix)
 * @return None
 *
 * @note Matrix A is modified in place
 * @note Working arrays must be pre-allocated with size >= isize
 */
void fmatrixAeqInvA(float *A[], uint32_t iColInd[], uint32_t iRowInd[], uint32_t iPivot[], uint32_t isize, uint32_t *pierror);

#endif   /* _ALGO_SF_MATRIX_H_ */