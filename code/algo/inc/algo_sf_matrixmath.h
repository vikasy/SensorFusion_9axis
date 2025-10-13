/*******************************************************************************
 * @file    algo_sf_matrixmath.h
 * @brief   Advanced matrix mathematics for sensor fusion (double precision)
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#ifndef _ALGO_SF_MATRIXMATH_H_
#define _ALGO_SF_MATRIXMATH_H_

#include "algo_sf_types.h"

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/**
 * @brief 3x3 matrix block type (double precision)
 */
typedef struct blk_mtx_3x3 {
	double elem[3][3];
} blk_mtx_3x3_t;

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

/**
 * @brief Compute inverse of 3x3 symmetric matrix
 *
 * Uses only upper triangular part of input matrix.
 *
 * @param[in] Amat - Input symmetric matrix [3x3]
 * @param[out] InvAmat - Inverse matrix [3x3]
 * @return 1 if successful, 0 if no inverse exists (singular)
 */
uint32_t SymMatInv3x3(const double Amat[3][3], double InvAmat[3][3]);

/**
 * @brief Compute product of two 3x3 matrices: ABmat = Amat * Bmat
 *
 * @param[in] Amat - First matrix [3x3]
 * @param[in] Bmat - Second matrix [3x3]
 * @param[out] ABmat - Product matrix [3x3]
 * @return None
 */
void MatProd3x3(const double Amat[3][3], const double Bmat[3][3], double ABmat[3][3]);

/**
 * @brief Create cross product matrix from vector: Amat = CPMat(Avec)
 *
 * Creates skew-symmetric matrix for cross product operation.
 *
 * @param[in] Avec - Input vector [3]
 * @param[out] Amat - Cross product matrix [3x3]
 * @return None
 */
void CrossPdctMtx_3x3(const double Avec[3], blk_mtx_3x3_t *Amat);

/**
 * @brief Scale block matrix by scalar: kAmat = k * Amat
 *
 * @param[in] k - Scalar multiplier
 * @param[in] Amat - Input matrix
 * @param[out] kAmat - Scaled output matrix
 * @return None
 */
void ScaleBlkMtx_3x3(const double k, const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *kAmat);

/**
 * @brief Create identity matrix: Imat = eye(3)
 *
 * @param[out] Imat - Identity matrix
 * @return None
 */
void IdentityBlkMtx_3x3(blk_mtx_3x3_t *Imat);

/**
 * @brief Create zero matrix: Zmat = zeros(3,3)
 *
 * @param[out] Zmat - Zero matrix
 * @return None
 */
void ZeroBlkMtx_3x3(blk_mtx_3x3_t *Zmat);

/**
 * @brief Multiply two block matrices: Cmat = Amat * Bmat
 *
 * @param[in] Amat - First matrix
 * @param[in] Bmat - Second matrix
 * @param[out] Cmat - Product matrix
 * @return None
 */
void MultBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const blk_mtx_3x3_t *Bmat, blk_mtx_3x3_t *Cmat);

/**
 * @brief Add two block matrices: Cmat = Amat + Bmat
 *
 * @param[in] Amat - First matrix
 * @param[in] Bmat - Second matrix
 * @param[out] Cmat - Sum matrix
 * @return None
 */
void AddBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const blk_mtx_3x3_t *Bmat, blk_mtx_3x3_t *Cmat);

/**
 * @brief Copy block matrix: Bmat = Amat
 *
 * @param[in] Amat - Source matrix
 * @param[out] Bmat - Destination matrix
 * @return None
 */
void SetBlkMtx_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Bmat);

/**
 * @brief Compute inverse of symmetric block matrix: Ainv = inverse(Amat)
 *
 * @param[in] Amat - Input symmetric matrix
 * @param[out] Ainv - Inverse matrix
 * @return 1 if successful, 0 if singular
 */
uint32_t InvBlkSymMtx1_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Ainv);

/**
 * @brief Transpose block matrix: Bmat = Amat'
 *
 * @param[in] Amat - Input matrix
 * @param[out] Bmat - Transposed matrix
 * @return None
 */
void TranspBlkMtx_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Bmat);

/**
 * @brief Print block matrix for debugging
 *
 * @param[in] Amat - Matrix to print
 * @param[in] ptext - Description text (max 10 chars)
 * @return None
 */
void PrintBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const char ptext[10]);

#endif /* _ALGO_SF_MATRIXMATH_H_ */