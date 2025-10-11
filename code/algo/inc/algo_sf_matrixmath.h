#ifndef _ALGO_SF_MATRIXMATH_H_
#define _ALGO_SF_MATRIXMATH_H_

/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include "algo_sf_types.h"

// 3x3 matrix data type
typedef struct blk_mtx_3x3 {
	double elem[3][3];
} blk_mtx_3x3_t;


uint32_t SymMatInv3x3(const double Amat[3][3], double InvAmat[3][3]);

void MatProd3x3(const double Amat[3][3], const double Bmat[3][3], double ABmat[3][3]);

// A = CPMat(Avec)
void CrossPdctMtx_3x3(const double Avec[3], blk_mtx_3x3_t *Amat);

// kA = k*A
void ScaleBlkMtx_3x3(const double k, const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *kAmat);

// I = eye(3)
void IdentityBlkMtx_3x3(blk_mtx_3x3_t *Imat);

//Z = zeros(3,3)
void ZeroBlkMtx_3x3(blk_mtx_3x3_t *Zmat);

// C = A*B
void MultBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const blk_mtx_3x3_t *Bmat, blk_mtx_3x3_t *Cmat);

// C = A + B
void AddBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const blk_mtx_3x3_t *Bmat, blk_mtx_3x3_t *Cmat);

// B = A
void SetBlkMtx_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Bmat);

// Ainv = inverse(A), square blk matrix 1 block of 3x3
uint32_t InvBlkSymMtx1_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Ainv);

// B = A'
void TranspBlkMtx_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Bmat);

// Print a block matrix
void PrintBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const char ptext[10]);

#endif /* _ALGO_SF_MATRIXMATH_H_ */