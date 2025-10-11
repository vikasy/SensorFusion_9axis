
/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include "algo_sf_matrixmath.h"

// Calculates inverse of a 3x3 "symmetric" matrix Amat
// using only upper triangular part of Amat
// return 0 if no inverse exist, else returns 1
uint32_t SymMatInv3x3(const double Amat[3][3], double InvAmat[3][3])
{
	double    cofac_a00;		// cofactor of Amat[0][0]
	double    cofac_a01;		// cofactor of Amat[0][1]
	double    cofac_a02;		// cofactor of Amat[0][2]
	double    det_A;			// determinant of Amat
	double    rec_det_A;		// reciprocal of the determinant

	uint32_t  ret_val = 0;		// 0 indicates no inverse exist
	uint32_t  i, j;

	// calculate cofactors of the first row
	cofac_a00 = Amat[1][1] * Amat[2][2] - Amat[1][2] * Amat[1][2];
	cofac_a01 = Amat[1][2] * Amat[0][2] - Amat[0][1] * Amat[2][2];
	cofac_a02 = Amat[0][1] * Amat[1][2] - Amat[0][2] * Amat[1][1];

	// calculate the determinant
	det_A = Amat[0][0] * cofac_a00 + Amat[0][1] * cofac_a01 + Amat[0][2] * cofac_a02;

	// calculate the inverse if exists
	if ( fabs(det_A) > EPSILON ) {
		rec_det_A = 1.0 / det_A;
		InvAmat[0][0] = cofac_a00 * rec_det_A;
		InvAmat[1][0] = InvAmat[0][1] = cofac_a01 * rec_det_A;
		InvAmat[2][0] = InvAmat[0][2] = cofac_a02 * rec_det_A;
		InvAmat[1][1] = (Amat[0][0] * Amat[2][2] - Amat[0][2] * Amat[0][2]) * rec_det_A;
		InvAmat[2][1] = InvAmat[1][2] = (Amat[0][2] * Amat[0][1] - Amat[0][0] * Amat[1][2]) * rec_det_A;
		InvAmat[2][2] = (Amat[0][0] * Amat[1][1] - Amat[0][1] * Amat[0][1]) * rec_det_A;
		ret_val = 1;
	}
	else {
		for(i = 0; i < 3; i++) {
			for(j = 0; j < 3; j++) {
				InvAmat[i][j] = 0.0;
			}
		}
	}
	return ret_val;

}


// Standard calculation of product of two 3x3 matrices Amat and Bmat
// ABmat = Amat*Bmat
void MatProd3x3(const double Amat[3][3], const double Bmat[3][3], double ABmat[3][3])
{
	uint32_t   i, j, k;

	for(i = 0; i < 3; i++) {
		for(j = 0; j < 3; j++) {
			ABmat[i][j] = 0.0;
			for(k = 0; k < 3; k++) {
				ABmat[i][j] = ABmat[i][j] + Amat[i][k]*Bmat[k][j];
			}
		}
	}
}


// A = CPMat(Avec)
void CrossPdctMtx_3x3(const double Avec[3], blk_mtx_3x3_t *Amat)
{

	Amat->elem[0][0] =  0.0;
	Amat->elem[0][1] = -Avec[2];
	Amat->elem[0][2] =  Avec[1];
	Amat->elem[1][0] =  Avec[2];
	Amat->elem[1][1] =  0.0;
	Amat->elem[1][2] = -Avec[0];
	Amat->elem[2][0] = -Avec[1];
	Amat->elem[2][1] =  Avec[0];
	Amat->elem[2][2] =  0.0;

}


// kA = k*A
void ScaleBlkMtx_3x3(const double k, const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *kAmat)
{
	uint32_t   i, j;

	for(i = 0; i < 3; i++) {
		for(j = 0; j < 3; j++) {
			kAmat->elem[i][j] = k*Amat->elem[i][j];
		}
	}

}


// I = eye(3)
void IdentityBlkMtx_3x3(blk_mtx_3x3_t *Imat)
{
	uint32_t  i, j;

	for( i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			Imat->elem[i][j] = 0.0;
			if (i == j) Imat->elem[i][j] = 1.0;
		}
	}
}


//Z = zeros(3,3)
void ZeroBlkMtx_3x3(blk_mtx_3x3_t *Zmat)
{
	uint32_t    i, j;

	for( i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			Zmat->elem[i][j] = 0.0;
		}
	}

}


// C = A*B
void MultBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const blk_mtx_3x3_t *Bmat, blk_mtx_3x3_t *Cmat)
{
	uint32_t    i, j, k;

	for( i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			Cmat->elem[i][j] = 0.0;
			for( k = 0; k < 3; k++) {
				Cmat->elem[i][j] += Amat->elem[i][k] * Bmat->elem[k][j];
			}			
		}
	}

}


// C = A + B
void AddBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const blk_mtx_3x3_t *Bmat, blk_mtx_3x3_t *Cmat)
{
	uint32_t     i, j;

	for( i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			Cmat->elem[i][j] = Amat->elem[i][j] + Bmat->elem[i][j];
		}
	}
}


// B = A
void SetBlkMtx_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Bmat)
{
	uint32_t      i, j;

	for( i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			Bmat->elem[i][j] = Amat->elem[i][j];
		}
	}

}


// Ainv = inverse(A), square blk matrix 1 block of 3x3
uint32_t InvBlkSymMtx1_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Ainv)
{
	uint32_t ret_val = 0;
	
	ret_val = SymMatInv3x3(Amat->elem, Ainv->elem);

	return ret_val;

}


// B = A'
void TranspBlkMtx_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Bmat)
{
	uint32_t   i, j;

	for( i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			Bmat->elem[i][j] = Amat->elem[j][i];
		}
	}
}


// Print a block matrix
void PrintBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const char ptext[10])
{
	printf("BlkArrPrint3x3, %s:",ptext);
	for (size_t k = 0; k < 3; k++) {
		for (size_t l = 0; l < 3; l++) {
			printf("%f    ",  Amat->elem[k][l]);
		}
	}
	printf("\n");
}

