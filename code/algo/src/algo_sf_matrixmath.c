/*******************************************************************************
 * @file    algo_sf_matrixmath.c
 * @brief   Implementation of 3x3 matrix mathematics operations
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#include "algo_sf_matrixmath.h"

/****************************************************************************
 * Private Constants
 ****************************************************************************/

/* Matrix dimension constants */
#define MATRIX_DIM_3             3        /* 3x3 matrix dimension */

/* Matrix initialization values */
#define MATRIX_ZERO_DBL          0.0      /* Zero for double precision matrices */
#define MATRIX_ONE_DBL           1.0      /* Identity diagonal element */

/* Matrix initialization values for float */
#define MATRIX_ZERO_FLT          0.0F     /* Zero for single precision matrices */
#define MATRIX_ONE_FLT           1.0F     /* Identity diagonal element for float */

/* Matrix negation constant */
#define MATRIX_NEG_ONE           -1.0     /* Negative one for sign changes */

/****************************************************************************
 * Symmetric Matrix Inversion Functions
 ****************************************************************************/

/**
 * @brief Calculates inverse of a 3x3 symmetric matrix
 *
 * Uses only upper triangular part of Amat to compute inverse. More efficient
 * than general matrix inversion since it exploits symmetry. Uses cofactor
 * expansion method.
 *
 * @param[in] Amat - Input 3x3 symmetric matrix (uses upper triangular)
 * @param[out] InvAmat - Output 3x3 inverse matrix
 * @return 1 if inverse exists, 0 if matrix is singular (det < EPSILON)
 *
 * @note Matrix must be symmetric for correct results
 */
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
		rec_det_A = MATRIX_ONE_DBL / det_A;
		InvAmat[0][0] = cofac_a00 * rec_det_A;
		InvAmat[1][0] = InvAmat[0][1] = cofac_a01 * rec_det_A;
		InvAmat[2][0] = InvAmat[0][2] = cofac_a02 * rec_det_A;
		InvAmat[1][1] = (Amat[0][0] * Amat[2][2] - Amat[0][2] * Amat[0][2]) * rec_det_A;
		InvAmat[2][1] = InvAmat[1][2] = (Amat[0][2] * Amat[0][1] - Amat[0][0] * Amat[1][2]) * rec_det_A;
		InvAmat[2][2] = (Amat[0][0] * Amat[1][1] - Amat[0][1] * Amat[0][1]) * rec_det_A;
		ret_val = 1;
	}
	else {
		for(i = 0; i < MATRIX_DIM_3; i++) {
			for(j = 0; j < MATRIX_DIM_3; j++) {
				InvAmat[i][j] = MATRIX_ZERO_DBL;
			}
		}
	}
	return ret_val;

}

/****************************************************************************
 * Standard Matrix Operations
 ****************************************************************************/

/**
 * @brief Standard calculation of product of two 3x3 matrices
 *
 * Computes matrix multiplication: ABmat = Amat * Bmat using standard
 * triple loop algorithm. Result is stored in output matrix.
 *
 * @param[in] Amat - First input 3x3 matrix (left operand)
 * @param[in] Bmat - Second input 3x3 matrix (right operand)
 * @param[out] ABmat - Output 3x3 matrix product
 * @return None
 *
 * @note Matrix multiplication is non-commutative: AB ≠ BA
 */
void MatProd3x3(const double Amat[3][3], const double Bmat[3][3], double ABmat[3][3])
{
	uint32_t   i, j, k;

	for(i = 0; i < MATRIX_DIM_3; i++) {
		for(j = 0; j < MATRIX_DIM_3; j++) {
			ABmat[i][j] = MATRIX_ZERO_DBL;
			for(k = 0; k < MATRIX_DIM_3; k++) {
				ABmat[i][j] = ABmat[i][j] + Amat[i][k]*Bmat[k][j];
			}
		}
	}
}

/****************************************************************************
 * Cross Product Matrix Operations
 ****************************************************************************/

/**
 * @brief Creates cross product matrix from 3D vector
 *
 * Constructs the skew-symmetric cross product matrix A = CPMat(Avec) such that
 * A*x = Avec × x (cross product). Matrix form:
 * A = [  0    -Az    Ay  ]
 *     [ Az     0    -Ax  ]
 *     [-Ay    Ax     0   ]
 *
 * @param[in] Avec - Input 3D vector [Ax, Ay, Az]
 * @param[out] Amat - Output 3x3 skew-symmetric cross product matrix
 * @return None
 *
 * @note Result matrix is always skew-symmetric (A^T = -A)
 */
void CrossPdctMtx_3x3(const double Avec[3], blk_mtx_3x3_t *Amat)
{

	Amat->elem[0][0] =  MATRIX_ZERO_DBL;
	Amat->elem[0][1] = -Avec[2];
	Amat->elem[0][2] =  Avec[1];
	Amat->elem[1][0] =  Avec[2];
	Amat->elem[1][1] =  MATRIX_ZERO_DBL;
	Amat->elem[1][2] = -Avec[0];
	Amat->elem[2][0] = -Avec[1];
	Amat->elem[2][1] =  Avec[0];
	Amat->elem[2][2] =  MATRIX_ZERO_DBL;

}

/****************************************************************************
 * Block Matrix Scalar Operations
 ****************************************************************************/

/**
 * @brief Scales a 3x3 block matrix by scalar value
 *
 * Computes element-wise scalar multiplication: kA = k * A where k is
 * a scalar and A is a 3x3 matrix.
 *
 * @param[in] k - Scalar multiplier
 * @param[in] Amat - Input 3x3 block matrix
 * @param[out] kAmat - Output scaled 3x3 matrix
 * @return None
 */
void ScaleBlkMtx_3x3(const double k, const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *kAmat)
{
	uint32_t   i, j;

	for(i = 0; i < MATRIX_DIM_3; i++) {
		for(j = 0; j < MATRIX_DIM_3; j++) {
			kAmat->elem[i][j] = k*Amat->elem[i][j];
		}
	}

}

/****************************************************************************
 * Block Matrix Initialization Functions
 ****************************************************************************/

/**
 * @brief Creates 3x3 identity matrix
 *
 * Initializes matrix to identity: I = eye(3) with ones on diagonal
 * and zeros elsewhere. I*A = A*I = A for any matrix A.
 *
 * @param[out] Imat - Output 3x3 identity matrix
 * @return None
 *
 * @note Identity matrix satisfies: I[i][j] = 1 if i==j, else 0
 */
void IdentityBlkMtx_3x3(blk_mtx_3x3_t *Imat)
{
	uint32_t  i, j;

	for( i = 0; i < MATRIX_DIM_3; i++) {
		for( j = 0; j < MATRIX_DIM_3; j++) {
			Imat->elem[i][j] = MATRIX_ZERO_DBL;
			if (i == j) Imat->elem[i][j] = MATRIX_ONE_DBL;
		}
	}
}

/**
 * @brief Creates 3x3 zero matrix
 *
 * Initializes all elements to zero: Z = zeros(3,3). Zero matrix is
 * additive identity: A + Z = Z + A = A for any matrix A.
 *
 * @param[out] Zmat - Output 3x3 zero matrix
 * @return None
 */
void ZeroBlkMtx_3x3(blk_mtx_3x3_t *Zmat)
{
	uint32_t    i, j;

	for( i = 0; i < MATRIX_DIM_3; i++) {
		for( j = 0; j < MATRIX_DIM_3; j++) {
			Zmat->elem[i][j] = MATRIX_ZERO_DBL;
		}
	}

}

/****************************************************************************
 * Block Matrix Arithmetic Operations
 ****************************************************************************/

/**
 * @brief Multiplies two 3x3 block matrices
 *
 * Computes matrix product: C = A * B using standard triple loop algorithm.
 * Elements computed as: C[i][j] = sum(A[i][k] * B[k][j]) for k=0..2
 *
 * @param[in] Amat - First input 3x3 block matrix (left operand)
 * @param[in] Bmat - Second input 3x3 block matrix (right operand)
 * @param[out] Cmat - Output 3x3 product matrix
 * @return None
 *
 * @note Matrix multiplication is non-commutative
 */
void MultBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const blk_mtx_3x3_t *Bmat, blk_mtx_3x3_t *Cmat)
{
	uint32_t    i, j, k;

	for( i = 0; i < MATRIX_DIM_3; i++) {
		for( j = 0; j < MATRIX_DIM_3; j++) {
			Cmat->elem[i][j] = MATRIX_ZERO_DBL;
			for( k = 0; k < MATRIX_DIM_3; k++) {
				Cmat->elem[i][j] += Amat->elem[i][k] * Bmat->elem[k][j];
			}
		}
	}

}

/**
 * @brief Adds two 3x3 block matrices
 *
 * Computes element-wise matrix addition: C = A + B. Each element
 * computed as: C[i][j] = A[i][j] + B[i][j]
 *
 * @param[in] Amat - First input 3x3 block matrix
 * @param[in] Bmat - Second input 3x3 block matrix
 * @param[out] Cmat - Output 3x3 sum matrix
 * @return None
 *
 * @note Matrix addition is commutative: A + B = B + A
 */
void AddBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const blk_mtx_3x3_t *Bmat, blk_mtx_3x3_t *Cmat)
{
	uint32_t     i, j;

	for( i = 0; i < MATRIX_DIM_3; i++) {
		for( j = 0; j < MATRIX_DIM_3; j++) {
			Cmat->elem[i][j] = Amat->elem[i][j] + Bmat->elem[i][j];
		}
	}
}

/****************************************************************************
 * Block Matrix Copy and Transpose Operations
 ****************************************************************************/

/**
 * @brief Copies one 3x3 block matrix to another
 *
 * Performs element-wise copy: B = A. All elements of A are copied to B.
 *
 * @param[in] Amat - Input 3x3 source block matrix
 * @param[out] Bmat - Output 3x3 destination block matrix
 * @return None
 */
void SetBlkMtx_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Bmat)
{
	uint32_t      i, j;

	for( i = 0; i < MATRIX_DIM_3; i++) {
		for( j = 0; j < MATRIX_DIM_3; j++) {
			Bmat->elem[i][j] = Amat->elem[i][j];
		}
	}

}

/**
 * @brief Computes inverse of symmetric 3x3 block matrix
 *
 * Calculates matrix inverse: Ainv = inverse(A) for a single block of 3x3.
 * Uses symmetric matrix inversion for efficiency.
 *
 * @param[in] Amat - Input 3x3 symmetric block matrix
 * @param[out] Ainv - Output 3x3 inverse matrix
 * @return 1 if inverse exists, 0 if matrix is singular
 *
 * @note Input matrix must be symmetric and non-singular
 */
uint32_t InvBlkSymMtx1_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Ainv)
{
	uint32_t ret_val = 0;

	ret_val = SymMatInv3x3(Amat->elem, Ainv->elem);

	return ret_val;

}

/**
 * @brief Transposes a 3x3 block matrix
 *
 * Computes matrix transpose: B = A^T where B[i][j] = A[j][i].
 * Swaps rows and columns of input matrix.
 *
 * @param[in] Amat - Input 3x3 block matrix
 * @param[out] Bmat - Output 3x3 transposed matrix
 * @return None
 *
 * @note For symmetric matrices: A^T = A
 */
void TranspBlkMtx_3x3(const blk_mtx_3x3_t *Amat, blk_mtx_3x3_t *Bmat)
{
	uint32_t   i, j;

	for( i = 0; i < MATRIX_DIM_3; i++) {
		for( j = 0; j < MATRIX_DIM_3; j++) {
			Bmat->elem[i][j] = Amat->elem[j][i];
		}
	}
}

/****************************************************************************
 * Block Matrix Utility Functions
 ****************************************************************************/

/**
 * @brief Prints a 3x3 block matrix to console
 *
 * Displays matrix elements with optional text label. Format:
 * "BlkArrPrint3x3, <ptext>: elem[0][0] elem[0][1] ... elem[2][2]"
 *
 * @param[in] Amat - Input 3x3 block matrix to print
 * @param[in] ptext - Text label (max 10 characters) to identify matrix
 * @return None
 *
 * @note Uses printf, may not be suitable for embedded systems
 */
void PrintBlkMtx_3x3(const blk_mtx_3x3_t *Amat, const char ptext[10])
{
	printf("BlkArrPrint3x3, %s:",ptext);
	for (size_t k = 0; k < MATRIX_DIM_3; k++) {
		for (size_t l = 0; l < MATRIX_DIM_3; l++) {
			printf("%f    ",  Amat->elem[k][l]);
		}
	}
	printf("\n");
}
