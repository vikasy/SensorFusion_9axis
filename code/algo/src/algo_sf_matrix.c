#include "algo_sf_matrix.h"

// compile time constants that are private to this file
#define CORRUPTMATRIX 0.001F			// column vector modulus limit for rotation matrix

// function sets the 3x3 matrix A to the identity matrix
void f3x3matrixAeqI(float A[][3])
{
	float    *pAij;	// pointer to A[i][j]
	uint32_t i, j;		// loop counters

	for (i = 0; i < 3; i++)
	{
		// set pAij to &A[i][j=0]
		pAij = A[i];
		for (j = 0; j < 3; j++)
		{
			*(pAij++) = 0.0F;
		}
		A[i][i] = 1.0F;
	}
	return;
}


// function sets the matrix A to the identity matrix
void fmatrixAeqI(float *A[], uint32_t rc)
{
	// rc = rows and columns in A

	float    *pAij;	    // pointer to A[i][j]
	uint32_t i, j;		// loop counters

	for (i = 0; i < rc; i++)
	{
		// set pAij to &A[i][j=0]
		pAij = A[i];
		for (j = 0; j < rc; j++)
		{
			*(pAij++) = 0.0F;
		}
		A[i][i] = 1.0F;
	}
	return;
}

// function sets every entry in the 3x3 matrix A to a constant scalar
void f3x3matrixAeqScalar(float A[][3], float Scalar)
{
	float    *pAij;	// pointer to A[i][j]
	uint32_t i, j;		// counters

	for (i = 0; i < 3; i++)
	{
		// set pAij to &A[i][j=0]
		pAij = A[i];
		for (j = 0; j < 3; j++)
		{
			*(pAij++) = Scalar;
		}
	}
	return;
}


// function uses Gauss-Jordan elimination to compute the inverse of matrix A in situ
// on exit, A is replaced with its inverse
void fmatrixAeqInvA(float *A[], uint32_t iColInd[], uint32_t iRowInd[], uint32_t iPivot[], uint32_t isize, uint32_t *pierror)
{
	float    largest;					// largest element used for pivoting
	float    scaling;					// scaling factor in pivoting
	float    recippiv;					// reciprocal of pivot element
	float    ftmp;						// temporary variable used in swaps
	uint32_t i, j, k, l, m;	    		// index counters
	int      n;
	uint32_t iPivotRow, iPivotCol;		// row and column of pivot element

	// to avoid compiler warnings
	iPivotRow = iPivotCol = 0;
	
	// default to successful inversion
	*pierror = false;
	
	// initialize the pivot array to 0
	for (j = 0; j < isize; j++)
	{
		iPivot[j] = 0;
	}

	// main loop i over the dimensions of the square matrix A
	for (i = 0; i < isize; i++)
	{
		// zero the largest element found for pivoting
		largest = 0.0F;
		// loop over candidate rows j
		for (j = 0; j < isize; j++)
		{
			// check if row j has been previously pivoted
			if (iPivot[j] != 1)
			{
				// loop over candidate columns k
				for (k = 0; k < isize; k++)
				{
					// check if column k has previously been pivoted
					if (iPivot[k] == 0)
					{
						// check if the pivot element is the largest found so far
						if (fabsf(A[j][k]) >= largest)
						{
							// and store this location as the current best candidate for pivoting
							iPivotRow = j;
							iPivotCol = k;
							largest = (float) fabsf(A[iPivotRow][iPivotCol]);
						}
					}
					else if (iPivot[k] > 1)
					{
						// zero determinant situation: exit with identity matrix and set error flag
						fmatrixAeqI(A, isize);
						*pierror = true;
						return;
					}
				}
			}
		}
		// increment the entry in iPivot to denote it has been selected for pivoting
		iPivot[iPivotCol]++;

		// check the pivot rows iPivotRow and iPivotCol are not the same before swapping
		if (iPivotRow != iPivotCol)
		{
			// loop over columns l
			for (l = 0; l < isize; l++)
			{
				// and swap all elements of rows iPivotRow and iPivotCol
				ftmp = A[iPivotRow][l];
				A[iPivotRow][l] = A[iPivotCol][l];
				A[iPivotCol][l] = ftmp;
			}
		}

		// record that on the i-th iteration rows iPivotRow and iPivotCol were swapped
		iRowInd[i] = iPivotRow;
		iColInd[i] = iPivotCol;

		// check for zero on-diagonal element (singular matrix) and return with identity matrix if detected
		if (A[iPivotCol][iPivotCol] == 0.0F)
		{
			// zero determinant situation: exit with identity matrix and set error flag
			fmatrixAeqI(A, isize);
			*pierror = true;
			return;
		}

		// calculate the reciprocal of the pivot element knowing it's non-zero
		recippiv = 1.0F / A[iPivotCol][iPivotCol];
		// by definition, the diagonal element normalizes to 1
		A[iPivotCol][iPivotCol] = 1.0F;
		// multiply all of row iPivotCol by the reciprocal of the pivot element including the diagonal element
		// the diagonal element A[iPivotCol][iPivotCol] now has value equal to the reciprocal of its previous value
		for (l = 0; l < isize; l++)
		{
			if (A[iPivotCol][l] != 0.0F)
				A[iPivotCol][l] *= recippiv;
		}
		// loop over all rows m of A
		for (m = 0; m < isize; m++)
		{
			if (m != iPivotCol)
			{
				// scaling factor for this row m is in column iPivotCol
				scaling = A[m][iPivotCol];
				// zero this element
				A[m][iPivotCol] = 0.0F;
				// loop over all columns l of A and perform elimination
				for (l = 0; l < isize; l++)
				{
					if ((A[iPivotCol][l] != 0.0F) && (scaling != 0.0F))
						A[m][l] -= A[iPivotCol][l] * scaling;
				}
			}
		}
	} // end of loop i over the matrix dimensions

	// finally, loop in inverse order to apply the missing column swaps
	for(n = isize - 1; n >= 0; n--)
	{
		// set i and j to the two columns to be swapped
		i = iRowInd[n];
		j = iColInd[n];

		// check that the two columns i and j to be swapped are not the same
		if (i != j)
		{
			// loop over all rows k to swap columns i and j of A
			for (k = 0; k < isize; k++)
			{
				ftmp = A[k][i];
				A[k][i] = A[k][j];
				A[k][j] = ftmp;
			}
		}
	}

	return;
}
