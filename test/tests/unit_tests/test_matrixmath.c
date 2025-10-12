/*******************************************************************************
 * Unit Tests for Matrix Math Functions
 *
 * Author: Vikas Yadav
 * Date: 2025-10-11
 *
 * Tests: code/algo/src/algo_sf_matrixmath.c
 *
 * Test Framework: Unity
 ******************************************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "algo_sf_matrixmath.h"
#include "test_vectors_matrix.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TEST_EPSILON 1e-6

/*******************************************************************************
 * Unity Framework Setup/Teardown
 ******************************************************************************/

void setUp(void) {
}

void tearDown(void) {
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

static int matrix_equals(const double m1[3][3], const double m2[3][3], double eps) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (fabs(m1[i][j] - m2[i][j]) > eps) {
                return 0;
            }
        }
    }
    return 1;
}

static int matrix_is_identity(const double m[3][3], double eps) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double expected = (i == j) ? 1.0 : 0.0;
            if (fabs(m[i][j] - expected) > eps) {
                return 0;
            }
        }
    }
    return 1;
}

/*******************************************************************************
 * Matrix Multiplication Tests (using MatProd3x3)
 ******************************************************************************/

void test_matrix_multiply_identity(void) {
    double identity[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    double test_matrix[3][3] = {
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0},
        {7.0, 8.0, 9.0}
    };

    double result[3][3];

    // I * M = M
    MatProd3x3(identity, test_matrix, result);
    TEST_ASSERT_TRUE(matrix_equals(result, test_matrix, TEST_EPSILON));
}

void test_matrix_multiply_zero(void) {
    double zero[3][3] = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    double test_matrix[3][3] = {
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0},
        {7.0, 8.0, 9.0}
    };

    double result[3][3];
    double expected_zero[3][3] = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    // 0 * M = 0
    MatProd3x3(zero, test_matrix, result);
    TEST_ASSERT_TRUE(matrix_equals(result, expected_zero, TEST_EPSILON));
}

void test_matrix_multiply_standard(void) {
    double m1[3][3] = {
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0},
        {7.0, 8.0, 9.0}
    };

    double m2[3][3] = {
        {9.0, 8.0, 7.0},
        {6.0, 5.0, 4.0},
        {3.0, 2.0, 1.0}
    };

    double result[3][3];

    // Manually calculated expected result
    double expected[3][3] = {
        {30.0, 24.0, 18.0},
        {84.0, 69.0, 54.0},
        {138.0, 114.0, 90.0}
    };

    MatProd3x3(m1, m2, result);
    TEST_ASSERT_TRUE(matrix_equals(result, expected, TEST_EPSILON));
}

/*******************************************************************************
 * Matrix Transpose Tests (using TranspBlkMtx_3x3)
 ******************************************************************************/

void test_matrix_transpose_identity(void) {
    blk_mtx_3x3_t identity;
    IdentityBlkMtx_3x3(&identity);

    blk_mtx_3x3_t result;

    // I^T = I
    TranspBlkMtx_3x3(&identity, &result);
    TEST_ASSERT_TRUE(matrix_equals(result.elem, identity.elem, TEST_EPSILON));
}

void test_matrix_transpose_standard(void) {
    blk_mtx_3x3_t m;
    m.elem[0][0] = 1.0; m.elem[0][1] = 2.0; m.elem[0][2] = 3.0;
    m.elem[1][0] = 4.0; m.elem[1][1] = 5.0; m.elem[1][2] = 6.0;
    m.elem[2][0] = 7.0; m.elem[2][1] = 8.0; m.elem[2][2] = 9.0;

    blk_mtx_3x3_t result;
    double expected[3][3] = {
        {1.0, 4.0, 7.0},
        {2.0, 5.0, 8.0},
        {3.0, 6.0, 9.0}
    };

    TranspBlkMtx_3x3(&m, &result);
    TEST_ASSERT_TRUE(matrix_equals(result.elem, expected, TEST_EPSILON));
}

void test_matrix_transpose_double(void) {
    // (M^T)^T = M
    blk_mtx_3x3_t m;
    m.elem[0][0] = 1.0; m.elem[0][1] = 2.0; m.elem[0][2] = 3.0;
    m.elem[1][0] = 4.0; m.elem[1][1] = 5.0; m.elem[1][2] = 6.0;
    m.elem[2][0] = 7.0; m.elem[2][1] = 8.0; m.elem[2][2] = 9.0;

    blk_mtx_3x3_t temp, result;

    TranspBlkMtx_3x3(&m, &temp);
    TranspBlkMtx_3x3(&temp, &result);

    TEST_ASSERT_TRUE(matrix_equals(result.elem, m.elem, TEST_EPSILON));
}

/*******************************************************************************
 * Matrix Addition Tests (using AddBlkMtx_3x3)
 * Note: No subtraction function in API, only addition
 ******************************************************************************/

void test_matrix_add_zero(void) {
    blk_mtx_3x3_t m;
    m.elem[0][0] = 1.0; m.elem[0][1] = 2.0; m.elem[0][2] = 3.0;
    m.elem[1][0] = 4.0; m.elem[1][1] = 5.0; m.elem[1][2] = 6.0;
    m.elem[2][0] = 7.0; m.elem[2][1] = 8.0; m.elem[2][2] = 9.0;

    blk_mtx_3x3_t zero;
    ZeroBlkMtx_3x3(&zero);

    blk_mtx_3x3_t result;

    // M + 0 = M
    AddBlkMtx_3x3(&m, &zero, &result);
    TEST_ASSERT_TRUE(matrix_equals(result.elem, m.elem, TEST_EPSILON));
}

void test_matrix_add_standard(void) {
    blk_mtx_3x3_t m1;
    m1.elem[0][0] = 1.0; m1.elem[0][1] = 2.0; m1.elem[0][2] = 3.0;
    m1.elem[1][0] = 4.0; m1.elem[1][1] = 5.0; m1.elem[1][2] = 6.0;
    m1.elem[2][0] = 7.0; m1.elem[2][1] = 8.0; m1.elem[2][2] = 9.0;

    blk_mtx_3x3_t m2;
    m2.elem[0][0] = 9.0; m2.elem[0][1] = 8.0; m2.elem[0][2] = 7.0;
    m2.elem[1][0] = 6.0; m2.elem[1][1] = 5.0; m2.elem[1][2] = 4.0;
    m2.elem[2][0] = 3.0; m2.elem[2][1] = 2.0; m2.elem[2][2] = 1.0;

    blk_mtx_3x3_t result;
    double expected[3][3] = {
        {10.0, 10.0, 10.0},
        {10.0, 10.0, 10.0},
        {10.0, 10.0, 10.0}
    };

    AddBlkMtx_3x3(&m1, &m2, &result);
    TEST_ASSERT_TRUE(matrix_equals(result.elem, expected, TEST_EPSILON));
}

/*******************************************************************************
 * Matrix Scalar Operations Tests (using ScaleBlkMtx_3x3)
 ******************************************************************************/

void test_matrix_scalar_multiply_identity(void) {
    blk_mtx_3x3_t m;
    m.elem[0][0] = 1.0; m.elem[0][1] = 2.0; m.elem[0][2] = 3.0;
    m.elem[1][0] = 4.0; m.elem[1][1] = 5.0; m.elem[1][2] = 6.0;
    m.elem[2][0] = 7.0; m.elem[2][1] = 8.0; m.elem[2][2] = 9.0;

    blk_mtx_3x3_t result;

    // M * 1 = M
    ScaleBlkMtx_3x3(1.0, &m, &result);
    TEST_ASSERT_TRUE(matrix_equals(result.elem, m.elem, TEST_EPSILON));
}

void test_matrix_scalar_multiply_zero(void) {
    blk_mtx_3x3_t m;
    m.elem[0][0] = 1.0; m.elem[0][1] = 2.0; m.elem[0][2] = 3.0;
    m.elem[1][0] = 4.0; m.elem[1][1] = 5.0; m.elem[1][2] = 6.0;
    m.elem[2][0] = 7.0; m.elem[2][1] = 8.0; m.elem[2][2] = 9.0;

    blk_mtx_3x3_t result;
    double expected_zero[3][3] = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    // M * 0 = 0
    ScaleBlkMtx_3x3(0.0, &m, &result);
    TEST_ASSERT_TRUE(matrix_equals(result.elem, expected_zero, TEST_EPSILON));
}

void test_matrix_scalar_multiply_negative(void) {
    blk_mtx_3x3_t m;
    m.elem[0][0] = 1.0; m.elem[0][1] = 2.0; m.elem[0][2] = 3.0;
    m.elem[1][0] = 4.0; m.elem[1][1] = 5.0; m.elem[1][2] = 6.0;
    m.elem[2][0] = 7.0; m.elem[2][1] = 8.0; m.elem[2][2] = 9.0;

    blk_mtx_3x3_t result;
    double expected[3][3] = {
        {-1.0, -2.0, -3.0},
        {-4.0, -5.0, -6.0},
        {-7.0, -8.0, -9.0}
    };

    // M * (-1) = -M
    ScaleBlkMtx_3x3(-1.0, &m, &result);
    TEST_ASSERT_TRUE(matrix_equals(result.elem, expected, TEST_EPSILON));
}

/*******************************************************************************
 * Verified Test Cases (using numpy-generated test vectors)
 ******************************************************************************/

// Test MatProd3x3 with verified A*B multiplication
void test_matrix_multiply_verified(void) {
    double result[3][3];

    MatProd3x3(test_matrix_multiply_AB_A, test_matrix_multiply_AB_B, result);

    // Compare with numpy-generated expected result
    TEST_ASSERT_TRUE(matrix_equals(result, test_matrix_multiply_AB_expected, TEST_EPSILON));
}

// Test TranspBlkMtx_3x3 with verified transpose
void test_matrix_transpose_verified(void) {
    blk_mtx_3x3_t input;
    // Copy test vector to block matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            input.elem[i][j] = test_matrix_transpose_A[i][j];
        }
    }

    blk_mtx_3x3_t result;
    TranspBlkMtx_3x3(&input, &result);

    // Compare with numpy-generated expected result
    TEST_ASSERT_TRUE(matrix_equals(result.elem, test_matrix_transpose_A_expected, TEST_EPSILON));
}

// Test AddBlkMtx_3x3 with verified addition
void test_matrix_add_verified(void) {
    blk_mtx_3x3_t m1, m2;

    // Copy test vectors to block matrices
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m1.elem[i][j] = test_matrix_add_AB_A[i][j];
            m2.elem[i][j] = test_matrix_add_AB_B[i][j];
        }
    }

    blk_mtx_3x3_t result;
    AddBlkMtx_3x3(&m1, &m2, &result);

    // Compare with numpy-generated expected result
    TEST_ASSERT_TRUE(matrix_equals(result.elem, test_matrix_add_AB_expected, TEST_EPSILON));
}

// Test ScaleBlkMtx_3x3 with verified scalar multiplication
void test_matrix_scalar_multiply_verified(void) {
    blk_mtx_3x3_t input;

    // Copy test vector to block matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            input.elem[i][j] = test_matrix_scalar_multiply_A_input[i][j];
        }
    }

    blk_mtx_3x3_t result;
    ScaleBlkMtx_3x3(test_scalar_scalar_multiply_A, &input, &result);

    // Compare with numpy-generated expected result
    TEST_ASSERT_TRUE(matrix_equals(result.elem, test_matrix_scalar_multiply_A_expected, TEST_EPSILON));
}

/*******************************************************************************
 * Main Test Runner
 ******************************************************************************/

int main(void) {
    UNITY_BEGIN();

    // Matrix multiply tests
    RUN_TEST(test_matrix_multiply_identity);
    RUN_TEST(test_matrix_multiply_zero);
    RUN_TEST(test_matrix_multiply_standard);

    // Matrix transpose tests
    RUN_TEST(test_matrix_transpose_identity);
    RUN_TEST(test_matrix_transpose_standard);
    RUN_TEST(test_matrix_transpose_double);

    // Matrix add tests
    RUN_TEST(test_matrix_add_zero);
    RUN_TEST(test_matrix_add_standard);

    // Matrix scalar operations
    RUN_TEST(test_matrix_scalar_multiply_identity);
    RUN_TEST(test_matrix_scalar_multiply_zero);
    RUN_TEST(test_matrix_scalar_multiply_negative);

    // Verified test cases (numpy-generated)
    RUN_TEST(test_matrix_multiply_verified);
    RUN_TEST(test_matrix_transpose_verified);
    RUN_TEST(test_matrix_add_verified);
    RUN_TEST(test_matrix_scalar_multiply_verified);

    return UNITY_END();
}
