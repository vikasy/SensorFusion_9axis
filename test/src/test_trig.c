
#include "main.h"
#include <inttypes.h>

#include "algo_sf_approxmath.h"

#include "TestData_NormQ.h"
#include "TestData_ProdPQ.h"
#include "TestData_QIntegrate.h"
#include "TestData_QIntegrate1.h"
#include "TestData_RotMtx2Quat.h"
#include "TestData_Quat2RotMtx.h"
#include "TestData_RotMtx2Angles.h"
#include "TrigTestData.h"

#define TESTNORM
#define TESTPROD
#define TESTINTEG
#define TESTINTEG1
#define TESTQ2R
#define TESTR2Q
#define TESTR2A
#define TESTMATRIX
#define TESTTRIG

#ifdef TESTTRIG
static void test_TrigApproxFunctions(void)
{

	// test trig approx functions
	bool                result;
	size_t              length;

	float In_sincos_i, In_asin_i, In_acos_i, In_atan_i, In_atan2_yi, In_atan2_xi;
	float Out_sin_i, Out_cos_i, Out_asin_i, Out_acos_i, Out_atan_i, Out_atan2_i;
	float sin_i, cos_i, asin_i, acos_i, atan_i, atan2_i;
	float unit1, unit2, unit3;

	length = sizeof(Input_Data) / (6 * sizeof(float));

	for (size_t i = 0; i < length; i++)
	{
		In_sincos_i = Input_Data[i][0];
		In_asin_i = Input_Data[i][1];
		In_acos_i = Input_Data[i][2];
		In_atan_i = Input_Data[i][3];
		In_atan2_yi = Input_Data[i][4];
		In_atan2_xi = Input_Data[i][5];

		sincos_approx(In_sincos_i, &Out_sin_i, &Out_cos_i);
		Out_asin_i = asin_approx(In_asin_i)*DEG2RAD;
		Out_acos_i = acos_approx(In_acos_i)*DEG2RAD;
		Out_atan_i = atan_approx(In_atan_i)*DEG2RAD;
		Out_atan2_i = atan2_approx(In_atan2_yi, In_atan2_xi)*DEG2RAD;

		sin_i = sin(In_sincos_i);
		cos_i = cos(In_sincos_i);
		asin_i = asin(In_asin_i)*DEG2RAD;
		acos_i = acos(In_acos_i)*DEG2RAD;
		atan_i = atan(In_atan_i)*DEG2RAD;
		atan2_i = atan2(In_atan2_yi, In_atan2_xi)*DEG2RAD;

		result = true;
		if (fabs(Out_sin_i - Output_Data[i][0]) > 1e-6)	    result = false;
		if (fabs(Out_cos_i - Output_Data[i][1]) > 1e-6)	    result = false;
		//if (fabs(Out_asin_i - Output_Data[i][2]) > 1e-6)	result = false;
		//if (fabs(Out_acos_i - Output_Data[i][3]) > 1e-6)	result = false;
		//if (fabs(Out_atan_i - Output_Data[i][4]) > 1e-6)	result = false;
		//if (fabs(Out_atan2_i - Output_Data[i][5]) > 1e-6)	result = false;

		unit1 = sqrt(Output_Data[i][0] * Output_Data[i][0] + Output_Data[i][1] * Output_Data[i][1]);
		unit2 = sqrt(Out_sin_i * Out_sin_i + Out_cos_i * Out_cos_i);
		unit3 = sqrt(sin_i * sin_i + cos_i * cos_i);

		printf("i=%d, Result: %d\r\n", i, result);
		if (result == false) {
			printf("Expected Output: %f, %f, %f, %f, %f, %f, %f \n", unit1, Output_Data[i][0], Output_Data[i][1], Output_Data[i][2], Output_Data[i][3], Output_Data[i][4], Output_Data[i][5]);
			printf("Actual Output: %f, %f, %f, %f, %f, %f, %f \n", unit2, Out_sin_i, Out_cos_i, Out_asin_i, Out_acos_i, Out_atan_i, Out_atan2_i);
			printf("MSVC Output: %f, %f, %f, %f, %f, %f, %f \n", unit3, sin_i, cos_i, asin_i, acos_i, atan_i, atan2_i);
		}
	}

}
#endif



#ifdef TESTNORM
static void test_QuatNormal(void)
{

	// test QuatNormal
	quaternion_double_t QuatIn, QuatOut;
	bool                result;
	size_t              length;

	length = sizeof(testin1) / (4 * sizeof(double));

	for (size_t i = 0; i < length; i++)
	{
		QuatIn.q0 = testin1[i][0];
		QuatIn.q1 = testin1[i][1];
		QuatIn.q2 = testin1[i][2];
		QuatIn.q3 = testin1[i][3];

		QuatNormal(&QuatIn, &QuatOut);

		result = true;
		if (fabs(QuatOut.q0 - testout1[i][0]) > 1e-5)	result = false;
		if (fabs(QuatOut.q1 - testout1[i][1]) > 1e-5)	result = false;
		if (fabs(QuatOut.q2 - testout1[i][2]) > 1e-5)	result = false;
		if (fabs(QuatOut.q3 - testout1[i][3]) > 1e-5)	result = false;

		printf("i=%d, Result: %d\r\n", i, result);
		if (result == false) {
			printf("Expected Output: %f, %f, %f, %f \n", testout1[i][0], testout1[i][1], testout1[i][2], testout1[i][3]);
			printf("Actual Output: %f, %f, %f, %f \n", QuatOut.q0, QuatOut.q1, QuatOut.q2, QuatOut.q3);
		}
	}

}
#endif


#ifdef TESTPROD
static void test_QuatProduct(void)
{

	// test QuatNormal
	quaternion_double_t QuatInP, QuatInQ, QuatPQ;
	bool                result;
	size_t              length;

	length = sizeof(testin1) / (4 * sizeof(double));

	for (size_t i = 0; i < length; i++)
	{
		QuatInP.q0 = testinP[i][0];
		QuatInP.q1 = testinP[i][1];
		QuatInP.q2 = testinP[i][2];
		QuatInP.q3 = testinP[i][3];

		QuatInQ.q0 = testinQ[i][0];
		QuatInQ.q1 = testinQ[i][1];
		QuatInQ.q2 = testinQ[i][2];
		QuatInQ.q3 = testinQ[i][3];

		QuatProduct(&QuatInP, &QuatInQ, &QuatPQ);

		result = true;
		if (fabs(QuatPQ.q0 - testoutPQ[i][0]) > 1e-5)	result = false;
		if (fabs(QuatPQ.q1 - testoutPQ[i][1]) > 1e-5)	result = false;
		if (fabs(QuatPQ.q2 - testoutPQ[i][2]) > 1e-5)	result = false;
		if (fabs(QuatPQ.q3 - testoutPQ[i][3]) > 1e-5)	result = false;

		printf("i=%d, Result: %d\r\n", i, result);
		if (result == false) {
			printf("Expected Output: %f, %f, %f, %f \n", testoutPQ[i][0], testoutPQ[i][1], testoutPQ[i][2], testoutPQ[i][3]);
			printf("Actual Output: %f, %f, %f, %f \n", QuatPQ.q0, QuatPQ.q1, QuatPQ.q2, QuatPQ.q3);
		}
	}

}
#endif


#ifdef TESTINTEG
static void test_QuatIntegrate(void)
{

	// test QuatNormal
	quaternion_double_t Quatpre, QuatIntOut;
	bool                result;
	size_t              length;
	double              ang_rate_dps[3];
	double              intvl;

	length = sizeof(testinTS) / sizeof(double);

	Quatpre.q0 = testQinit[0];
	Quatpre.q1 = testQinit[1];
	Quatpre.q2 = testQinit[2];
	Quatpre.q3 = testQinit[3];

	intvl = testinTS[1] - testinTS[0];

	for (size_t i = 0; i < length; i++)
	{
		ang_rate_dps[0] = testinW[i][0] / DEG2RAD;
		ang_rate_dps[1] = testinW[i][1] / DEG2RAD;
		ang_rate_dps[2] = testinW[i][2] / DEG2RAD;

		QuatIntegrate(&Quatpre, ang_rate_dps, intvl, &QuatIntOut);

		Quatpre.q0 = QuatIntOut.q0;
		Quatpre.q1 = QuatIntOut.q1;
		Quatpre.q2 = QuatIntOut.q2;
		Quatpre.q3 = QuatIntOut.q3;

		result = true;
		if (fabs(QuatIntOut.q0 - testoutQint[i][0]) > 1e-3)	result = false;
		if (fabs(QuatIntOut.q1 - testoutQint[i][1]) > 1e-3)	result = false;
		if (fabs(QuatIntOut.q2 - testoutQint[i][2]) > 1e-3)	result = false;
		if (fabs(QuatIntOut.q3 - testoutQint[i][3]) > 1e-3)	result = false;

		printf("i=%d, Result: %d\r\n", i, result);
		if (result == false) {
			printf("Expected Output: %f, %f, %f, %f \n", testoutQint[i][0], testoutQint[i][1], testoutQint[i][2], testoutQint[i][3]);
			printf("Actual Output: %f, %f, %f, %f \n", QuatIntOut.q0, QuatIntOut.q1, QuatIntOut.q2, QuatIntOut.q3);
		}
	}

}
#endif


#ifdef TESTINTEG1
static void test_QuatIntegrate1st(void)
{

	quaternion_double_t Quatpre, QuatIntOut;
	bool                result;
	size_t              length;
	double              ang_rate_dps[3], pre_ang_rate_dps[3];
	double              intvl;

	length = sizeof(testinTS) / sizeof(double);

	Quatpre.q0 = testQinit[0];
	Quatpre.q1 = testQinit[1];
	Quatpre.q2 = testQinit[2];
	Quatpre.q3 = testQinit[3];

	pre_ang_rate_dps[0] = 0.0;
	pre_ang_rate_dps[1] = 0.0;
	pre_ang_rate_dps[2] = 0.0;

	intvl = testinTS[1] - testinTS[0];

	for (size_t i = 0; i < length - 1; i++)
	{
		ang_rate_dps[0] = testinW[i][0] / DEG2RAD;
		ang_rate_dps[1] = testinW[i][1] / DEG2RAD;
		ang_rate_dps[2] = testinW[i][2] / DEG2RAD;

		QuatIntegrate1st(&Quatpre, pre_ang_rate_dps, ang_rate_dps, intvl, &QuatIntOut);

		Quatpre.q0 = QuatIntOut.q0;
		Quatpre.q1 = QuatIntOut.q1;
		Quatpre.q2 = QuatIntOut.q2;
		Quatpre.q3 = QuatIntOut.q3;

		pre_ang_rate_dps[0] = ang_rate_dps[0];
		pre_ang_rate_dps[1] = ang_rate_dps[1];
		pre_ang_rate_dps[2] = ang_rate_dps[2];

		result = true;
		if (fabs(QuatIntOut.q0 - testoutQint[i][0]) > 1e-3)	result = false;
		if (fabs(QuatIntOut.q1 - testoutQint[i][1]) > 1e-3)	result = false;
		if (fabs(QuatIntOut.q2 - testoutQint[i][2]) > 1e-3)	result = false;
		if (fabs(QuatIntOut.q3 - testoutQint[i][3]) > 1e-3)	result = false;

		printf("indx=%d, Result: %d\r\n", i, result);
		if (result == false) {
			printf("Expected Output: %f, %f, %f, %f \n", testoutQint[i][0], testoutQint[i][1], testoutQint[i][2], testoutQint[i][3]);
			printf("Actual Output: %f, %f, %f, %f \n", QuatIntOut.q0, QuatIntOut.q1, QuatIntOut.q2, QuatIntOut.q3);
		}
	}

}
#endif


#ifdef TESTQ2R
static void test_Quat2RotMtx(void)
{
	quaternion_double_t QuatIn;
	bool                result;
	size_t              length;
	double              R[3][3];

	length = sizeof(testinQuat) / (4 * sizeof(double));

	for (size_t i = 0; i < length; i++)
	{
		QuatIn.q0 = testinQuat[i][0];
		QuatIn.q1 = testinQuat[i][1];
		QuatIn.q2 = testinQuat[i][2];
		QuatIn.q3 = testinQuat[i][3];

		Quat2RotMtx(&QuatIn, R);

		result = true;
		if (fabs(R[0][0] - testoutR[i][0]) > 1e-5)	result = false;
		if (fabs(R[1][0] - testoutR[i][1]) > 1e-5)	result = false;
		if (fabs(R[2][0] - testoutR[i][2]) > 1e-5)	result = false;
		if (fabs(R[0][1] - testoutR[i][3]) > 1e-5)	result = false;
		if (fabs(R[1][1] - testoutR[i][4]) > 1e-5)	result = false;
		if (fabs(R[2][1] - testoutR[i][5]) > 1e-5)	result = false;
		if (fabs(R[0][2] - testoutR[i][6]) > 1e-5)	result = false;
		if (fabs(R[1][2] - testoutR[i][7]) > 1e-5)	result = false;
		if (fabs(R[2][2] - testoutR[i][8]) > 1e-5)	result = false;

		printf("i=%d, Result: %d\r\n", i, result);
		if (result == false) {
			printf("Expected Output: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", testoutR[i][0], testoutR[i][1], testoutR[i][2], testoutR[i][3], testoutR[i][4], testoutR[i][5], testoutR[i][6], testoutR[i][7], testoutR[i][8]);
			printf("Actual Output: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1], R[0][2], R[1][2], R[2][2]);
		}
	}

}
#endif


#ifdef TESTR2Q
static void test_RotMtx2Quat(void)
{
	quaternion_double_t QuatOut;
	bool                result;
	size_t              length;
	double              R[3][3];

	length = sizeof(testinR) / (9 * sizeof(double));

	for (size_t i = 0; i < length; i++)
	{
		R[0][0] = testinR[i][0];
		R[1][0] = testinR[i][1];
		R[2][0] = testinR[i][2];
		R[0][1] = testinR[i][3];
		R[1][1] = testinR[i][4];
		R[2][1] = testinR[i][5];
		R[0][2] = testinR[i][6];
		R[1][2] = testinR[i][7];
		R[2][2] = testinR[i][8];

		RotMtx2Quat(R, &QuatOut);

		result = true;
		if (fabs(QuatOut.q0 - testoutQ[i][0]) > 1e-5)	result = false;
		if (fabs(QuatOut.q1 - testoutQ[i][1]) > 1e-5)	result = false;
		if (fabs(QuatOut.q2 - testoutQ[i][2]) > 1e-5)	result = false;
		if (fabs(QuatOut.q3 - testoutQ[i][3]) > 1e-5)	result = false;

		printf("i=%d, Result: %d\r\n", i, result);
		if (result == false) {
			printf("Expected Output: %f, %f, %f, %f \n", testoutQ[i][0], testoutQ[i][1], testoutQ[i][2], testoutQ[i][3]);
			printf("Actual Output: %f, %f, %f, %f \n", QuatOut.q0, QuatOut.q1, QuatOut.q2, QuatOut.q3);
		}
	}

}
#endif


#ifdef TESTR2A
static void test_RotMtx2Angles(void)
{
	quaternion_double_t QuatOut;
	bool                result;
	size_t              length;
	double              R[3][3];
	double              theta;
	double              phi;
	double              psi;

	length = sizeof(testinRotMtx) / (9 * sizeof(double));

	for (size_t i = 0; i < length; i++)
	{
		R[0][0] = testinRotMtx[i][0];
		R[1][0] = testinRotMtx[i][1];
		R[2][0] = testinRotMtx[i][2];
		R[0][1] = testinRotMtx[i][3];
		R[1][1] = testinRotMtx[i][4];
		R[2][1] = testinRotMtx[i][5];
		R[0][2] = testinRotMtx[i][6];
		R[1][2] = testinRotMtx[i][7];
		R[2][2] = testinRotMtx[i][8];

		RotMtx2Angles(R, &theta, &phi, &psi);

		result = true;
		if (fabs(phi - testoutAngles[i][0]*RAD2DEG) > 1e-5)	result = false;
		if (fabs(theta - testoutAngles[i][1]*RAD2DEG) > 1e-5)	result = false;
		if (fabs(psi - testoutAngles[i][2]*RAD2DEG) > 1e-5)	result = false;

		printf("i=%d, Result: %d\r\n", i, result);
		if (result == false) {
			printf("Expected Output: %f, %f, %f \n", testoutAngles[i][0]* RAD2DEG, testoutAngles[i][1]* RAD2DEG, testoutAngles[i][2]* RAD2DEG);
			printf("Actual Output: %f, %f, %f \n", phi, theta, psi);
		}
	}

}
#endif


#ifdef TESTMATRIX
static void test_MatrixMath(void)
{
	uint32_t valid;

	const double A1[3][3] = {
	    { 1.075334279092200,  2.696058334963207, -2.692438883309332},
	    { 2.696058334963207,  0.637530479717962, -0.965063829766623},
	    {-2.692438883309332, -0.965063829766623,  7.156793879451521},
    };
	double A1inv[3][3] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	const double Ainv_exp[3][3] = {
	    {-0.093767835184712, 0.431142473469379, 0.022861611487251},
	    { 0.431142473469379, -0.011535136193225,   0.160643527879793 },
	    { 0.022861611487251,   0.160643527879793,   0.169990189802632 },
	};

	double B[3][3];

	const double eye3[3][3] = { { 1.0, 0.0, 0.0 },{ 0.0, 1.0, 0.0 },{ 0.0, 0.0, 1.0 } };
	double inveye[3][3];

	const double C1[3][3] = {
		{27.694370298848771,   7.254042249461056, -2.049660582997746},
		{-13.498869401565212, -0.630548731896562, -1.241443482163119},
		{30.349234663318544,   7.147429038260959,  14.896976077854649},
	};

	double D[3][3] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0 };

	double K_factor1 = 22.005;
	double K_factor2 = 100.0;

	blk_mtx_3x3_t Ablk, Ainvblk, Bblk, Cblk, Dblk, Iblk, Zblk;

	for( i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++)
		{
			Ablk.elem[i][j] = A1[i][j];
			Ainvblk.elem[i][j] = A1inv[i][j];
			Bblk.elem[i][j] = B[i][j];
			Cblk.elem[i][j] = C1[i][j];
			Dblk.elem[i][j] = D[i][j];
			Iblk.elem[i][j] = -91919199191.9191919;
			Zblk.elem[i][j] = 989239089.0;
		}
	}
	

	valid = SymMatInv3x3(A1, A1inv);
	printf("Result: %d\r\n", valid);
	printf("Expected Output: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Ainv_exp[0][0], Ainv_exp[0][1], Ainv_exp[0][2], Ainv_exp[1][0], Ainv_exp[1][1], Ainv_exp[1][2], Ainv_exp[2][0], Ainv_exp[2][1], Ainv_exp[2][2]);
	printf("Actual Output: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", A1inv[0][0], A1inv[0][1], A1inv[0][2], A1inv[1][0], A1inv[1][1], A1inv[1][2], A1inv[2][0], A1inv[2][1], A1inv[2][2]);
	
	MatProd3x3(A1, A1inv, B);
	printf("Actual Output: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", B[0][0], B[0][1], B[0][2], B[1][0], B[1][1], B[1][2], B[2][0], B[2][1], B[2][2]);

	MatProd3x3(A1, C1, B);
	printf("Actual Output: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", B[0][0], B[0][1], B[0][2], B[1][0], B[1][1], B[1][2], B[2][0], B[2][1], B[2][2]);

	valid = SymMatInv3x3(eye3, inveye);
	printf("Result: %d\r\n", valid);
	printf("Actual Output: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", inveye[0][0], inveye[0][1], inveye[0][2], inveye[1][0], inveye[1][1], inveye[1][2], inveye[2][0], inveye[2][1], inveye[2][2]);

	MatProd3x3(A1, eye3, B);
	printf("Actual Output: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", B[0][0], B[0][1], B[0][2], B[1][0], B[1][1], B[1][2], B[2][0], B[2][1], B[2][2]);

	// kA = k*A
	ScaleBlkMtx_3x3(K_factor1, &Dblk, &Bblk);
	printf("Actual Output k1*D: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	ScaleBlkMtx_3x3(K_factor2, &Dblk, &Bblk);
	printf("Actual Output k2*D: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	// I = eye(3)
	IdentityBlkMtx_3x3(&Iblk);
	printf("Actual Output I: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Iblk.elem[0][0], Iblk.elem[0][1], Iblk.elem[0][2], Iblk.elem[1][0], Iblk.elem[1][1], Iblk.elem[1][2], Iblk.elem[2][0], Iblk.elem[2][1], Iblk.elem[2][2]);

	//Z = zeros(3,3)
	ZeroBlkMtx_3x3(&Zblk);
	printf("Actual Output 0: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Zblk.elem[0][0], Zblk.elem[0][1], Zblk.elem[0][2], Zblk.elem[1][0], Zblk.elem[1][1], Zblk.elem[1][2], Zblk.elem[2][0], Zblk.elem[2][1], Zblk.elem[2][2]);

	// C = A*B
	MultBlkMtx_3x3(&Ablk, &Cblk, &Bblk);
	printf("Actual Output A1*C1: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	MultBlkMtx_3x3(&Ablk, &Iblk, &Bblk);
	printf("Actual Output A1*I: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	MultBlkMtx_3x3(&Ablk, &Zblk, &Bblk);
	printf("Actual Output A1*0: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	MultBlkMtx_3x3(&Ablk, &Dblk, &Bblk);
	printf("Actual Output A1*D: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	// C = A + B
	AddBlkMtx_3x3(&Ablk, &Cblk, &Bblk);
	printf("Actual Output A1 + C: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	AddBlkMtx_3x3(&Ablk, &Iblk, &Bblk);
	printf("Actual Output A1 + I: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	AddBlkMtx_3x3(&Ablk, &Zblk, &Bblk);
	printf("Actual Output A1 + 0: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	AddBlkMtx_3x3(&Ablk, &Dblk, &Bblk);
	printf("Actual Output A1 + D: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	// B = A
	SetBlkMtx_3x3(&Ablk, &Bblk);
	printf("Actual Output B=A1: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	SetBlkMtx_3x3(&Cblk, &Bblk);
	printf("Actual Output B=C1: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	// Ainv = inverse(A), square blk matrix 1 block of 3x3
	valid = InvBlkSymMtx1_3x3(&Ablk, &Ainvblk);
	printf("Result: %d\r\n", valid);
	printf("Actual Output A1 inv: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Ainvblk.elem[0][0], Ainvblk.elem[0][1], Ainvblk.elem[0][2], Ainvblk.elem[1][0], Ainvblk.elem[1][1], Ainvblk.elem[1][2], Ainvblk.elem[2][0], Ainvblk.elem[2][1], Ainvblk.elem[2][2]);

	valid = InvBlkSymMtx1_3x3(&Iblk, &Ainvblk);
	printf("Result: %d\r\n", valid);
	printf("Actual Output I inverse: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Ainvblk.elem[0][0], Ainvblk.elem[0][1], Ainvblk.elem[0][2], Ainvblk.elem[1][0], Ainvblk.elem[1][1], Ainvblk.elem[1][2], Ainvblk.elem[2][0], Ainvblk.elem[2][1], Ainvblk.elem[2][2]);

	valid = InvBlkSymMtx1_3x3(&Zblk, &Ainvblk);
	printf("Result: %d\r\n", valid);
	printf("Actual Output 0 inv: %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Ainvblk.elem[0][0], Ainvblk.elem[0][1], Ainvblk.elem[0][2], Ainvblk.elem[1][0], Ainvblk.elem[1][1], Ainvblk.elem[1][2], Ainvblk.elem[2][0], Ainvblk.elem[2][1], Ainvblk.elem[2][2]);

	// B = A'
	TranspBlkMtx_3x3(&Ablk, &Bblk);
	printf("Actual Output A1': %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

	TranspBlkMtx_3x3(&Cblk, &Bblk);
	printf("Actual Output C1': %f, %f, %f, %f,%f, %f, %f, %f,%f \n", Bblk.elem[0][0], Bblk.elem[0][1], Bblk.elem[0][2], Bblk.elem[1][0], Bblk.elem[1][1], Bblk.elem[1][2], Bblk.elem[2][0], Bblk.elem[2][1], Bblk.elem[2][2]);

}
#endif


void test_utility(void)
{
#ifdef TESTNORM
	printf("\nTest QuatNormal \n");
	test_QuatNormal();
#endif
#ifdef TESTPROD
	printf("\n\nTest QuatProduct PQ \n");
	test_QuatProduct();
#endif
#ifdef TESTINTEG
	printf("\n\nTest QuatIntegrate \n");
	test_QuatIntegrate();
#endif
#ifdef TESTINTEG1
	printf("\n\nTest QuatIntegrate 1st order\n");
	test_QuatIntegrate1st();
#endif
#ifdef TESTQ2R
	printf("\n\nTest Quat2RotMtx \n");
	test_Quat2RotMtx();
#endif
#ifdef TESTR2Q
	printf("\n\nTest RotMtx2Quat \n");
	test_RotMtx2Quat();
#endif
#ifdef TESTR2A
	printf("\n\nTest RotMtx2Angles \n");
	test_RotMtx2Angles();
#endif
#ifdef TESTMATRIX
	printf("\n\nTest Matrix Math \n");
	test_MatrixMath();
#endif
#ifdef TESTTRIG
	printf("\n\nTest Trig Math \n");
	test_TrigApproxFunctions();
#endif
}
