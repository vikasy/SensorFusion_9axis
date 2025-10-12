/*******************************************************************************
 * Test to verify the tilt formula calculation
 *
 * This test directly calls the tilt calculation to understand how it interprets
 * accelerometer inputs
 ******************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CHX 0
#define CHY 1
#define CHZ 2
#define EPSILON 0.001

// Copy of the algorithm's tilt calculation
static void sf_6xag_algo_tilt_rotmtx(const double accel_avg[3], double RotMtx[3][3])
{
	double mag_grav;
	double mag_grav_yz;
	double mag_grav_yz_sq;
	double V1[3] = { 1, 0, 0 };
	double V2[3] = { 0, 1, 0 };
	double V3[3] = { 0, 0, 1 };
	double alpha = 1.0;
	uint32_t  k;

	mag_grav_yz_sq = (accel_avg[1] * accel_avg[1]) + (accel_avg[2] * accel_avg[2]);
	mag_grav_yz = sqrt(mag_grav_yz_sq);
	mag_grav = sqrt((accel_avg[0] * accel_avg[0]) + mag_grav_yz_sq);

	if ((mag_grav > 0.0) && (mag_grav_yz > 0.0)) {
		alpha = mag_grav / mag_grav_yz;
		for(k = CHX; k <= CHZ; k++) {
			V3[k] = accel_avg[k] / mag_grav;
		}
	}

	V1[0] = 1.0 / alpha;
	V1[1] = -alpha * V3[0] * V3[1];
	V1[2] = -alpha * V3[0] * V3[2];
	V2[0] = 0.0;
	V2[1] = alpha * V3[2];
	V2[2] = -alpha * V3[1];
	for(k = CHX; k <= CHZ; k++) {
		RotMtx[k][0] = V1[k];
		RotMtx[k][1] = V2[k];
		RotMtx[k][2] = V3[k];
	}
}

// Copy of angle extraction
static void extract_angles(const double RotMtx[3][3], double *roll, double *pitch)
{
	*roll = asin(RotMtx[0][2]) * 180.0 / M_PI;

	if ((RotMtx[0][2] < (1 - EPSILON)) && (RotMtx[0][2] > -(1 - EPSILON))) {
		*pitch = atan2(-RotMtx[1][2], RotMtx[2][2]) * 180.0 / M_PI;
	} else {
		*pitch = 0.0;
	}
}

void test_tilt(const char* desc, double ax, double ay, double az)
{
	double accel[3] = {ax, ay, az};
	double RotMtx[3][3] = {0};
	double roll, pitch;

	sf_6xag_algo_tilt_rotmtx(accel, RotMtx);
	extract_angles(RotMtx, &roll, &pitch);

	double mag = sqrt(ax*ax + ay*ay + az*az);

	printf("\n%s\n", desc);
	printf("  Input (m/s²): [%.3f, %.3f, %.3f], |a|=%.3f\n", ax, ay, az, mag);
	printf("  RotMtx[0][2] = %.4f, RotMtx[1][2] = %.4f, RotMtx[2][2] = %.4f\n",
	       RotMtx[0][2], RotMtx[1][2], RotMtx[2][2]);
	printf("  Roll = %.2f°, Pitch = %.2f°\n", roll, pitch);
	printf("  Verification: asin(%.4f) = %.2f°\n", RotMtx[0][2], asin(RotMtx[0][2])*180/M_PI);
}

int main(void)
{
	printf("================================================================================\n");
	printf("  Tilt Formula Verification\n");
	printf("================================================================================\n");

	double g = 9.80665;

	// Test 1: Level
	test_tilt("Test 1: Level", 0, 0, g);

	// Test 2: 30° roll (ax = 0.5g, az = 0.866g)
	test_tilt("Test 2: 30° roll input", 0.5*g, 0, 0.866*g);

	// Test 3: Pure X (1g)
	test_tilt("Test 3: Pure X=1g", g, 0, 0);

	// Test 4: 45° roll (ax = 0.707g, az = 0.707g)
	test_tilt("Test 4: 45° roll input", 0.707*g, 0, 0.707*g);

	// Test 5: What gives 19.76° roll?
	double target_angle = 19.76 * M_PI / 180.0;
	double ax_target = sin(target_angle) * g;
	double az_target = cos(target_angle) * g;
	test_tilt("Test 5: What angle does 19.76° input give?", ax_target, 0, az_target);

	printf("\n================================================================================\n");

	return 0;
}
