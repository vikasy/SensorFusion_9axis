
/*****
* Author: Vikas Yadav
* Date: 2020
* 6-Axis Sensor Fusion (Accelerometer + Gyroscope)
*/

#include "algo_sf_fusion.h"
#include "algo_sf_6x_sensor_fusion.h"

// Global debug counter
static int global_sample_count = 0;

static void sf_6xag_algo_reset(state_vec_6XAG_t *ptr_state_vec_6XAG);
static void sf_6xag_algo_init_orient(state_vec_6XAG_t *ptr_state_vec_6XAG,
	                                 phys_sensor_t    *ptr_accel_data);
static void sf_6xag_algo_tilt_rotmtx(const double accel_avg[3],
	                                 double       RotMtx[3][3]);

void sf_6xag_algo_nom_timeupdate(state_vec_6XAG_t *ptr_state_vec_6XAG);
void sf_6xag_algo_measupdate(state_vec_6XAG_t *ptr_state_vec_6XAG);
void sf_6xag_algo_rotmtx2angles(const double RotMtx[3][3],
									   double       *theta,
									   double       *phi,
									   double       *psi,
									   double       *rho,
									   double       *chi,
									   double       prev_theta,
									   double       prev_psi);




static void sf_6xag_algo_reset(state_vec_6XAG_t *ptr_state_vec_6XAG)
{
    // initialize rotation matrix and quaternion to 1
	// another real data based initialization of rot mtx and quat happens during the first run of the algo
	ptr_state_vec_6XAG->QuatPost.q0 = 1.0f;
	ptr_state_vec_6XAG->RotMtxPost[CHX][CHX] = 1.0f;
	ptr_state_vec_6XAG->RotMtxPost[CHY][CHY] = 1.0f;
	ptr_state_vec_6XAG->RotMtxPost[CHZ][CHZ] = 1.0f;

	// initialize P0, error cov matrix	
	for(int32_t i = 0; i < 3; i++) {
		for(int32_t j = 0; j < 3; j++) {
			ZeroBlkMtx_3x3(&(ptr_state_vec_6XAG->ErrCovMtxPost[i][j]));
		}
	}

	ZeroBlkMtx_3x3(&(ptr_state_vec_6XAG->ProcNoiseVar[0][2]));
	ZeroBlkMtx_3x3(&(ptr_state_vec_6XAG->ProcNoiseVar[1][2]));
	ZeroBlkMtx_3x3(&(ptr_state_vec_6XAG->ProcNoiseVar[2][0]));
	ZeroBlkMtx_3x3(&(ptr_state_vec_6XAG->ProcNoiseVar[2][1]));

	ptr_state_vec_6XAG->update_ErrCovMtx = 1;

	// Initialize algorithm output-related state variables with reasonable defaults
	// Initialize orientation angles to zero (no rotation)
	ptr_state_vec_6XAG->PhiPost = 0.0;     // roll angle
	ptr_state_vec_6XAG->ThetaPost = 0.0;   // pitch angle  
	ptr_state_vec_6XAG->PsiPost = 0.0;     // yaw angle
	ptr_state_vec_6XAG->RhoPost = 0.0;     // compass heading
	ptr_state_vec_6XAG->ChiPost = 0.0;     // tilt angle

	// Initialize gravity vector (pointing down in sensor frame)
	ptr_state_vec_6XAG->GravPostS[0] = 0.0;
	ptr_state_vec_6XAG->GravPostS[1] = 0.0; 
	ptr_state_vec_6XAG->GravPostS[2] = -9.80665; // standard gravity in m/s²

	// Initialize linear acceleration to zero
	for(int32_t i = 0; i < 4; i++) {
		ptr_state_vec_6XAG->AccPostG[i] = 0.0;
	}
	for(int32_t i = 0; i < 3; i++) {
		ptr_state_vec_6XAG->AccPostS[i] = 0.0;
	}

	// Initialize gyro bias to zero
	for(int32_t i = 0; i < 3; i++) {
		ptr_state_vec_6XAG->BiasPostS[i] = 0.0;
		ptr_state_vec_6XAG->BiasErrPostS[i] = 0.0;
		ptr_state_vec_6XAG->OrntErrPostS[i] = 0.0;
		ptr_state_vec_6XAG->AccErrPostS[i] = 0.0;
		ptr_state_vec_6XAG->Omega[i] = 0.0;
		ptr_state_vec_6XAG->AngRatePrev[i] = 0.0;
	}

	// Initialize operation mode flags
	ptr_state_vec_6XAG->OpMode = 0;
	ptr_state_vec_6XAG->SensFlags = 0;
	ptr_state_vec_6XAG->NomupdtTS = 0;
	ptr_state_vec_6XAG->MeasupdtTS = 0;
	ptr_state_vec_6XAG->OrientInit = false;

	// clear the reset flag
	ptr_state_vec_6XAG->Reset = false;

} /* sf_6xag_algo_reset */

  /**
  * @brief: This function is an interface between SF algorithm and algorithm manager.
  * It is used by algoritm manager to create 6axis (accel+gyro) SF algorithm, which outputs
  * gravity, lin acceleration, game rotation vector, and orientaton angle among other things.
  * This function internally allocates the memory required for algo state data.
  *
  * @param[in]: algo_init_data: pointer to algo init data which include physical sensor
  *                  information required for filter algo
  *
  * @param[out]: sf_algo_id: A unique id for this SF algo, to be used by algorithm manager
  *                   for all other API based communication with SF algo, e.g. to send data
  *                   to this algo, or to run this algo to get algo output, or to stop this algo.
  *                   Returns 0 if invalid input or if no memory to allocate
  *
  */
uintptr_t sf_6xag_algo_init(sf_algo_init_data_t *algo_init_data)
{
	state_vec_6XAG_t *ptr_state_vec_6XAG;

	// check for valid input
	if (algo_init_data == NULL) {
		printf("init failed invalid input \n");
		return 0;
	}

	// allocate memory for algo state data
	ptr_state_vec_6XAG = calloc(1, sizeof(state_vec_6XAG_t));
	if (ptr_state_vec_6XAG == NULL) {
		printf("init failed no memory \n");
		return 0;
	}
	// set algo id to the start address of state vec memory
	ptr_state_vec_6XAG->AlgoID = (uintptr_t)ptr_state_vec_6XAG;

	// specify the type of algorithm (6axis Accel + Gyro)
	ptr_state_vec_6XAG->AlgoType = SF_6AG;

	// add sensor specifications for Accel (0) and Gyro (1)
	ptr_state_vec_6XAG->AccData.ScaleFactor = algo_init_data->Acc_GPERCOUNT;
	ptr_state_vec_6XAG->AccData.SensorID = ACC;
	ptr_state_vec_6XAG->GyroData.ScaleFactor = algo_init_data->Gyro_DPSPERCOUNT;
	ptr_state_vec_6XAG->GyroData.SensorID = GYRO;

	// initialize parameters for model noise matrix Q 
	ptr_state_vec_6XAG->ProcNoiseVarOrient = SF_6XAG_QOrient;
	ptr_state_vec_6XAG->ProcNoiseVarBias = SF_6XAG_QBias;
	ptr_state_vec_6XAG->ProcNoiseVarBiasOrient = SF_6XAG_QBiasOrient;
	ptr_state_vec_6XAG->ProcNoiseVarLinAcc = SF_6XAG_QLinAcc;

    // initialize parameter for measurement noise matrix R
	ptr_state_vec_6XAG->MeasNoiseVarAcc = SF_6XAG_QVACC + SF_6XAG_QWACC + ((SF_6XAG_QVGYRO + SF_6XAG_QWGYRO) * SF_DELTA_T_SQ);

	// set time constant for linear acceleration estimation model
	ptr_state_vec_6XAG->LinAccTC = 0.5;

	// clear the reset flag
	sf_6xag_algo_reset(ptr_state_vec_6XAG);

	printf("\n6-axis SF algo initialized \n");
	printf("start 6axis SF algo with algo id = %zu (0x%zx)\n", ptr_state_vec_6XAG->AlgoID, ptr_state_vec_6XAG->AlgoID);
	printf("ptr_state_vec_6XAG address = %p\n", ptr_state_vec_6XAG);
	printf("sizeof(ptr_state_vec_6XAG) = %zu bytes\n", sizeof(ptr_state_vec_6XAG));
	printf("sizeof(*ptr_state_vec_6XAG) = %zu bytes\n", sizeof(*ptr_state_vec_6XAG));

	return ptr_state_vec_6XAG->AlgoID;

} /* sf_6xag_algo_init */


static void sf_6xag_algo_init_orient(state_vec_6XAG_t *ptr_state_vec_6XAG,
	                                 phys_sensor_t    *ptr_accel_data)
{
	double      accel_avg[3];
	uint32_t    i;

	// Check if CountAvg has been populated (magnitude check)
	double mag_check = 0.0;
	for(i = CHX; i <= CHZ; i++) {
		double val = ptr_accel_data->CountAvg[i] * ptr_accel_data->ScaleFactor * GTOMSEC2;
		mag_check += val * val;
	}

	if (mag_check < 1e-6) {
		// CountAvg not yet populated - use most recent raw count from buffer
		for(i = CHX; i <= CHZ; i++) {
			// Use most recent sample from buffer (index 0)
			accel_avg[i] = (ptr_accel_data->CountBuff[0][i])*(ptr_accel_data->ScaleFactor)*GTOMSEC2;
		}
	} else {
		// Use averaged counts
		for(i = CHX; i <= CHZ; i++) {
			accel_avg[i] = (ptr_accel_data->CountAvg[i])*(ptr_accel_data->ScaleFactor)*GTOMSEC2;
		}
	}

	// initialize the a posteriori orientation state vector to the tilt orientation
	sf_6xag_algo_tilt_rotmtx(accel_avg, ptr_state_vec_6XAG->RotMtxPost);

	RotMtx2Quat(ptr_state_vec_6XAG->RotMtxPost, &(ptr_state_vec_6XAG->QuatPost));

	// clear the init flag
	ptr_state_vec_6XAG->OrientInit = true;

} /* sf_6xag_algo_init_orient */




static void sf_6xag_algo_run_orig(state_vec_6XAG_t *ptr_state_vec_6XAG,
	                              sf_algo_output_t *ptr_algo_out)
{
	int64_t     curr_time_msec;
	uint32_t    i;

	// do a reset and return if requested
	if (ptr_state_vec_6XAG->Reset) {
		printf("6-axis SF algo reset\n");
		sf_6xag_algo_reset(ptr_state_vec_6XAG);
		return;
	}

	// do a once-only orientation lock to accelerometer tilt
	if (!ptr_state_vec_6XAG->OrientInit) {
		printf("6-axis SF algo initial orientation lock\n");
		sf_6xag_algo_init_orient(ptr_state_vec_6XAG, &(ptr_state_vec_6XAG->AccData));
		printf("Quat after init_orient: [%.15f, %.15f, %.15f, %.15f]\n",
			ptr_state_vec_6XAG->QuatPost.q0, ptr_state_vec_6XAG->QuatPost.q1,
			ptr_state_vec_6XAG->QuatPost.q2, ptr_state_vec_6XAG->QuatPost.q3);
	}

	global_sample_count++;
	printf("running SF...");
	curr_time_msec = (int64_t)clock();

	// if new gyro data is available, apply nominal time udpate
	if ( ptr_state_vec_6XAG->NomupdtTS < ptr_state_vec_6XAG->GyroData.timestamp ) {
		sf_6xag_algo_nom_timeupdate(ptr_state_vec_6XAG);
	}
	// if new accel data is availabel apply measurement update
	if ( ptr_state_vec_6XAG->MeasupdtTS < ptr_state_vec_6XAG->AccData.timestamp ) {
		sf_6xag_algo_measupdate(ptr_state_vec_6XAG);
	}

	for(i = 0; i < 3; i++) {
		ptr_state_vec_6XAG->GravPostS[i] = -1.0 * GTOMSEC2 * ptr_state_vec_6XAG->RotMtxPost[i][2];
	}

	// Convert quaternions to angles
	sf_6xag_algo_rotmtx2angles(ptr_state_vec_6XAG->RotMtxPost, &(ptr_state_vec_6XAG->ThetaPost), 
		                        &(ptr_state_vec_6XAG->PhiPost), &(ptr_state_vec_6XAG->PsiPost), 
		                        &(ptr_state_vec_6XAG->RhoPost), &(ptr_state_vec_6XAG->ChiPost),
		                        ptr_state_vec_6XAG->ThetaPost, ptr_state_vec_6XAG->PsiPost);

	// Assign outputs
	ptr_algo_out->algo_type = SF_6AG;
	ptr_algo_out->quat.q0 = (float)ptr_state_vec_6XAG->QuatPost.q0;
	ptr_algo_out->quat.q1 = (float)ptr_state_vec_6XAG->QuatPost.q1;
	ptr_algo_out->quat.q2 = (float)ptr_state_vec_6XAG->QuatPost.q2;
	ptr_algo_out->quat.q3 = (float)ptr_state_vec_6XAG->QuatPost.q3;
	ptr_algo_out->orientation[0] = (float)ptr_state_vec_6XAG->PsiPost;
	ptr_algo_out->orientation[1] = (float)ptr_state_vec_6XAG->ThetaPost;
	ptr_algo_out->orientation[2] = (float)ptr_state_vec_6XAG->PhiPost;
	ptr_algo_out->gravity[0] = (float)ptr_state_vec_6XAG->GravPostS[0];
	ptr_algo_out->gravity[1] = (float)ptr_state_vec_6XAG->GravPostS[1];
	ptr_algo_out->gravity[2] = (float)ptr_state_vec_6XAG->GravPostS[2];
	ptr_algo_out->linear_acc[0] = (float)ptr_state_vec_6XAG->AccPostG[0];
	ptr_algo_out->linear_acc[1] = (float)ptr_state_vec_6XAG->AccPostG[1];
	ptr_algo_out->linear_acc[2] = (float)ptr_state_vec_6XAG->AccPostG[2];
	ptr_algo_out->valid_flag = 0xFFFFFFFF;
	ptr_algo_out->mode = ptr_state_vec_6XAG->OpMode;
	ptr_algo_out->timestamp_ns = curr_time_msec / NSEC2MSEC;

	// Print calculated quaternion and bias for first 22 runs
	if (global_sample_count <= 22) {
		printf("C Run[%d]: Quat=[%.15f, %.15f, %.15f, %.15f], BiasPostS=[%.12f, %.12f, %.12f]\n",
			global_sample_count,
			ptr_algo_out->quat.q0, ptr_algo_out->quat.q1,
			ptr_algo_out->quat.q2, ptr_algo_out->quat.q3,
			ptr_state_vec_6XAG->BiasPostS[0], ptr_state_vec_6XAG->BiasPostS[1], ptr_state_vec_6XAG->BiasPostS[2]);
	}

} /* sf_6xag_algo_run_orig */


/*  Calculate orientation matrix based on accelerometer sensor data 
*/
static void sf_6xag_algo_tilt_rotmtx(const double accel_avg[3],
	                                 double       RotMtx[3][3])
{
	// Fixed tilt initialization using Gram-Schmidt orthogonalization (matches Python)
	double mag_grav;
	double V1[3];
	double V2[3];
	double V3[3];
	double V1_norm;
	double dot_product;
	uint32_t k;

	// Normalize gravity vector (V3 = third column)
	mag_grav = sqrt(accel_avg[0] * accel_avg[0] +
	                accel_avg[1] * accel_avg[1] +
	                accel_avg[2] * accel_avg[2]);

	if (mag_grav < 1e-6) {
		// No gravity - return identity
		for(k = 0; k < 3; k++) {
			for(uint32_t j = 0; j < 3; j++) {
				RotMtx[k][j] = (k == j) ? 1.0 : 0.0;
			}
		}
		return;
	}

	// V3 = normalized gravity
	for(k = 0; k < 3; k++) {
		V3[k] = accel_avg[k] / mag_grav;
	}

	// Choose reference vector for V1
	if (fabs(V3[0]) < 0.9) {
		V1[0] = 1.0;
		V1[1] = 0.0;
		V1[2] = 0.0;
	} else {
		V1[0] = 0.0;
		V1[1] = 1.0;
		V1[2] = 0.0;
	}

	// Gram-Schmidt: V1 = V1 - (V1·V3)V3
	dot_product = V1[0]*V3[0] + V1[1]*V3[1] + V1[2]*V3[2];
	for(k = 0; k < 3; k++) {
		V1[k] = V1[k] - dot_product * V3[k];
	}

	// Normalize V1
	V1_norm = sqrt(V1[0]*V1[0] + V1[1]*V1[1] + V1[2]*V1[2]);
	if (V1_norm > 1e-6) {
		for(k = 0; k < 3; k++) {
			V1[k] = V1[k] / V1_norm;
		}
	} else {
		V1[0] = 1.0;
		V1[1] = 0.0;
		V1[2] = 0.0;
	}

	// V2 = V3 × V1 (cross product)
	V2[0] = V3[1]*V1[2] - V3[2]*V1[1];
	V2[1] = V3[2]*V1[0] - V3[0]*V1[2];
	V2[2] = V3[0]*V1[1] - V3[1]*V1[0];

	// Rotation matrix: R = [V1 V2 V3]
	for(k = 0; k < 3; k++) {
		RotMtx[k][0] = V1[k];
		RotMtx[k][1] = V2[k];
		RotMtx[k][2] = V3[k];
	}
}

/*  Update the orientation angles, compass heading, and tilt angles (in Deg) 
*  based on the updated rotation matrix 
*  Input: Rotation Matrix, cordinate frame, prev theta and psi angles 
*  Output: -90 <= phi <= 90  
*         -180 <= theta < 180 
*          0 <= psi, rho, tilt < 360 
*  Use prev values of theta and psi for resolving Gimbal lock condition 
*/
void sf_6xag_algo_rotmtx2angles(const double RotMtx[3][3],
                                       double       *theta,
                                       double       *phi,
                                       double       *psi,
                                       double       *rho,
                                       double       *chi,
	                                   double       prev_theta,
	                                   double       prev_psi)
{
    #define MAX_POS_PITCH_DEG (179.9999)

	static uint32_t prev_theta_used = 0;
	double          angle_val;
	uint32_t        angle_vld;

	/*  pitch angle [-90,90) - AN5017 NED: θ = asin(-R[0][2]) */
	*theta = asin(-RotMtx[0][2]) * RAD2DEG;

	/*  roll angle [-180,180) and yaw angle [0, 360) - AN5017 NED    */
	*phi = prev_theta;  // using prev_theta as temporary storage for roll
	*psi = prev_psi;
	if( (RotMtx[0][2] < (1 - EPSILON)) && (RotMtx[0][2] > -(1 - EPSILON)) ) {
		angle_val = atan2_safe(RotMtx[1][2], RotMtx[2][2], &angle_vld);  // AN5017: φ = atan2(R[1][2], R[2][2])
		if(angle_vld == 1) {
			*phi = angle_val * RAD2DEG;
		}
		angle_val = atan2_safe(RotMtx[0][1], RotMtx[0][0], &angle_vld);  // AN5017: ψ = atan2(R[0][1], R[0][0])
		if(angle_vld == 1) {
			*psi = angle_val * RAD2DEG;
		}
	}
	/*  Gimbal lock at pitch = ±90°, resolve using prev values (AN5017 Section 2.6, Eqs 23-24) */
	else {
		angle_val = atan2_safe(RotMtx[1][0], RotMtx[1][1], &angle_vld);
		if(angle_vld == 1) {
			if (prev_theta_used == 0) {
				if ((RotMtx[0][2] <= -(1 - EPSILON))) {  // pitch = +90°
					*psi = (angle_val * RAD2DEG) - *phi;  // tan(ψ - φ) = R[1][0]/R[1][1]
				}
				else {  // pitch = -90°
					*psi = (angle_val * RAD2DEG) + *phi;  // tan(ψ + φ) = -R[1][0]/R[1][1]
				}
				prev_theta_used = 1;
			}
			else {
				if ((RotMtx[0][2] <= -(1 - EPSILON))) {  // pitch = +90°
					*phi = (angle_val * RAD2DEG) - *psi;  // tan(ψ - φ) = R[1][0]/R[1][1]
				}
				else {  // pitch = -90°
					*phi = (angle_val * RAD2DEG) + *psi;  // tan(ψ + φ) = -R[1][0]/R[1][1]
				}
				prev_theta_used = 0;
			}
		}
	}

	if (*theta > MAX_POS_PITCH_DEG) {
		*theta = -180.0;
	}
	if (*psi < 0.0) {
		*psi += 360.0;
	}
	else if (*psi >= 360.0) {
		*psi = 0.0;
	}

	/* compass heading angle */
	*rho = *psi;

	/* tilt angle chi [0,180] */
	*chi = acos(RotMtx[2][2]) * RAD2DEG;

}


void sf_6xag_algo_nom_timeupdate(state_vec_6XAG_t *ptr_state_vec_6XAG)
{
#ifdef GYRO_BIAS_TIME_AVG
	#define GYRO_BIAS_TIME_AVG_LEN   (60*60*SF_GYRO_FS) // 1 hour
	static uint32_t gyro_bias_time_avg_len = 1;
#endif /* GYRO_BIAS_TIME_AVG */

	quaternion_double_t  QuatInt;
	double               delta_T;
	double               Cacc2;
	double               orient_err;
	blk_mtx_3x3_t        Atemp, Btemp, Ctemp;
	uint32_t             i, k;

	// apply nominal time update for all samples in Gyro buffer
	//delta_T = (double)(ptr_state_vec_6XAG->GyroData.timestamp - ptr_state_vec_6XAG->NomupdtTS);
	delta_T = SF_GYRO_SAMP_INTVL;

	// Debug specific samples around divergence point
	int debug_sample = (global_sample_count == 50);

	if (debug_sample) {
		printf("\n================================================================================\n");
		printf("TIME_UPDATE - Sample %d (C)\n", global_sample_count);
		printf("================================================================================\n");
		printf("Timestamp: %lld\n", ptr_state_vec_6XAG->GyroData.timestamp);
		printf("Quat BEFORE integration: [%.8f, %.8f, %.8f, %.8f]\n",
			ptr_state_vec_6XAG->QuatPost.q0, ptr_state_vec_6XAG->QuatPost.q1,
			ptr_state_vec_6XAG->QuatPost.q2, ptr_state_vec_6XAG->QuatPost.q3);
		printf("BiasErrPostS: [%.8f, %.8f, %.8f]\n",
			ptr_state_vec_6XAG->BiasErrPostS[0], ptr_state_vec_6XAG->BiasErrPostS[1], ptr_state_vec_6XAG->BiasErrPostS[2]);
	}
#if 0	
	printf("a=%d, b=%f,c=%f,d=%f\n\n", ptr_state_vec_6XAG->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][0], ptr_state_vec_6XAG->GyroData.ScaleFactor,
		(ptr_state_vec_6XAG->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][0] * ptr_state_vec_6XAG->GyroData.ScaleFactor), ptr_state_vec_6XAG->BiasPostS[0]);
	printf("a=%d, b=%f,c=%f,d=%f\n\n", ptr_state_vec_6XAG->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][1], ptr_state_vec_6XAG->GyroData.ScaleFactor,
		(ptr_state_vec_6XAG->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][1] * ptr_state_vec_6XAG->GyroData.ScaleFactor), ptr_state_vec_6XAG->BiasPostS[1]);
	printf("a=%d, b=%f,c=%f,d=%f\n\n", ptr_state_vec_6XAG->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][2], ptr_state_vec_6XAG->GyroData.ScaleFactor,
		(ptr_state_vec_6XAG->GyroData.CountBuff[SF_OVERSAMPLE_RATIO - 1][2] * ptr_state_vec_6XAG->GyroData.ScaleFactor), ptr_state_vec_6XAG->BiasPostS[2]);
#endif // 0

	for(k = 0; k < SF_OVERSAMPLE_RATIO; k++) {
		for(i = CHX; i <= CHZ; i++) {
#ifdef GYRO_BIAS_TIME_AVG
			ptr_state_vec_6XAG->BiasErrPostS[i] *= gyro_bias_time_avg_len;
			ptr_state_vec_6XAG->BiasErrPostS[i] += ptr_state_vec_6XAG->GyroData.CountBuff[k][i];
			ptr_state_vec_6XAG->BiasErrPostS[i] /= (gyro_bias_time_avg_len++);
			if(gyro_bias_time_avg_len > GYRO_BIAS_TIME_AVG_LEN)
				gyro_bias_time_avg_len = GYRO_BIAS_TIME_AVG_LEN;
#endif /* GYRO_BIAS_TIME_AVG */
			ptr_state_vec_6XAG->Omega[i] = ptr_state_vec_6XAG->GyroData.CountBuff[k][i] * ptr_state_vec_6XAG->GyroData.ScaleFactor;
			ptr_state_vec_6XAG->Omega[i] -= ptr_state_vec_6XAG->BiasPostS[i];  // Fixed: Use BiasPostS (accumulated bias) not BiasErrPostS (latest correction)
			ptr_state_vec_6XAG->AngRatePrev[i] = ptr_state_vec_6XAG->Omega[i];
		}

		if (debug_sample) {
			printf("  Buffer[%d] gyro_raw: [%d, %d, %d]\n", k,
				ptr_state_vec_6XAG->GyroData.CountBuff[k][CHX],
				ptr_state_vec_6XAG->GyroData.CountBuff[k][CHY],
				ptr_state_vec_6XAG->GyroData.CountBuff[k][CHZ]);
			printf("  Buffer[%d] omega (bias-corrected): [%.8f, %.8f, %.8f] deg/s\n", k,
				ptr_state_vec_6XAG->Omega[CHX], ptr_state_vec_6XAG->Omega[CHY], ptr_state_vec_6XAG->Omega[CHZ]);
			printf("  Quat before integrate[%d]: [%.8f, %.8f, %.8f, %.8f]\n", k,
				ptr_state_vec_6XAG->QuatPost.q0, ptr_state_vec_6XAG->QuatPost.q1,
				ptr_state_vec_6XAG->QuatPost.q2, ptr_state_vec_6XAG->QuatPost.q3);
		}

		QuatIntegrate(&(ptr_state_vec_6XAG->QuatPost), ptr_state_vec_6XAG->Omega, delta_T, &QuatInt);
		ptr_state_vec_6XAG->QuatPost.q0 = QuatInt.q0;
		ptr_state_vec_6XAG->QuatPost.q1 = QuatInt.q1;
		ptr_state_vec_6XAG->QuatPost.q2 = QuatInt.q2;
		ptr_state_vec_6XAG->QuatPost.q3 = QuatInt.q3;

		if (debug_sample) {
			printf("  Quat after integrate[%d]:  [%.8f, %.8f, %.8f, %.8f]\n", k,
				ptr_state_vec_6XAG->QuatPost.q0, ptr_state_vec_6XAG->QuatPost.q1,
				ptr_state_vec_6XAG->QuatPost.q2, ptr_state_vec_6XAG->QuatPost.q3);
		}
		//printf("quat tup =%f, %f, %f, %f \n", ptr_state_vec_6XAG->QuatPost.q0, ptr_state_vec_6XAG->QuatPost.q1, ptr_state_vec_6XAG->QuatPost.q2, ptr_state_vec_6XAG->QuatPost.q3);
	}
	// Normalize quaternion
	QuatNormal(&QuatInt, &(ptr_state_vec_6XAG->QuatPost));

	if (debug_sample) {
		printf("Quat AFTER normalization: [%.8f, %.8f, %.8f, %.8f]\n",
			ptr_state_vec_6XAG->QuatPost.q0, ptr_state_vec_6XAG->QuatPost.q1,
			ptr_state_vec_6XAG->QuatPost.q2, ptr_state_vec_6XAG->QuatPost.q3);
	}

	// Update rotation matrix as well
	Quat2RotMtx( &(ptr_state_vec_6XAG->QuatPost), ptr_state_vec_6XAG->RotMtxPost);
	// Use gyro timestamp to maintain sync with sensor data
	// Before updating, check for large time gap
	if ( (ptr_state_vec_6XAG->GyroData.timestamp - ptr_state_vec_6XAG->NomupdtTS) > (SF_GYRO_MAX_MISS_DUR) ) {
		// large time gap - skip error covariance update
		ptr_state_vec_6XAG->update_ErrCovMtx = 0;
		// if current time is greater than last nom update time by max miss duration, 
	    // mark missing gyro data
		ptr_state_vec_6XAG->OpMode &= ~(SF_GYRO_MASK);
		ptr_state_vec_6XAG->OpMode |= SF_GYRO_MISSING;
	} else {
		ptr_state_vec_6XAG->update_ErrCovMtx = 1;
		// if gyro data used is available, mark mode_op to normal gyro mode
		ptr_state_vec_6XAG->OpMode &= ~(SF_GYRO_MASK);
	}
	ptr_state_vec_6XAG->NomupdtTS = ptr_state_vec_6XAG->GyroData.timestamp;
	//printf("nomupdt_ts =%d\n", ptr_state_vec_6XAG->NomupdtTS);

	if (ptr_state_vec_6XAG->update_ErrCovMtx == 1) {
		// Update posteriosi covariance matrix P_pri = A' *P_post *A + Qw
		// In other words, update Qw based on aposteriori error covariance matrix
		// where Qw = a_fn(P_post) = f(P_post) + Qinit
		Cacc2 = ptr_state_vec_6XAG->LinAccTC * ptr_state_vec_6XAG->LinAccTC;

		IdentityBlkMtx_3x3(&(ptr_state_vec_6XAG->ProcNoiseVar[0][0]));
		ScaleBlkMtx_3x3(ptr_state_vec_6XAG->ProcNoiseVarOrient, &(ptr_state_vec_6XAG->ProcNoiseVar[0][0]), &Atemp);
		ScaleBlkMtx_3x3(delta_T*delta_T, &(ptr_state_vec_6XAG->ErrCovMtxPost[1][1]), &Btemp);
		AddBlkMtx_3x3(&Atemp, &(ptr_state_vec_6XAG->ErrCovMtxPost[0][0]), &Ctemp);
		AddBlkMtx_3x3(&Ctemp, &Btemp, &(ptr_state_vec_6XAG->ProcNoiseVar[0][0]));

		IdentityBlkMtx_3x3(&(ptr_state_vec_6XAG->ProcNoiseVar[1][1]));
		ScaleBlkMtx_3x3(ptr_state_vec_6XAG->ProcNoiseVarBias, &(ptr_state_vec_6XAG->ProcNoiseVar[1][1]), &Atemp);
		AddBlkMtx_3x3(&Atemp, &(ptr_state_vec_6XAG->ErrCovMtxPost[1][1]), &(ptr_state_vec_6XAG->ProcNoiseVar[1][1]));

		IdentityBlkMtx_3x3(&(ptr_state_vec_6XAG->ProcNoiseVar[2][2]));
		ScaleBlkMtx_3x3(ptr_state_vec_6XAG->ProcNoiseVarLinAcc, &(ptr_state_vec_6XAG->ProcNoiseVar[2][2]), &Atemp);
		ScaleBlkMtx_3x3(Cacc2, &(ptr_state_vec_6XAG->ErrCovMtxPost[2][2]), &Btemp);
		AddBlkMtx_3x3(&Atemp, &Btemp, &(ptr_state_vec_6XAG->ProcNoiseVar[2][2]));

		IdentityBlkMtx_3x3(&(ptr_state_vec_6XAG->ProcNoiseVar[0][1]));
		ScaleBlkMtx_3x3(ptr_state_vec_6XAG->ProcNoiseVarBiasOrient, &(ptr_state_vec_6XAG->ProcNoiseVar[0][1]), &Atemp);
		ScaleBlkMtx_3x3(-1 * delta_T, &(ptr_state_vec_6XAG->ErrCovMtxPost[1][1]), &Btemp);
		// BUG FIX: Add Btemp to Atemp (was missing - Btemp was computed but never used!)
		// This matches the original Freescale formula: Qw[0][1] = error_product - alpha*Qwb
		// which corresponds to: I*ProcNoiseVarBiasOrient + (-delta_T)*ErrCovMtxPost[1][1]
		AddBlkMtx_3x3(&Atemp, &Btemp, &Ctemp);
		SetBlkMtx_3x3(&Ctemp, &(ptr_state_vec_6XAG->ProcNoiseVar[0][1]));
		TranspBlkMtx_3x3(&Ctemp, &Btemp);
		SetBlkMtx_3x3(&Btemp, &(ptr_state_vec_6XAG->ProcNoiseVar[1][0]));

		orient_err = ptr_state_vec_6XAG->ProcNoiseVar[0][0].elem[0][0] * ptr_state_vec_6XAG->ProcNoiseVar[0][0].elem[0][0];
		orient_err += ptr_state_vec_6XAG->ProcNoiseVar[0][0].elem[1][1] * ptr_state_vec_6XAG->ProcNoiseVar[0][0].elem[1][1];
		orient_err += ptr_state_vec_6XAG->ProcNoiseVar[0][0].elem[2][2] * ptr_state_vec_6XAG->ProcNoiseVar[0][0].elem[2][2];
		if(orient_err > SF_MAX_ORIENT_ERR) {
			ptr_state_vec_6XAG->update_ErrCovMtx = 0;
		}
	}

}


void sf_6xag_algo_measupdate(state_vec_6XAG_t *ptr_state_vec_6XAG)
{

	blk_mtx_3x3_t        Atemp, Btemp;
	blk_mtx_3x3_t        Ftemp[3], Gtemp, Ginv;
	blk_mtx_3x3_t        Cmat[3];
	blk_mtx_3x3_t        Qv_mat;
	quaternion_double_t  QuatInt;
	double               Mupdt[9];
	double               gyro_corr[3];
	double               orient_err;
	uint32_t             inv_exist;
	uint32_t             i, j, k;

	inv_exist = 1;

	int debug_meas = (global_sample_count == 50);  // Debug sample 50 where divergence is noticeable

	if (debug_meas) {
		printf("\n================================================================================\n");
		printf("MEASUREMENT_UPDATE - Sample %d (C)\n", global_sample_count);
		printf("================================================================================\n");
		printf("Timestamp: %lld\n", ptr_state_vec_6XAG->AccData.timestamp);
		printf("Quat BEFORE update: [%.15f, %.15f, %.15f, %.15f]\n",
			ptr_state_vec_6XAG->QuatPost.q0, ptr_state_vec_6XAG->QuatPost.q1,
			ptr_state_vec_6XAG->QuatPost.q2, ptr_state_vec_6XAG->QuatPost.q3);
		printf("BiasErrPostS BEFORE: [%.15f, %.15f, %.15f]\n",
			ptr_state_vec_6XAG->BiasErrPostS[0], ptr_state_vec_6XAG->BiasErrPostS[1], ptr_state_vec_6XAG->BiasErrPostS[2]);
		printf("BiasPostS BEFORE: [%.15f, %.15f, %.15f]\n",
			ptr_state_vec_6XAG->BiasPostS[0], ptr_state_vec_6XAG->BiasPostS[1], ptr_state_vec_6XAG->BiasPostS[2]);
		printf("\nInput acc counts: [%d, %d, %d]\n",
			ptr_state_vec_6XAG->AccData.CountBuff[SF_OVERSAMPLE_RATIO-1][0],
			ptr_state_vec_6XAG->AccData.CountBuff[SF_OVERSAMPLE_RATIO-1][1],
			ptr_state_vec_6XAG->AccData.CountBuff[SF_OVERSAMPLE_RATIO-1][2]);
		printf("Acc scale: %.15f\n", ptr_state_vec_6XAG->AccData.ScaleFactor);

		printf("\nProc Noise Var (Qw) for bias - ProcNoiseVar[1][*]:\n");
		printf("Qw[1][0]:\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.8e, %.8e, %.8e]\n",
				ptr_state_vec_6XAG->ProcNoiseVar[1][0].elem[i][0],
				ptr_state_vec_6XAG->ProcNoiseVar[1][0].elem[i][1],
				ptr_state_vec_6XAG->ProcNoiseVar[1][0].elem[i][2]);
		}
		printf("Qw[1][1]:\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.8e, %.8e, %.8e]\n",
				ptr_state_vec_6XAG->ProcNoiseVar[1][1].elem[i][0],
				ptr_state_vec_6XAG->ProcNoiseVar[1][1].elem[i][1],
				ptr_state_vec_6XAG->ProcNoiseVar[1][1].elem[i][2]);
		}
		printf("Qw[1][2]:\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.8e, %.8e, %.8e]\n",
				ptr_state_vec_6XAG->ProcNoiseVar[1][2].elem[i][0],
				ptr_state_vec_6XAG->ProcNoiseVar[1][2].elem[i][1],
				ptr_state_vec_6XAG->ProcNoiseVar[1][2].elem[i][2]);
		}
	}

	// Compute error in Gravity Vector
	for(i = CHX; i <= CHZ; i++) {
		ptr_state_vec_6XAG->GravGyrPriS[i] = -ptr_state_vec_6XAG->RotMtxPost[i][2] * GTOMSEC2;
		//printf("g=%f, h=%f\n\n", ptr_state_vec_6XAG->GravGyrPriS[i], ptr_state_vec_6XAG->RotMtxPost[i][2]);
		ptr_state_vec_6XAG->GravErrPriS[i] = ptr_state_vec_6XAG->AccData.CountBuff[SF_OVERSAMPLE_RATIO - 1][i];
		ptr_state_vec_6XAG->GravErrPriS[i] *= -1.0;
		ptr_state_vec_6XAG->GravErrPriS[i] *= ptr_state_vec_6XAG->AccData.ScaleFactor * GTOMSEC2;
		ptr_state_vec_6XAG->GravErrPriS[i] += ptr_state_vec_6XAG->LinAccTC *ptr_state_vec_6XAG->AccPostS[i];
		ptr_state_vec_6XAG->GravErrPriS[i] -= ptr_state_vec_6XAG->GravGyrPriS[i];
	}

	if (debug_meas) {
		printf("\nGravErrPriS: [%.8f, %.8f, %.8f]\n",
			ptr_state_vec_6XAG->GravErrPriS[0], ptr_state_vec_6XAG->GravErrPriS[1], ptr_state_vec_6XAG->GravErrPriS[2]);
		printf("GravGyrPriS: [%.8f, %.8f, %.8f]\n",
			ptr_state_vec_6XAG->GravGyrPriS[0], ptr_state_vec_6XAG->GravGyrPriS[1], ptr_state_vec_6XAG->GravGyrPriS[2]);
	}

	// Compute the measurement matrix, C using a priori gravity values
	CrossPdctMtx_3x3(ptr_state_vec_6XAG->GravGyrPriS, &Atemp);
	ScaleBlkMtx_3x3(-DEG2RAD, &Atemp, &Cmat[0]);
	ScaleBlkMtx_3x3((DEG2RAD * SF_DELTA_T), &Atemp, &Cmat[1]);
	IdentityBlkMtx_3x3(&Cmat[2]);

	if (debug_meas) {
		printf("\nMeasurement matrix C[0]:\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.15e, %.15e, %.15e]\n",
				Cmat[0].elem[i][0], Cmat[0].elem[i][1], Cmat[0].elem[i][2]);
		}
		printf("Measurement matrix C[1]:\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.15e, %.15e, %.15e]\n",
				Cmat[1].elem[i][0], Cmat[1].elem[i][1], Cmat[1].elem[i][2]);
		}
		printf("Measurement matrix C[2]:\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.15e, %.15e, %.15e]\n",
				Cmat[2].elem[i][0], Cmat[2].elem[i][1], Cmat[2].elem[i][2]);
		}
	}

	/* Compute Kalman gain, K = Qw*C'*inv(C*Qw*C' + Qv) 
	 * F[3] = Qw[3][3]*C[3]'
	 * G = C[3]*F[3] + Qv
	 * Ginv = inv(G)
	 * K[3] = F[3]*Ginv
	 */
	
	// F[3] = Qw[3][3]*C[3]'
	for( i = CHX; i <= CHZ; i++) {
		ZeroBlkMtx_3x3(&Ftemp[i]);
		for( j = CHX; j <= CHZ; j++) {
			TranspBlkMtx_3x3(&Cmat[j], &Atemp);
			MultBlkMtx_3x3(&(ptr_state_vec_6XAG->ProcNoiseVar[i][j]), &Atemp, &Btemp);
			AddBlkMtx_3x3(&Ftemp[i], &Btemp, &Atemp);
			SetBlkMtx_3x3(&Atemp, &Ftemp[i]);
		}
	}

	if (debug_meas) {
		printf("\nF[0] (Qw[0][*] @ C[*].T):\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.15e, %.15e, %.15e]\n",
				Ftemp[0].elem[i][0], Ftemp[0].elem[i][1], Ftemp[0].elem[i][2]);
		}
		printf("F[1] (Qw[1][*] @ C[*].T):\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.15e, %.15e, %.15e]\n",
				Ftemp[1].elem[i][0], Ftemp[1].elem[i][1], Ftemp[1].elem[i][2]);
		}
		printf("F[2] (Qw[2][*] @ C[*].T):\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.15e, %.15e, %.15e]\n",
				Ftemp[2].elem[i][0], Ftemp[2].elem[i][1], Ftemp[2].elem[i][2]);
		}
	}
	
	// G = C[3]*F[3] + Qv
	IdentityBlkMtx_3x3(&Qv_mat);
	ScaleBlkMtx_3x3(ptr_state_vec_6XAG->MeasNoiseVarAcc, &Qv_mat, &Gtemp);
	//PrintBlkMtx_3x3(&Gtemp,"Qv");
	for( i = CHX; i <= CHZ; i++) {
		MultBlkMtx_3x3(&Cmat[i], &Ftemp[i], &Atemp);
		//PrintBlkMtx_3x3(&Atemp,"Ci*Fi");
		AddBlkMtx_3x3( &Atemp, &Gtemp, &Btemp );
		SetBlkMtx_3x3( &Btemp, &Gtemp );
		//PrintBlkMtx_3x3(&Gtemp,"Qv+Ci*Fi");
	}
	
	// Ginv = inv(G)
	inv_exist = InvBlkSymMtx1_3x3(&Gtemp, &Ginv);

	// K[3] = F[3]*Ginv
	if (inv_exist == 1) {
		for(  i = CHX; i <= CHZ; i++) {
			MultBlkMtx_3x3(&Ftemp[i], &Ginv, &(ptr_state_vec_6XAG->KalmanGain[i]));
		}
	}

	if (debug_meas) {
		printf("\nKalman Gain K[0]:\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.8f, %.8f, %.8f]\n",
				ptr_state_vec_6XAG->KalmanGain[0].elem[i][0],
				ptr_state_vec_6XAG->KalmanGain[0].elem[i][1],
				ptr_state_vec_6XAG->KalmanGain[0].elem[i][2]);
		}
		printf("Kalman Gain K[1]:\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.8f, %.8f, %.8f]\n",
				ptr_state_vec_6XAG->KalmanGain[1].elem[i][0],
				ptr_state_vec_6XAG->KalmanGain[1].elem[i][1],
				ptr_state_vec_6XAG->KalmanGain[1].elem[i][2]);
		}
		printf("Kalman Gain K[2]:\n");
		for(i = 0; i < 3; i++) {
			printf("  [%.8f, %.8f, %.8f]\n",
				ptr_state_vec_6XAG->KalmanGain[2].elem[i][0],
				ptr_state_vec_6XAG->KalmanGain[2].elem[i][1],
				ptr_state_vec_6XAG->KalmanGain[2].elem[i][2]);
		}
	}
#if 0	
	printf("\nKgain %f, %f, %f, %f, %f, %f, %f, %f,%f; %f, %f, %f,%f, %f, %f, %f,%f, %f; %f, %f,%f, %f, %f, %f, %f, %f, %f\n\n",
		ptr_state_vec_6XAG->KalmanGain[0].elem[0][0], ptr_state_vec_6XAG->KalmanGain[0].elem[0][1], ptr_state_vec_6XAG->KalmanGain[0].elem[0][2],
		ptr_state_vec_6XAG->KalmanGain[0].elem[1][0], ptr_state_vec_6XAG->KalmanGain[0].elem[1][1], ptr_state_vec_6XAG->KalmanGain[0].elem[1][2],
		ptr_state_vec_6XAG->KalmanGain[0].elem[2][0], ptr_state_vec_6XAG->KalmanGain[0].elem[2][1], ptr_state_vec_6XAG->KalmanGain[0].elem[2][2],
		ptr_state_vec_6XAG->KalmanGain[1].elem[0][0], ptr_state_vec_6XAG->KalmanGain[1].elem[0][1], ptr_state_vec_6XAG->KalmanGain[1].elem[0][2],
		ptr_state_vec_6XAG->KalmanGain[1].elem[1][0], ptr_state_vec_6XAG->KalmanGain[1].elem[1][1], ptr_state_vec_6XAG->KalmanGain[1].elem[1][2],
		ptr_state_vec_6XAG->KalmanGain[1].elem[2][0], ptr_state_vec_6XAG->KalmanGain[1].elem[2][1], ptr_state_vec_6XAG->KalmanGain[1].elem[2][2],
		ptr_state_vec_6XAG->KalmanGain[2].elem[0][0], ptr_state_vec_6XAG->KalmanGain[2].elem[0][1], ptr_state_vec_6XAG->KalmanGain[2].elem[0][2],
		ptr_state_vec_6XAG->KalmanGain[2].elem[1][0], ptr_state_vec_6XAG->KalmanGain[2].elem[1][1], ptr_state_vec_6XAG->KalmanGain[2].elem[1][2],
		ptr_state_vec_6XAG->KalmanGain[2].elem[2][0], ptr_state_vec_6XAG->KalmanGain[2].elem[2][1], ptr_state_vec_6XAG->KalmanGain[2].elem[2][2]);
#endif // 0

	// Measurement Update of state vector, xe+ = xe- + K*ze 
	// where z is input measurement with ze = C*xe = [(gA- - gG-); (mM- - mG-)] 
	// Mupdt = K[3]*state_var.GravErrPriS;
#if 0
	for( k = CHX; k <= CHZ; k++) {
		Mupdt[3*k] = 0.0;
		Mupdt[3*k+1] = 0.0;
		Mupdt[3*k+2] = 0.0;
		for( i = CHX; i <= CHZ; i++) {
			Mupdt[3*k] += ptr_state_vec_6XAG->KalmanGain[k].elem[0][i] * ptr_state_vec_6XAG->GravErrPriS[i];
			Mupdt[3*k+1] += ptr_state_vec_6XAG->KalmanGain[k].elem[1][i] * ptr_state_vec_6XAG->GravErrPriS[i];
			Mupdt[3*k+2] += ptr_state_vec_6XAG->KalmanGain[k].elem[2][i] * ptr_state_vec_6XAG->GravErrPriS[i];
		}
	}
#endif // 0
	for (k = CHX; k <= CHZ; k++) {
		for (j = CHX; j <= CHZ; j++)
		{
			Mupdt[3*k + j] = 0.0;
			for (i = CHX; i <= CHZ; i++) {
				Mupdt[3*k + j] += ptr_state_vec_6XAG->KalmanGain[k].elem[j][i] * ptr_state_vec_6XAG->GravErrPriS[i];
			}
		}
	}

	if (debug_meas) {
		printf("\nM_updt (full): [%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f]\n",
			Mupdt[0], Mupdt[1], Mupdt[2], Mupdt[3], Mupdt[4], Mupdt[5], Mupdt[6], Mupdt[7], Mupdt[8]);
	}

	// Update the following state information for next iteration
	for(  j = CHX; j <= CHZ; j++) {
		ptr_state_vec_6XAG->OrntErrPostS[j] = Mupdt[j];
		ptr_state_vec_6XAG->BiasErrPostS[j] = Mupdt[j+3];
		ptr_state_vec_6XAG->AccErrPostS[j] = Mupdt[j+6];
		gyro_corr[j] = -Mupdt[j] / SF_DELTA_T;
	}

	if (debug_meas) {
		printf("\nOrntErrPostS: [%.8f, %.8f, %.8f]\n",
			ptr_state_vec_6XAG->OrntErrPostS[0], ptr_state_vec_6XAG->OrntErrPostS[1], ptr_state_vec_6XAG->OrntErrPostS[2]);
		printf("BiasErrPostS AFTER: [%.8f, %.8f, %.8f]\n",
			ptr_state_vec_6XAG->BiasErrPostS[0], ptr_state_vec_6XAG->BiasErrPostS[1], ptr_state_vec_6XAG->BiasErrPostS[2]);
		printf("AccErrPostS: [%.8f, %.8f, %.8f]\n",
			ptr_state_vec_6XAG->AccErrPostS[0], ptr_state_vec_6XAG->AccErrPostS[1], ptr_state_vec_6XAG->AccErrPostS[2]);
	}

	// Update the rotation matrix by rotating it back to remove error, as estimated
	// by OrntErrPostS (error in orientation angles based on measurement update) 
	// Integrate quaternion
	QuatIntegrate(&(ptr_state_vec_6XAG->QuatPost), gyro_corr, SF_DELTA_T, &QuatInt);
	// Normalize quaternion
	QuatNormal(&QuatInt, &(ptr_state_vec_6XAG->QuatPost));
	// Update rotation matrix
	Quat2RotMtx(&(ptr_state_vec_6XAG->QuatPost), ptr_state_vec_6XAG->RotMtxPost);
	// Use accelerometer timestamp to maintain sync with sensor data
	// Check for missing accel data
	if( ptr_state_vec_6XAG->AccData.timestamp - ptr_state_vec_6XAG->MeasupdtTS > SF_ACCEL_MAX_MISS_DUR) {
		// if accel data used is not available for a long time, mark mode_op to degraded accel mode
		ptr_state_vec_6XAG->OpMode &= ~(SF_ACC_MASK);
		ptr_state_vec_6XAG->OpMode |= SF_ACC_MISSING;
	}
	else {
		// if accel data used is available, mark mode_op to normal accel mode
		ptr_state_vec_6XAG->OpMode &= ~(SF_ACC_MASK);
	}
	ptr_state_vec_6XAG->MeasupdtTS = ptr_state_vec_6XAG->AccData.timestamp;

	// Update aposteriori covariance matrix, P_post = (I9 - K*C)*Qw
	//  Compute A= (I9 - K*C)
	for(  i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			if (i == j) {
				IdentityBlkMtx_3x3(&(ptr_state_vec_6XAG->ErrCovMtxPost[i][j]));
			}
			else {
				ZeroBlkMtx_3x3(&(ptr_state_vec_6XAG->ErrCovMtxPost[i][j]));
			}
			MultBlkMtx_3x3(&(ptr_state_vec_6XAG->KalmanGain[i]), &Cmat[j], &Atemp);
			ScaleBlkMtx_3x3(-1.0, &Atemp, &Btemp);
			AddBlkMtx_3x3(&(ptr_state_vec_6XAG->ErrCovMtxPost[i][j]), &Btemp, &Atemp);
			SetBlkMtx_3x3(&Atemp, &(ptr_state_vec_6XAG->ErrCovMtxPost[i][j]));
		}
	}
	// Compuate P_post = A*Qw
	for( i = 0; i < 3; i++) {
		for( j = 0; j < 3; j++) {
			ZeroBlkMtx_3x3(&Atemp);
			for( k = 0; k < 3; k++) {
				MultBlkMtx_3x3(&(ptr_state_vec_6XAG->ErrCovMtxPost[i][k]), &(ptr_state_vec_6XAG->ProcNoiseVar[k][j]), &Btemp);
				AddBlkMtx_3x3(&Atemp, &Btemp, &Gtemp);
				SetBlkMtx_3x3(&Gtemp, &Atemp);
			}
			TranspBlkMtx_3x3(&Atemp, &Btemp);
			AddBlkMtx_3x3(&Atemp, &Btemp, &Gtemp);
			ScaleBlkMtx_3x3(0.5, &Gtemp, &Atemp);
			if (i == j) {
				IdentityBlkMtx_3x3(&Gtemp);
				ScaleBlkMtx_3x3(EPSILON, &Gtemp, &Btemp);
			}
			else {
				ZeroBlkMtx_3x3(&Btemp);
			}
			AddBlkMtx_3x3( &Atemp, &Btemp, &(ptr_state_vec_6XAG->ErrCovMtxPost[i][j]));
		}
	}

	orient_err = ptr_state_vec_6XAG->ErrCovMtxPost[0][0].elem[0][0] * ptr_state_vec_6XAG->ErrCovMtxPost[0][0].elem[0][0];
	orient_err += ptr_state_vec_6XAG->ErrCovMtxPost[0][0].elem[1][1] * ptr_state_vec_6XAG->ErrCovMtxPost[0][0].elem[1][1];
	orient_err += ptr_state_vec_6XAG->ErrCovMtxPost[0][0].elem[2][2] * ptr_state_vec_6XAG->ErrCovMtxPost[0][0].elem[2][2];
	if((ptr_state_vec_6XAG->update_ErrCovMtx == 0) && (orient_err < SF_MAX_ORIENT_ERR)) {
		ptr_state_vec_6XAG->update_ErrCovMtx = 1;
	}

	// Update gyro bias and lin acc based on meas update
	for( j = CHX; j <= CHZ; j++) {
		ptr_state_vec_6XAG->BiasPostS[j] -= ptr_state_vec_6XAG->BiasErrPostS[j];
		// Clamp gyro bias to prevent unbounded growth (from Sources_org/FS/fusion.c lines 1349-1350)
		if (ptr_state_vec_6XAG->BiasPostS[j] < -5.0) ptr_state_vec_6XAG->BiasPostS[j] = -5.0;
		if (ptr_state_vec_6XAG->BiasPostS[j] > 5.0) ptr_state_vec_6XAG->BiasPostS[j] = 5.0;
		ptr_state_vec_6XAG->AccPostS[j] *= ptr_state_vec_6XAG->LinAccTC;
		ptr_state_vec_6XAG->AccPostS[j] -= ptr_state_vec_6XAG->AccErrPostS[j];
		//printf("u = %f and v = %f \n", ptr_state_vec_6XAG->AccErrPostS[j], ptr_state_vec_6XAG->AccPostS[j]);
	}
	for( j = CHX; j <= CHZ; j++) {
		ptr_state_vec_6XAG->AccPostG[j] = 0.0;
		for( k = CHX; k <= CHZ; k++) {
			ptr_state_vec_6XAG->AccPostG[j] += (ptr_state_vec_6XAG->RotMtxPost[j][k] * ptr_state_vec_6XAG->AccPostS[k]);
		}
	}
	ptr_state_vec_6XAG->AccPostG[3] -= GTOMSEC2;

	if (debug_meas) {
		printf("\nQuat AFTER update: [%.15f, %.15f, %.15f, %.15f]\n",
			ptr_state_vec_6XAG->QuatPost.q0, ptr_state_vec_6XAG->QuatPost.q1,
			ptr_state_vec_6XAG->QuatPost.q2, ptr_state_vec_6XAG->QuatPost.q3);
		printf("Final BiasPostS (after update): [%.15f, %.15f, %.15f]\n",
			ptr_state_vec_6XAG->BiasPostS[0], ptr_state_vec_6XAG->BiasPostS[1], ptr_state_vec_6XAG->BiasPostS[2]);
	}

}


void sf_6xag_algo_stop(uintptr_t sf_algo_id)
{
	state_vec_6XAG_t *ptr_state_vec_6XAG;

	// check for input validity
	if (sf_algo_id == (uint32_t)NULL) {
		printf("invalid input \n");
		return;
	}
	// data integrity check
	ptr_state_vec_6XAG = (state_vec_6XAG_t *)sf_algo_id;
	if (ptr_state_vec_6XAG->AlgoID != sf_algo_id) {
		printf("algo id not matching \n");
		return;
	}

	printf("stop algo with algo id = %zu (0x%zx)\n", sf_algo_id, sf_algo_id);
	free(ptr_state_vec_6XAG);

	return;

} /* sf_6xag_algo_stop */


  /**
  * @brief: This function is an interface between SF algorithm and algorithm manager.
  * It is used by algoritm manager to run an existing 6axis (accel+gyro) SF algorithm, after
  * all required sensor data has been collected.
  *
  * @param[in]: sf_algo_id: SF algo id that was provided to algo manager at the time
  *                         of algo initialization (creation)
  *             ptr_algo_out: .
  *
  * @param[out]: None
  *
  */
void sf_6xag_algo_run(uintptr_t        sf_algo_id,
	sf_algo_output_t *ptr_algo_out)
{

	state_vec_6XAG_t *ptr_state_vec_6XAG;

	// check for input validity
	if (sf_algo_id == (uint32_t)NULL) {
		printf("invalid input \n");
		return;
	}
	if (ptr_algo_out == (uint32_t)NULL) {
		printf("invalid output mem \n");
		return;
	}

	ptr_state_vec_6XAG = (state_vec_6XAG_t *)sf_algo_id;
	if (ptr_state_vec_6XAG->AlgoID != sf_algo_id) {
		printf("invalid algo id \n");
		return;
	}

	sf_6xag_algo_run_orig(ptr_state_vec_6XAG, ptr_algo_out);

	// unsignal sf run
	algo_sf_6xag_unsignal_sf_run();

} /* sf_6xag_algo_run */




