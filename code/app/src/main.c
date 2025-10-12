
/*****
* Author: Vikas Yadav
* Date: 2020
*/

#include "main.h"
#include <inttypes.h>

//#define RUN_TEST_ONLY

// Compile-time flag for fusion algorithm selection
// Define USE_9AXIS_FUSION for 9-axis (A+G+M), otherwise 6-axis (A+G) is used

// Error check: Ensure both flags are not defined simultaneously
#if defined(USE_9AXIS_FUSION) && defined(USE_6AXIS_FUSION)
    #error "Cannot define both USE_9AXIS_FUSION and USE_6AXIS_FUSION simultaneously. Choose one."
#endif

// Auto-define 6-axis if neither flag is defined
#if !defined(USE_9AXIS_FUSION) && !defined(USE_6AXIS_FUSION)
    #define USE_6AXIS_FUSION
#endif

// Fallback: If 9-axis is not defined, ensure 6-axis is defined
#ifndef USE_9AXIS_FUSION
    #ifndef USE_6AXIS_FUSION
        #define USE_6AXIS_FUSION
    #endif
#endif

//#include "AGMQ_true_90.h"
//#include "AGMQ_AOPData_0930.h"
//#include "test_vector_motionless.h"
//#include "test_vector_moving.h"
#include "test_input_output_0930.h"

// Compatibility with old test data structure
typedef test_sensor_sample_t test_data_sample;
#define test_vec sensor_input_data

// start time offset to match PC time with recorded data time
int64_t start_time_offset_ns;

sf_algo_init_data_t algo_init_data;
sf_algo_output_t    algo_output;

void test_utility(void);

int main(void)
{
	uint32_t         i;    // loop counters
	uint32_t         len;
	uintptr_t        sf_algo_id;  // Algorithm handle (6-axis or 9-axis)
	sensor_data_t    acc;
	sensor_data_t    gyro;
#ifdef USE_9AXIS_FUSION
	sensor_data_t    mag;  // Magnetometer data for 9-axis fusion
#endif
	uint32_t            sf_ready_to_run = 0;
	uint32_t            sf_run_cnt = 0;
	int64_t             sampling_delay_ns = 0;

#ifdef RUN_TEST_ONLY	
	/**************************************/
	test_utility();
	printf("Done Testing. Hit enter to exit. \n");
	getchar();
	return 0;
/**************************************/
#else

	// Show platform information
	print_platform_info();
	
	len = sizeof(test_vec) / sizeof(test_data_sample);
	printf("\n=== Sensor Fusion Test ===\n");
#ifdef USE_9AXIS_FUSION
	printf("Algorithm: 9-axis fusion (Accelerometer + Gyroscope + Magnetometer)\n");
#else
	printf("Algorithm: 6-axis fusion (Accelerometer + Gyroscope)\n");
#endif
	printf("Total samples available: %d\n", len);
	
	// Limit to first 50 samples for quick test
	if (len > 50) {
		len = 50;
		printf("Running quick test with first %d samples\n", len);
	}
	
	if (len > 0) {
		start_time_offset_ns = test_vec[0].ts;
		printf("Starting timestamp: %llu ns\n", (unsigned long long)start_time_offset_ns);
	}

	algo_init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
	algo_init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
	algo_init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

	// Initialize sensor fusion algorithm based on compile-time flag
#ifdef USE_9AXIS_FUSION
	sf_algo_id = sf_9xagm_algo_init(&algo_init_data);
	printf("Initialized 9-axis sensor fusion algorithm\n");
#else
	sf_algo_id = sf_6xag_algo_init(&algo_init_data);
	printf("Initialized 6-axis sensor fusion algorithm\n");
#endif

	
	// Run through the input data set, one by one
	printf("\nProcessing sensor data...\n");
	for (i = 0; i < len; i++) {
		// Progress indicator for every 1st sample
		if (i % 1 == 0) {
			printf("Processing sample %d/%d (sensor_id=%d)\n", i+1, len, test_vec[i].id);
		}
		if (test_vec[i].id == 0) {  // Accelerometer data (sensor_id = 0)
			acc.sensordata[0] = test_vec[i].x;
			acc.sensordata[1] = test_vec[i].y;
			acc.sensordata[2] = test_vec[i].z;
			acc.timestamp = (test_vec[i].ts - start_time_offset_ns)*10000000;
			acc.sensorID = ACC;
#ifdef USE_9AXIS_FUSION
			sf_ready_to_run = sf_9xagm_data_preproc(sf_algo_id, &acc);
#else
			sf_ready_to_run = sf_6xag_data_preproc(sf_algo_id, &acc);
#endif
			printf("Acc ready = %d\n", sf_ready_to_run);
			if (i + 1 < len) sampling_delay_ns = test_vec[i+1].ts - test_vec[i].ts;
		}
		else if (test_vec[i].id == 1) {  // Gyroscope data (sensor_id = 1)
			gyro.sensordata[0] = test_vec[i].x;
			gyro.sensordata[1] = test_vec[i].y;
			gyro.sensordata[2] = test_vec[i].z;
			gyro.timestamp = (test_vec[i].ts - start_time_offset_ns)*10000000;
			gyro.sensorID = GYRO;
#ifdef USE_9AXIS_FUSION
			sf_ready_to_run = sf_9xagm_data_preproc(sf_algo_id, &gyro);
#else
			sf_ready_to_run = sf_6xag_data_preproc(sf_algo_id, &gyro);
#endif
			printf("Gyro ready = %d\n", sf_ready_to_run);
			if (i + 1 < len) sampling_delay_ns = test_vec[i+1].ts - test_vec[i].ts;
		}
#ifdef USE_9AXIS_FUSION
		else if (test_vec[i].id == 2) {  // Magnetometer data (sensor_id = 2)
			mag.sensordata[0] = test_vec[i].x;
			mag.sensordata[1] = test_vec[i].y;
			mag.sensordata[2] = test_vec[i].z;
			mag.timestamp = (test_vec[i].ts - start_time_offset_ns)*10000000;
			mag.sensorID = MAG;
			sf_ready_to_run = sf_9xagm_data_preproc(sf_algo_id, &mag);
			printf("Mag ready = %d\n", sf_ready_to_run);
			if (i + 1 < len) sampling_delay_ns = test_vec[i+1].ts - test_vec[i].ts;
		}
#endif
		// Check if we have enough sensor data to run fusion algorithm
#ifdef USE_9AXIS_FUSION
		if ( sf_ready_to_run == 7 ) {  // 9-axis: A(1) + G(2) + M(4) = 7
			// run one pass of 9-axis fusion algorithm
			sf_9xagm_algo_run(sf_algo_id, &algo_output);
#else
		if ( sf_ready_to_run == 3 ) {  // 6-axis: A(1) + G(2) = 3
			// run one pass of 6-axis fusion algorithm
			sf_6xag_algo_run(sf_algo_id, &algo_output);
#endif
		    sf_ready_to_run = 0;
		    sf_run_cnt++;
		    (void)sf_run_cnt;  // Suppress unused variable warning
		    
		    // Show quaternion output for first few results
		    if (sf_run_cnt <= 5) {
		        printf("  -> Fusion result %d: quat=(%.6f, %.6f, %.6f, %.6f), euler=(%.2f°, %.2f°, %.2f°)\n", 
		               sf_run_cnt, algo_output.quat.q0, algo_output.quat.q1, algo_output.quat.q2, algo_output.quat.q3,
		               algo_output.orientation[0], algo_output.orientation[1], algo_output.orientation[2]);
		    }
		}
		if (sampling_delay_ns > 0) {
			_sleep(1);
			sampling_delay_ns = 0;
		}
		
	}


	// Stop sensor fusion algorithm (free memory etc)
#ifdef USE_9AXIS_FUSION
	sf_9xagm_algo_stop(sf_algo_id);
	printf("Stopped 9-axis sensor fusion algorithm\n");
#else
	sf_6xag_algo_stop(sf_algo_id);
	printf("Stopped 6-axis sensor fusion algorithm\n");
#endif

	/**************************************/
	printf("Done. Hit enter to exit. \n");
	getchar();	
	return 0;

#endif

}
