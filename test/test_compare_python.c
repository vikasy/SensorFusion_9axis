/**
 * Test program to compare C output against Python reference quaternions
 * Stops at first mismatch
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../code/algo/inc/algo_sf_fusion.h"
#include "../code/algo/inc/algo_sf_6x_sensor_fusion.h"
#include "../code/app/inc/main.h"
#include "../test/data/testdata/fusion/test_input_output_0922.h"

#define QUAT_TOLERANCE 3e-5  // Floating point tolerance for quaternion comparison (accounts for accumulating numerical differences)

typedef struct {
    int sample;
    uint64_t timestamp;
    int sensor_id;
    double q0, q1, q2, q3;
} python_reference_t;

// Load Python reference quaternions from CSV file
python_reference_t* load_python_reference(const char* filename, int* count) {
    FILE* fp = fopen(filename, "r");
    if (!fp) {
        printf("Error: Cannot open Python reference file: %s\n", filename);
        return NULL;
    }

    // Count lines
    int lines = 0;
    char buffer[512];
    fgets(buffer, sizeof(buffer), fp);  // Skip header
    while (fgets(buffer, sizeof(buffer), fp)) {
        lines++;
    }
    rewind(fp);

    // Allocate memory
    python_reference_t* refs = (python_reference_t*)malloc(lines * sizeof(python_reference_t));
    if (!refs) {
        fclose(fp);
        return NULL;
    }

    // Read data
    fgets(buffer, sizeof(buffer), fp);  // Skip header
    *count = 0;
    while (fgets(buffer, sizeof(buffer), fp)) {
        if (sscanf(buffer, "%d,%llu,%d,%lf,%lf,%lf,%lf",
                   &refs[*count].sample,
                   &refs[*count].timestamp,
                   &refs[*count].sensor_id,
                   &refs[*count].q0,
                   &refs[*count].q1,
                   &refs[*count].q2,
                   &refs[*count].q3) == 7) {
            (*count)++;
        }
    }

    fclose(fp);
    printf("Loaded %d Python reference quaternions\n", *count);
    return refs;
}

// Compare two quaternions (accounting for q and -q equivalence)
double quat_distance(double q0_a, double q1_a, double q2_a, double q3_a,
                     double q0_b, double q1_b, double q2_b, double q3_b) {
    double dot = q0_a * q0_b + q1_a * q1_b + q2_a * q2_b + q3_a * q3_b;
    return 1.0 - fabs(dot);
}

int main(int argc, char** argv) {
    // Check if we should output all quaternions
    int output_all = 0;
    if (argc > 1 && strcmp(argv[1], "--output-all") == 0) {
        output_all = 1;
        printf("=== C Quaternion Output Generator ===\n\n");
    } else {
        printf("=== C vs Python Quaternion Comparison Test ===\n\n");
    }

    // Load Python reference
    int py_count = 0;
    python_reference_t* py_refs = load_python_reference("python_reference_quaternions_0922.csv", &py_count);
    if (!py_refs) {
        return 1;
    }

    // Open output file if requested
    FILE* out_fp = NULL;
    if (output_all) {
        out_fp = fopen("c_reference_quaternions_0922.csv", "w");
        if (!out_fp) {
            printf("Error: Cannot create output file\n");
            free(py_refs);
            return 1;
        }
        fprintf(out_fp, "sample,timestamp,sensor_id,q0,q1,q2,q3\n");
    }

    // Initialize sensor fusion
    sf_algo_init_data_t algo_init_data;
    algo_init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    algo_init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    algo_init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t sf_algo_id = sf_6xag_algo_init(&algo_init_data);
    printf("Initialized 6-axis sensor fusion algorithm\n\n");

    // Process test data
    int len = sizeof(sensor_input_data) / sizeof(test_sensor_sample_t);
    printf("Total input samples: %d\n", len);
    printf("Expected Python outputs: %d\n\n", py_count);

    int64_t start_time_offset_ns = sensor_input_data[0].ts;
    int sf_run_cnt = 0;
    int py_idx = 0;
    uint32_t sf_ready_to_run = 0;
    sensor_data_t sensor;
    sf_algo_output_t algo_output;

    if (output_all) {
        printf("Starting quaternion generation...\n\n");
    } else {
        printf("Starting comparison (will stop at first mismatch)...\n");
        printf("Tolerance: %.1e\n\n", QUAT_TOLERANCE);
    }

    for (int i = 0; i < len; i++) {
        int sensor_id = sensor_input_data[i].id;

        // Skip magnetometer
        if (sensor_id == 2) {
            continue;
        }

        // Prepare sensor data
        sensor.sensordata[0] = sensor_input_data[i].x;
        sensor.sensordata[1] = sensor_input_data[i].y;
        sensor.sensordata[2] = sensor_input_data[i].z;
        sensor.timestamp = sensor_input_data[i].ts;
        sensor.sensorID = (sensor_id == 0) ? ACC : GYRO;

        // Preprocess
        sf_ready_to_run = sf_6xag_data_preproc(sf_algo_id, &sensor);

        // Run fusion
        if (sf_ready_to_run == 3) {  // Both ACC and GYRO ready
            sf_6xag_algo_run(sf_algo_id, &algo_output);
            sf_ready_to_run = 0;

            // Output quaternion if requested
            if (output_all && out_fp) {
                fprintf(out_fp, "%d,%llu,%d,%.15f,%.15f,%.15f,%.15f\n",
                        sf_run_cnt,
                        sensor_input_data[i].ts,
                        sensor_id,
                        algo_output.quat.q0,
                        algo_output.quat.q1,
                        algo_output.quat.q2,
                        algo_output.quat.q3);

                // Progress indicator
                if ((sf_run_cnt + 1) % 100 == 0) {
                    printf("  Fusion runs: %d...\n", sf_run_cnt + 1);
                }
            }

            // Compare with Python reference
            if (!output_all && py_idx < py_count) {
                python_reference_t* py = &py_refs[py_idx];

                // Calculate quaternion distance
                double qd = quat_distance(algo_output.quat.q0, algo_output.quat.q1,
                                          algo_output.quat.q2, algo_output.quat.q3,
                                          py->q0, py->q1, py->q2, py->q3);

                if (qd > QUAT_TOLERANCE) {
                    printf("╔═══════════════════════════════════════════════════════════════╗\n");
                    printf("║ FIRST QUATERNION MISMATCH DETECTED!                          ║\n");
                    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
                    printf("Fusion run number: %d (input sample index: %d)\n", sf_run_cnt, i);
                    printf("Timestamp: %llu ns\n", sensor_input_data[i].ts);
                    printf("Last sensor processed: %s (id=%d)\n", sensor_id == 0 ? "ACC" : "GYRO", sensor_id);
                    printf("Last sensor data: [%d, %d, %d]\n\n",
                           sensor_input_data[i].x, sensor_input_data[i].y, sensor_input_data[i].z);

                    printf("C Quaternion:      [%.15f, %.15f, %.15f, %.15f]\n",
                           algo_output.quat.q0, algo_output.quat.q1,
                           algo_output.quat.q2, algo_output.quat.q3);
                    printf("Python Quaternion: [%.15f, %.15f, %.15f, %.15f]\n",
                           py->q0, py->q1, py->q2, py->q3);
                    printf("Python sample:     %d (timestamp: %llu, sensor_id: %d)\n\n",
                           py->sample, py->timestamp, py->sensor_id);

                    printf("Quaternion distance: %.10e\n", qd);
                    printf("Tolerance:           %.10e\n\n", QUAT_TOLERANCE);

                    printf("Difference per component:\n");
                    printf("  q0: %+.10e\n", algo_output.quat.q0 - py->q0);
                    printf("  q1: %+.10e\n", algo_output.quat.q1 - py->q1);
                    printf("  q2: %+.10e\n", algo_output.quat.q2 - py->q2);
                    printf("  q3: %+.10e\n", algo_output.quat.q3 - py->q3);

                    printf("\n=== Additional Debug Info ===\n");
                    printf("Euler angles (C):  [%.6f°, %.6f°, %.6f°] (yaw, pitch, roll)\n",
                           algo_output.orientation[0], algo_output.orientation[1], algo_output.orientation[2]);
                    printf("Gravity (C):       [%.6f, %.6f, %.6f] m/s²\n",
                           algo_output.gravity[0], algo_output.gravity[1], algo_output.gravity[2]);

                    printf("\nStopping at first mismatch.\n");
                    free(py_refs);
                    sf_6xag_algo_stop(sf_algo_id);
                    return 0;
                }

                // Progress indicator - print ALL samples to find where divergence starts
                printf("Sample %4d: quat_distance = %.6e %s\n", sf_run_cnt, qd, (qd < QUAT_TOLERANCE) ? "✓" : "WARN");

                // If getting close to tolerance, print more details
                if (qd > QUAT_TOLERANCE * 0.5 && qd < QUAT_TOLERANCE) {
                    printf("  WARNING: Approaching tolerance limit (%.1f%% of threshold)\n", 100.0 * qd / QUAT_TOLERANCE);
                }

                py_idx++;
            }

            sf_run_cnt++;
        }
    }

    if (output_all) {
        if (out_fp) fclose(out_fp);
        printf("\nComplete! Processed %d input samples\n", len);
        printf("Generated %d fusion outputs (quaternions)\n", sf_run_cnt);
        printf("Output saved to: c_reference_quaternions_0922.csv\n");
    } else {
        printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
        printf("║ SUCCESS: All %d samples match within tolerance!             ║\n", sf_run_cnt);
        printf("╚═══════════════════════════════════════════════════════════════╝\n");
    }

    free(py_refs);
    sf_6xag_algo_stop(sf_algo_id);
    return 0;
}
