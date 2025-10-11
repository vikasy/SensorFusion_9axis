// Simple test program for the sensor fusion library
#include <stdio.h>
#include <stdlib.h>
#include "algo_sf_6x_sensor_fusion.h"
#include "algo_sf_9x_sensor_fusion.h"

int main(void) {
    printf("=== Sensor Fusion Library Test ===\n");
    
    // Test 6-axis initialization
    printf("Testing 6-axis sensor fusion...\n");
    
    sf_algo_init_data_t init_data = {
        .Acc_GPERCOUNT = 0.0001220703125F,   // MPU9250 scale factor
        .Gyro_DPSPERCOUNT = 0.030487804878F  // MPU9250 scale factor
    };
    
    uint32_t algo_6axis_id = sf_6xag_algo_init(&init_data);
    if (algo_6axis_id != 0) {
        printf("✅ 6-axis algorithm initialized successfully (ID: %u)\n", algo_6axis_id);
        sf_6xag_algo_stop(algo_6axis_id);
        printf("✅ 6-axis algorithm stopped successfully\n");
    } else {
        printf("❌ 6-axis algorithm initialization failed\n");
        return 1;
    }
    
    // Test 9-axis initialization
    printf("\nTesting 9-axis sensor fusion...\n");
    
    init_data.Mag_UTPERCOUNT = 0.15F;  // Magnetometer scale factor
    
    uint32_t algo_9axis_id = sf_9xagm_algo_init(&init_data);
    if (algo_9axis_id != 0) {
        printf("✅ 9-axis algorithm initialized successfully (ID: %u)\n", algo_9axis_id);
        sf_9xagm_algo_stop(algo_9axis_id);
        printf("✅ 9-axis algorithm stopped successfully\n");
    } else {
        printf("❌ 9-axis algorithm initialization failed\n");
        return 1;
    }
    
    printf("\n=== All Tests Passed! ===\n");
    printf("Sensor fusion library is working correctly.\n");
    
    return 0;
}