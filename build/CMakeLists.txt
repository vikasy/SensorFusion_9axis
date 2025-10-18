cmake_minimum_required(VERSION 3.10)
project(SF_9axis VERSION 1.0.0 LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler options
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -std=c99 -O2 -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pointer-to-int-cast -Wno-int-to-pointer-cast -Wno-pointer-integer-compare")
add_definitions(-D_POSIX_C_SOURCE=199309L)

# Sensor fusion algorithm selection (using shared config)
# Set FUSION=9axis for 9-axis fusion, default is 6-axis
if(NOT DEFINED FUSION)
    set(FUSION "6axis")
endif()

if(FUSION STREQUAL "9axis")
    add_definitions(-DUSE_9AXIS_FUSION)
    set(FUSION_TYPE "9-axis")
else()
    add_definitions(-DUSE_6AXIS_FUSION)
    set(FUSION_TYPE "6-axis")
endif()

# Platform-specific optimizations (same as Makefile)
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    # macOS optimizations (preserve IEEE floating point for sensor fusion algorithms)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -mtune=native -fno-math-errno")
    add_definitions(-D__APPLE_USE_RFC_3542)
endif()

# Include directories
include_directories(
    ../code/algo/inc
    ../code/app/inc
    ../test/data/testdata/fusion  # Required for main.c test data
)

# Source directories
set(ALGO_SRC_DIR "../code/algo/src")
set(APP_SRC_DIR "../code/app/src")

# Common algorithm source files
set(COMMON_ALGO_SOURCES
    ${ALGO_SRC_DIR}/algo_sf_approx.c
    ${ALGO_SRC_DIR}/algo_sf_approxmath.c
    ${ALGO_SRC_DIR}/algo_sf_matrix.c
    ${ALGO_SRC_DIR}/algo_sf_matrixmath.c
    ${ALGO_SRC_DIR}/algo_sf_orientation.c
    ${ALGO_SRC_DIR}/algo_sf_quatmath.c
    ${ALGO_SRC_DIR}/algo_sf_sensordata.c
    ${ALGO_SRC_DIR}/algo_sf_mag_cal.c
)

# Fusion-specific algorithm sources - conditional compilation
if(FUSION STREQUAL "9axis")
    set(FUSION_ALGO_SOURCES ${ALGO_SRC_DIR}/algo_sf_9x_sensor_fusion.c)
else()
    set(FUSION_ALGO_SOURCES ${ALGO_SRC_DIR}/algo_sf_6x_sensor_fusion.c)
endif()

# All algorithm sources
set(ALGO_SOURCES ${COMMON_ALGO_SOURCES} ${FUSION_ALGO_SOURCES})

# Application source files
set(APP_SOURCES ${APP_SRC_DIR}/main.c)

# Test source files (disabled due to compilation issues)
set(TEST_SOURCES "")

# Target names
set(MAIN_TARGET "sf_9axis")
set(TEST_TARGET "sf_9axis_test")
set(STATIC_LIB "libsf9axis")

# Create static library for algorithms (algorithm only, no main)
add_library(${STATIC_LIB} STATIC ${ALGO_SOURCES})

# Create main executable (links main.c with sensor fusion library)
add_executable(${MAIN_TARGET} ${APP_SOURCES})
target_link_libraries(${MAIN_TARGET} ${STATIC_LIB} m)

# Create test executable if test sources exist
if(TEST_SOURCES)
    add_executable(${TEST_TARGET} ${TEST_SOURCES})
    target_link_libraries(${TEST_TARGET} ${STATIC_LIB} m)
endif()

# Set output directories inside build folder
set_target_properties(${MAIN_TARGET} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
)

set_target_properties(${STATIC_LIB} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
    OUTPUT_NAME "sf9axis"
)

if(TEST_SOURCES)
    set_target_properties(${TEST_TARGET} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )
endif()



# Install targets (same as Makefile)
set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Install prefix")

install(TARGETS ${MAIN_TARGET} ${STATIC_LIB}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Install headers (same as Makefile)
install(DIRECTORY ../code/algo/inc/ DESTINATION include/sf9axis
    FILES_MATCHING PATTERN "*.h"
)

# Add build information target
add_custom_target(info
    COMMAND ${CMAKE_COMMAND} -E echo "Project: ${PROJECT_NAME} v${PROJECT_VERSION}"
    COMMAND ${CMAKE_COMMAND} -E echo "Fusion Algorithm: ${FUSION_TYPE}"
    COMMAND ${CMAKE_COMMAND} -E echo "Compiler: ${CMAKE_C_COMPILER}"
    COMMAND ${CMAKE_COMMAND} -E echo "Flags: ${CMAKE_C_FLAGS}"
    COMMAND ${CMAKE_COMMAND} -E echo "Algorithm Sources: ${ALGO_SOURCES}"
    COMMAND ${CMAKE_COMMAND} -E echo "App Sources: ${APP_SOURCES}"
    VERBATIM
)

# Add library-only target (same as Makefile)
add_custom_target(lib DEPENDS ${STATIC_LIB})

# Add test target if tests exist (same as Makefile)
if(TEST_SOURCES)
    add_custom_target(test DEPENDS ${TEST_TARGET})
endif()

# ==============================================================================
# TEST SUITE INTEGRATION
# ==============================================================================

option(BUILD_TESTS "Build test suite" ON)

if(BUILD_TESTS)
    message(STATUS "==================================================")
    message(STATUS "Test Suite Configuration")
    message(STATUS "==================================================")

    # Unity test framework
    set(UNITY_SRC_DIR ../test/unity/src)
    set(UNITY_SRC ${UNITY_SRC_DIR}/unity.c)

    # Test helper sources
    set(TEST_HELPER_SOURCES
        ../test/common/helpers/test_helpers.c
        ../test/common/helpers/test_data_generator.c
    )

    # Additional include directories for tests
    include_directories(
        ${UNITY_SRC_DIR}
        ../test/common/helpers
        ../test/common/test_vectors
        ../test/testdata/quaternion
        ../test/testdata/fusion
    )

    # All algorithm sources (both 6-axis and 9-axis for comprehensive testing)
    set(ALL_ALGO_SOURCES
        ${COMMON_ALGO_SOURCES}
        ${ALGO_SRC_DIR}/algo_sf_6x_sensor_fusion.c
        ${ALGO_SRC_DIR}/algo_sf_9x_sensor_fusion.c
    )

    # --------------------------------------------------------------------------
    # UNIT TESTS
    # --------------------------------------------------------------------------

    # Test: Quaternion Math
    add_executable(test_quatmath
        ../test/tests/unit_tests/test_quatmath.c
        ${UNITY_SRC}
        ${ALGO_SRC_DIR}/algo_sf_quatmath.c
        ${ALGO_SRC_DIR}/algo_sf_approxmath.c
    )
    target_compile_definitions(test_quatmath PRIVATE UNITY_INCLUDE_DOUBLE)
    target_link_libraries(test_quatmath m)
    set_target_properties(test_quatmath PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # Test: Matrix Math
    add_executable(test_matrixmath
        ../test/tests/unit_tests/test_matrixmath.c
        ${UNITY_SRC}
        ${ALGO_SRC_DIR}/algo_sf_matrixmath.c
        ${ALGO_SRC_DIR}/algo_sf_matrix.c
    )
    target_compile_definitions(test_matrixmath PRIVATE UNITY_INCLUDE_DOUBLE)
    target_link_libraries(test_matrixmath m)
    set_target_properties(test_matrixmath PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # --------------------------------------------------------------------------
    # INTEGRATION TESTS
    # --------------------------------------------------------------------------

    # Test: 6-axis Fusion Integration
    add_executable(test_6axis_fusion
        ../test/tests/integration_tests/test_6axis_fusion.c
        ${UNITY_SRC}
        ${ALL_ALGO_SOURCES}
        ${TEST_HELPER_SOURCES}
    )
    target_compile_definitions(test_6axis_fusion PRIVATE UNITY_INCLUDE_DOUBLE USE_6AXIS_FUSION USE_9AXIS_FUSION)
    target_link_libraries(test_6axis_fusion m)
    set_target_properties(test_6axis_fusion PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # Test: 9-axis Fusion Integration
    add_executable(test_9axis_fusion
        ../test/tests/integration_tests/test_9axis_fusion.c
        ${UNITY_SRC}
        ${ALL_ALGO_SOURCES}
        ${TEST_HELPER_SOURCES}
    )
    target_compile_definitions(test_9axis_fusion PRIVATE UNITY_INCLUDE_DOUBLE USE_6AXIS_FUSION USE_9AXIS_FUSION)
    target_link_libraries(test_9axis_fusion m)
    set_target_properties(test_9axis_fusion PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # --------------------------------------------------------------------------
    # VALIDATION TESTS
    # --------------------------------------------------------------------------

    # Test: 6-axis Simple Validation
    add_executable(test_6axis_simple
        ../test/tests/validation_tests/test_6axis_simple.c
        ${ALL_ALGO_SOURCES}
        ${TEST_HELPER_SOURCES}
    )
    target_compile_definitions(test_6axis_simple PRIVATE USE_6AXIS_FUSION USE_9AXIS_FUSION)
    target_link_libraries(test_6axis_simple m)
    set_target_properties(test_6axis_simple PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # # Test: Realistic Validation
    # add_executable(test_realistic_validation
    #     ../test/validation/test_realistic_validation.c
    #     ${UNITY_SRC}
    #     ${ALL_ALGO_SOURCES}
    #     ${TEST_HELPER_SOURCES}
    # )
    # target_compile_definitions(test_realistic_validation PRIVATE UNITY_INCLUDE_DOUBLE)
    # target_link_libraries(test_realistic_validation m)
    # set_target_properties(test_realistic_validation PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    # )

    # --------------------------------------------------------------------------
    # END-TO-END TESTS
    # --------------------------------------------------------------------------

    # Test: 6-axis E2E with Synthetic Datasets
    add_executable(test_6axis_synthetic
        ../test/tests/e2e_tests/test_6axis_synthetic.c
        ${ALL_ALGO_SOURCES}
        ${TEST_HELPER_SOURCES}
    )
    target_compile_definitions(test_6axis_synthetic PRIVATE USE_6AXIS_FUSION USE_9AXIS_FUSION)
    target_link_libraries(test_6axis_synthetic m)
    set_target_properties(test_6axis_synthetic PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # Test: 9-axis E2E with Synthetic Datasets
    add_executable(test_9axis_synthetic
        ../test/tests/e2e_tests/test_9axis_synthetic.c
        ${ALL_ALGO_SOURCES}
        ${TEST_HELPER_SOURCES}
    )
    target_compile_definitions(test_9axis_synthetic PRIVATE USE_6AXIS_FUSION USE_9AXIS_FUSION)
    target_link_libraries(test_9axis_synthetic m)
    set_target_properties(test_9axis_synthetic PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # # Test: 6-axis E2E with RepoIMU Dataset (TODO: Update API)
    # add_executable(test_6axis_e2e
    #     ../test/e2e/test_6axis_e2e.c
    #     ${ALL_ALGO_SOURCES}
    #     ${TEST_HELPER_SOURCES}
    # )
    # target_link_libraries(test_6axis_e2e m)
    # set_target_properties(test_6axis_e2e PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    # )

    # # Test: 9-axis E2E with RepoIMU Dataset (TODO: Update API)
    # add_executable(test_9axis_e2e
    #     ../test/e2e/test_9axis_e2e.c
    #     ${ALL_ALGO_SOURCES}
    #     ${TEST_HELPER_SOURCES}
    # )
    # target_link_libraries(test_9axis_e2e m)
    # set_target_properties(test_9axis_e2e PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    # )

    # # Test: 6-axis FIUMARG Database
    # add_executable(test_6axis_fiumargdb
    #     ../test/e2e/test_6axis_fiumargdb.c
    #     ${ALL_ALGO_SOURCES}
    #     ${TEST_HELPER_SOURCES}
    # )
    # target_link_libraries(test_6axis_fiumargdb m)
    # set_target_properties(test_6axis_fiumargdb PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    # )

    # # Test: 9-axis FIUMARG Database
    # add_executable(test_9axis_fiumargdb
    #     ../test/e2e/test_9axis_fiumargdb.c
    #     ${ALL_ALGO_SOURCES}
    #     ${TEST_HELPER_SOURCES}
    # )
    # target_link_libraries(test_9axis_fiumargdb m)
    # set_target_properties(test_9axis_fiumargdb PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    # )

    # --------------------------------------------------------------------------
    # DIAGNOSTIC TOOLS
    # --------------------------------------------------------------------------

    # Test: Orientation Mapping Diagnostic
    add_executable(test_orientation_mapping
        ../test/tests/diagnostic_tests/test_orientation_mapping.c
        ${ALL_ALGO_SOURCES}
        ${TEST_HELPER_SOURCES}
    )
    target_compile_definitions(test_orientation_mapping PRIVATE USE_6AXIS_FUSION USE_9AXIS_FUSION)
    target_link_libraries(test_orientation_mapping m)
    set_target_properties(test_orientation_mapping PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # Test: Fusion Testdata Validation (using converted Excel data)
    add_executable(test_fusion_testdata
        ../test/tests/e2e_tests/test_fusion_testdata.c
        ${ALL_ALGO_SOURCES}
        ${TEST_HELPER_SOURCES}
    )
    target_compile_definitions(test_fusion_testdata PRIVATE USE_6AXIS_FUSION USE_9AXIS_FUSION)
    target_link_libraries(test_fusion_testdata m)
    target_include_directories(test_fusion_testdata PRIVATE
        ../test/data/datasets/fusion
    )
    set_target_properties(test_fusion_testdata PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # Test: Compare C output against Python reference
    add_executable(test_compare_python
        ../test/code/test_compare_python.c
        ${ALL_ALGO_SOURCES}
    )
    target_compile_definitions(test_compare_python PRIVATE USE_6AXIS_FUSION USE_9AXIS_FUSION)
    target_link_libraries(test_compare_python m)
    target_include_directories(test_compare_python PRIVATE
        ../test/data/datasets/fusion
    )
    set_target_properties(test_compare_python PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # Tool: Generate C reference quaternions
    add_executable(generate_c_reference
        ../test/tests/e2e_tests/generate_c_reference.c
        ${ALL_ALGO_SOURCES}
    )
    target_compile_definitions(generate_c_reference PRIVATE USE_6AXIS_FUSION USE_9AXIS_FUSION)
    target_link_libraries(generate_c_reference m)
    target_include_directories(generate_c_reference PRIVATE
        ../test/data/datasets/fusion
    )
    set_target_properties(generate_c_reference PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    )

    # # Test: Coordinate Frame Discovery
    # add_executable(test_coordinate_frame
    #     ../test/diagnostic/test_coordinate_frame.c
    #     ${ALL_ALGO_SOURCES}
    # )
    # target_link_libraries(test_coordinate_frame m)
    # set_target_properties(test_coordinate_frame PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    # )

    # # Test: Tilt Formula Analysis
    # add_executable(test_tilt_formula
    #     ../test/diagnostic/test_tilt_formula.c
    #     ${ALGO_SRC_DIR}/algo_sf_approxmath.c
    # )
    # target_link_libraries(test_tilt_formula m)
    # set_target_properties(test_tilt_formula PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
    # )

    # --------------------------------------------------------------------------
    # CTEST INTEGRATION
    # --------------------------------------------------------------------------

    enable_testing()

    # Configure CTest output
    set(CTEST_OUTPUT_ON_FAILURE ON)

    # Create test_artifacts directory for test outputs
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test_artifacts)

    # Unit Tests
    add_test(NAME QuaternionMath COMMAND test_quatmath)
    add_test(NAME MatrixMath COMMAND test_matrixmath)

    # Integration Tests
    add_test(NAME 6AxisFusionIntegration COMMAND test_6axis_fusion)
    add_test(NAME 9AxisFusionIntegration COMMAND test_9axis_fusion)

    # Set test properties
    set_tests_properties(QuaternionMath MatrixMath PROPERTIES TIMEOUT 30)
    set_tests_properties(6AxisFusionIntegration 9AxisFusionIntegration PROPERTIES TIMEOUT 60)

    # Custom target to clean up temporary test artifacts
    # CTest creates Testing/Temporary directory automatically; we delete it after tests complete
    add_custom_target(clean_test_artifacts
        COMMAND ${CMAKE_COMMAND} -E echo "Cleaning temporary test artifacts..."
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/Testing/Temporary
        COMMENT "Removing Testing/Temporary directory after test completion"
        VERBATIM
    )

    # --------------------------------------------------------------------------
    # CUSTOM TEST TARGETS
    # --------------------------------------------------------------------------

    # Target: Run all unit tests
    add_custom_target(test_unit
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/bin/test_quatmath
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/bin/test_matrixmath
        DEPENDS test_quatmath test_matrixmath
        COMMENT "Running unit tests..."
        VERBATIM
    )

    # Target: Run all integration tests
    add_custom_target(test_integration
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/bin/test_6axis_fusion
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/bin/test_9axis_fusion
        DEPENDS test_6axis_fusion test_9axis_fusion
        COMMENT "Running integration tests..."
        VERBATIM
    )

    # Target: Build all tests
    add_custom_target(build_all_tests
        DEPENDS
            test_quatmath
            test_matrixmath
            test_6axis_fusion
            test_9axis_fusion
            # Add more test targets here as they are fixed
        COMMENT "Building all tests..."
    )

    message(STATUS "Test binary output: ${CMAKE_CURRENT_SOURCE_DIR}/bin")
    message(STATUS "Unity double precision: ENABLED")
    message(STATUS "Unit tests: test_quatmath, test_matrixmath")
    message(STATUS "Integration tests: test_6axis_fusion, test_9axis_fusion")
    message(STATUS "Note: Validation/E2E tests commented out (API needs update)")
    message(STATUS "==================================================")

endif(BUILD_TESTS)
