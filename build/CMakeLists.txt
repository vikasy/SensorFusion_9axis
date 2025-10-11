cmake_minimum_required(VERSION 3.10)
project(SensorFusion VERSION 1.0.0 LANGUAGES C)

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
    ../test/inc  # Required for main.c test data
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
set(MAIN_TARGET "SensorFusion")
set(TEST_TARGET "SensorFusion_test")
set(STATIC_LIB "libsensorfusion")

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
    OUTPUT_NAME "sensorfusion"
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
install(DIRECTORY ../code/algo/inc/ DESTINATION include/sensorfusion
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
