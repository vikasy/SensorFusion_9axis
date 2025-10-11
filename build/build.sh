#!/bin/bash
# Simple CMake build script for SensorFusion project
# Usage: ./build.sh [6axis|9axis] [clean]

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to show usage
show_usage() {
    echo "Usage: $0 [fusion_type] [options]"
    echo ""
    echo "Fusion Types:"
    echo "  6axis           Build with 6-axis sensor fusion (default)"
    echo "  9axis           Build with 9-axis sensor fusion"
    echo ""
    echo "Options:"
    echo "  clean           Clean build directory"
    echo "  debug           Build with debug symbols (default)"
    echo "  release         Build in release mode"
    echo ""
    echo "Examples:"
    echo "  $0               # Build 6axis debug"
    echo "  $0 9axis         # Build 9axis debug" 
    echo "  $0 6axis release # Build 6axis release"
    echo "  $0 clean         # Clean build files"
}

# Default options
FUSION_TYPE="6axis"
BUILD_TYPE="Debug"
CLEAN_BUILD=false
JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        6axis|9axis)
            FUSION_TYPE="$1"
            shift
            ;;
        clean)
            CLEAN_BUILD=true
            shift
            ;;
        debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        release)
            BUILD_TYPE="Release"
            shift
            ;;
        -h|--help|help)
            show_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Error: Unknown option '$1'${NC}"
            show_usage
            exit 1
            ;;
    esac
done

# Handle clean request
if [ "$CLEAN_BUILD" = true ]; then
    echo -e "${BLUE}Cleaning build directories...${NC}"
    rm -rf cmake obj bin/* lib/*
    echo -e "${GREEN}Clean completed.${NC}"
    exit 0
fi

# Build directories
BUILD_DIR="cmake"
CODE_DIR="../code"

echo -e "${BLUE}Building SensorFusion with ${FUSION_TYPE} algorithm (${BUILD_TYPE})...${NC}"

# Create build directory
mkdir -p ${BUILD_DIR}

# Configure with CMake (from build directory, output to cmake subdirectory)
echo -e "${BLUE}Configuring project...${NC}"
cmake -B ${BUILD_DIR} -S . -DFUSION=${FUSION_TYPE} -DCMAKE_BUILD_TYPE=${BUILD_TYPE}

# Build
echo -e "${BLUE}Building project...${NC}"
cmake --build ${BUILD_DIR} --config ${BUILD_TYPE} -j${JOBS}

# Move object files to obj directory
echo -e "${BLUE}Organizing object files...${NC}"
mkdir -p obj
find ${BUILD_DIR} -name "*.o" -exec mv {} obj/ \;

# Check if build was successful
if [[ -f "bin/SensorFusion" ]]; then
    echo -e "${GREEN}✅ Build completed successfully!${NC}"
    echo -e "${BLUE}Executable: $(pwd)/bin/SensorFusion${NC}"
    echo -e "${BLUE}Library: $(pwd)/lib/libsensorfusion.a${NC}"
    
    # Show built files
    echo -e "${BLUE}Built files:${NC}"
    echo "Executables:"
    ls -la bin/ 2>/dev/null || echo "  (none)"
    echo "Libraries:"
    ls -la lib/ 2>/dev/null || echo "  (none)"
else
    echo -e "${RED}❌ Build failed - executable not found${NC}"
    exit 1
fi
