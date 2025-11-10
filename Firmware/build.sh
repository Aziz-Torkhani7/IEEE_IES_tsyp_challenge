#!/bin/bash
# Build script for CubeSat Autonomous System
# Usage: ./build.sh [clean|rebuild|run]

set -e  # Exit on error

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="$SCRIPT_DIR/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=================================================="
echo "  CubeSat Autonomous System - Build Script"
echo -e "==================================================${NC}\n"

# Parse command
CMD="${1:-build}"

case "$CMD" in
    clean)
        echo -e "${YELLOW}Cleaning build directory...${NC}"
        rm -rf "$BUILD_DIR"
        echo -e "${GREEN}✓ Clean complete${NC}"
        ;;
        
    rebuild)
        echo -e "${YELLOW}Rebuilding from scratch...${NC}"
        rm -rf "$BUILD_DIR"
        mkdir -p "$BUILD_DIR"
        cd "$BUILD_DIR"
        
        echo -e "${YELLOW}Running CMake...${NC}"
        cmake ..
        
        echo -e "${YELLOW}Compiling...${NC}"
        make -j$(nproc)
        
        echo -e "${GREEN}✓ Rebuild complete${NC}"
        echo -e "\nBinary: ${BUILD_DIR}/HardwareInterface"
        ;;
        
    build)
        mkdir -p "$BUILD_DIR"
        cd "$BUILD_DIR"
        
        if [ ! -f "Makefile" ]; then
            echo -e "${YELLOW}Running CMake (first build)...${NC}"
            cmake ..
        fi
        
        echo -e "${YELLOW}Compiling...${NC}"
        make -j$(nproc)
        
        echo -e "${GREEN}✓ Build complete${NC}"
        echo -e "\nBinary: ${BUILD_DIR}/HardwareInterface"
        ;;
        
    run)
        if [ ! -f "$BUILD_DIR/HardwareInterface" ]; then
            echo -e "${RED}✗ Binary not found. Building first...${NC}"
            "$0" build
        fi
        
        AI_HOST="${2:-127.0.0.1}"
        AI_PORT="${3:-5050}"
        
        echo -e "${GREEN}Starting CubeSat System${NC}"
        echo -e "AI Server: ${AI_HOST}:${AI_PORT}\n"
        
        "$BUILD_DIR/HardwareInterface" "$AI_HOST" "$AI_PORT"
        ;;
        
    *)
        echo -e "${RED}Unknown command: $CMD${NC}"
        echo ""
        echo "Usage: $0 [command] [options]"
        echo ""
        echo "Commands:"
        echo "  build          Build project (default)"
        echo "  clean          Remove build directory"
        echo "  rebuild        Clean and build from scratch"
        echo "  run [host] [port]  Run the system"
        echo ""
        echo "Examples:"
        echo "  $0 build"
        echo "  $0 rebuild"
        echo "  $0 run 192.168.1.100 5050"
        exit 1
        ;;
esac
