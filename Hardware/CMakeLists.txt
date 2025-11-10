cmake_minimum_required(VERSION 3.22)
project(HardwareInterface VERSION 1.0)

# Enable C++11 for modern features
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
add_executable(
	HardwareInterface
	# Main entry point
	src/main.cpp
	
	# Controller core
	src/controller/controller.cpp
	src/controller/device.cpp
	src/controller/sensor.cpp
	src/controller/actuator.cpp
	
	# Sensors
	src/controller/sensors/lidar.cpp
	src/controller/sensors/BQ34Z100.cpp
	src/controller/sensors/ina219.cpp
	
	# Actuators
	src/controller/actuators/adcs.cpp
	
	# Modules (autonomous behaviors)
	src/modules/module.cpp
	src/modules/mppt.cpp
	src/modules/ai_navigation.cpp
	
	# Communication
	src/communication/tcp_client.cpp
	src/communication/ai_interface.cpp
	
	# Energy management
	src/energy/manager.cpp
)

# Link libraries
target_link_libraries(HardwareInterface 
	pthread  # For TCP sockets and threading
)

# Include directories (allows #include <filename.h> from src/)
target_include_directories(HardwareInterface PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Compiler warnings for code quality
target_compile_options(HardwareInterface PRIVATE 
	-Wall 
	-Wextra 
	-Wpedantic
)

# Print build configuration
message(STATUS "Building CubeSat Autonomous System v${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")