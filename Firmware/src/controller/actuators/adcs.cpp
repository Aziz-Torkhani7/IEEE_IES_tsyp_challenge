/**
 * @file adcs.cpp
 * @brief ADCS (Attitude Determination and Control System) actuator implementation
 * 
 * Implements attitude control using reaction wheels and/or magnetorquers.
 * Provides rotation commands and integrates with AI navigation corrections.
 * 
 * Features:
 * - Attitude control via rotate() method
 * - AI correction integration
 * - Safe default behaviors
 * 
 * @author CubeSat Autonomy Team
 * @date 2025-11-10
 */

#include "adcs.h"
#include <iostream>
#include <cmath>

/**
 * @brief Apply rotation command to ADCS
 * 
 * Rotates the spacecraft around specified axis by given angle.
 * This method would interface with actual ADCS hardware (reaction wheels,
 * magnetorquers, etc.) in production deployment.
 * 
 * @param axis Rotation axis vector [x, y, z] (should be unit vector)
 * @param angle Rotation angles in degrees [roll, pitch, yaw]
 * 
 * @note In simulation/testing, this logs the command. In production,
 *       it would send I2C/SPI commands to ADCS microcontroller.
 */
void ADCS::rotate(std::vector<uint8_t> axis, std::vector<float> angle) {
    // Store current command
    current_axis = axis;
    current_angle = angle;
    
    // Log rotation command
    std::cout << "ADCS: Rotate command - Axis[";
    for (size_t i = 0; i < axis.size(); i++) {
        std::cout << static_cast<int>(axis[i]);
        if (i < axis.size() - 1) std::cout << ",";
    }
    std::cout << "] Angle[";
    for (size_t i = 0; i < angle.size(); i++) {
        std::cout << angle[i];
        if (i < angle.size() - 1) std::cout << ",";
    }
    std::cout << "]" << std::endl;
    
    // In production: Send to ADCS hardware
    // Example: i2c_write(ADCS_I2C_ADDR, ROTATE_CMD, data, length);
}

/**
 * @brief Write raw bytes to ADCS
 * 
 * Low-level write interface for ADCS commands.
 * Used by Controller to send pre-formatted commands.
 * 
 * @param bytes Raw command bytes
 * @return true if command sent successfully
 * 
 * @note Command format depends on ADCS hardware specification.
 *       Typical format might be:
 *       Byte 0: Command ID (0x01 = rotate, 0x02 = detumble, etc.)
 *       Bytes 1-12: Parameters (axis + angles as floats)
 */
bool ADCS::write(std::vector<uint8_t> bytes) {
    if (bytes.empty()) {
        std::cerr << "ADCS: Write called with empty data" << std::endl;
        return false;
    }
    
    // Log command
    std::cout << "ADCS: Write " << bytes.size() << " bytes: [";
    for (size_t i = 0; i < bytes.size() && i < 16; i++) {
        std::cout << "0x" << std::hex << static_cast<int>(bytes[i]);
        if (i < bytes.size() - 1) std::cout << ",";
    }
    if (bytes.size() > 16) std::cout << "...";
    std::cout << std::dec << "]" << std::endl;
    
    // In production: Send to hardware via I2C/SPI
    // Example implementation:
    // return i2c_write(ADCS_I2C_ADDR, bytes.data(), bytes.size()) == bytes.size();
    
    // For now, return true (simulation mode)
    return true;
}
