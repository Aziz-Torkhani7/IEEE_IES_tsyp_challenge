/**
 * @file ai_navigation.cpp
 * @brief AI navigation module implementation
 * 
 * Implements autonomous navigation using AI server corrections.
 * Integrates obstacle detection and evasive maneuvers.
 * 
 * @author CubeSat Autonomy Team
 * @date 2025-11-10
 */

#include "ai_navigation.h"
#include "../controller/actuators/adcs.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <algorithm>

AINavigationModule::AINavigationModule(AIInterface& ai_interface,
                                       uint32_t adcs_id,
                                       uint32_t update_interval_ms)
    : _ai_interface(ai_interface),
      _adcs_id(adcs_id),
      _update_interval_ms(update_interval_ms),
      _last_update_time_ms(0),
      _obstacle_detected(false),
      _last_obstacle_distance(999.0) {
}

bool AINavigationModule::condition(Controller& controller) {
    // Check if AI is connected
    if (!_ai_interface.isConnected()) {
        return false;
    }
    
    // Check if update interval has elapsed
    uint64_t now = getCurrentTimeMs();
    if (now - _last_update_time_ms >= _update_interval_ms) {
        return true;
    }
    
    return false;
}

void AINavigationModule::poll(Controller& controller) {
    std::cout << "AI Navigation: Polling AI server..." << std::endl;
    
    // Send telemetry and receive corrections
    AIInterface::NavigationCorrections corrections;
    AIInterface::ObstacleInfo obstacle;
    AIInterface::AnomalyReport anomaly;
    anomaly.valid = false;
    
    bool success = _ai_interface.sendTelemetryAndReceive(controller, corrections, obstacle, anomaly);
    
    if (!success) {
        std::cerr << "AI Navigation: Failed to communicate with AI server: " 
                  << _ai_interface.getLastError() << std::endl;
        return;
    }
    
    // Update timestamp
    _last_update_time_ms = getCurrentTimeMs();
    
    // Process corrections
    if (corrections.valid) {
        std::cout << "AI Navigation: Received corrections - "
                  << "Roll: " << corrections.roll 
                  << " Pitch: " << corrections.pitch 
                  << " Yaw: " << corrections.yaw << std::endl;
        
        applyCorrections(controller, corrections);
    }
    
    // Process obstacle detection
    if (obstacle.detected) {
        std::cout << "AI Navigation: OBSTACLE DETECTED! "
                  << "Type: " << obstacle.object_type
                  << " Distance: " << obstacle.distance_m << "m"
                  << " Angle: " << obstacle.angle_deg << "°" << std::endl;
        
        _obstacle_detected = true;
        _last_obstacle_distance = obstacle.distance_m;
        
        // Execute avoidance if critically close
        if (obstacle.distance_m < 30.0) {
            std::cout << "AI Navigation: Executing evasive maneuver!" << std::endl;
            executeAvoidanceManeuver(controller, obstacle);
        }
    } else {
        _obstacle_detected = false;
        _last_obstacle_distance = 999.0;
    }

    // Process anomaly diagnostics
    if (anomaly.valid) {
        std::cout << "AI Navigation: Anomaly score=" << anomaly.severity
                  << " label=" << anomaly.label
                  << " confidence=" << anomaly.confidence
                  << " action=" << anomaly.recommended_action << std::endl;
        handleAnomaly(controller, anomaly);
    }
}

void AINavigationModule::applyCorrections(Controller& controller,
                                          const AIInterface::NavigationCorrections& corrections) {
    /**
     * Apply AI navigation corrections to ADCS
     * Converts corrections (in radians) to ADCS commands
     */
    
    try {
        // Get ADCS actuator
        Actuator& actuator = controller.getActuator(_adcs_id);
        ADCS* adcs = dynamic_cast<ADCS*>(&actuator);
        
        if (!adcs) {
            std::cerr << "AI Navigation: ADCS actuator not found at ID " << _adcs_id << std::endl;
            return;
        }
        
        // Convert corrections to degrees
        float roll_deg = static_cast<float>(corrections.roll * 180.0 / M_PI);
        float pitch_deg = static_cast<float>(corrections.pitch * 180.0 / M_PI);
        float yaw_deg = static_cast<float>(corrections.yaw * 180.0 / M_PI);
        
        // Apply small corrections (limit to ±5 degrees for safety)
        roll_deg = std::max(-5.0f, std::min(5.0f, roll_deg));
        pitch_deg = std::max(-5.0f, std::min(5.0f, pitch_deg));
        yaw_deg = std::max(-5.0f, std::min(5.0f, yaw_deg));
        
        // Build rotation command
        // Axis: [1, 1, 1] means apply to all axes
        std::vector<uint8_t> axis = {1, 1, 1};
        std::vector<float> angles = {roll_deg, pitch_deg, yaw_deg};
        
        // Apply rotation
        adcs->rotate(axis, angles);
        
        std::cout << "AI Navigation: Applied corrections - "
                  << "Roll: " << roll_deg << "° "
                  << "Pitch: " << pitch_deg << "° "
                  << "Yaw: " << yaw_deg << "°" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "AI Navigation: Error applying corrections: " << e.what() << std::endl;
    }
}

void AINavigationModule::executeAvoidanceManeuver(Controller& controller,
                                                   const AIInterface::ObstacleInfo& obstacle) {
    /**
     * Execute emergency avoidance maneuver
     * 
     * Strategy: Rotate away from obstacle direction
     * - If obstacle is ahead (angle < 30°): Roll 90° + Pitch up 30°
     * - If obstacle is to side: Yaw away from it
     */
    
    try {
        Actuator& actuator = controller.getActuator(_adcs_id);
        ADCS* adcs = dynamic_cast<ADCS*>(&actuator);
        
        if (!adcs) {
            std::cerr << "AI Navigation: ADCS not available for avoidance!" << std::endl;
            return;
        }
        
        // Determine avoidance strategy based on obstacle angle
        float yaw_correction = 0.0f;
        float pitch_correction = 0.0f;
        float roll_correction = 0.0f;
        
        if (std::abs(obstacle.angle_deg) < 30.0) {
            // Obstacle directly ahead - perform evasive roll + pitch
            roll_correction = 45.0f;  // Roll 45 degrees
            pitch_correction = 20.0f; // Pitch up
            std::cout << "AI Navigation: Obstacle ahead - Rolling and pitching" << std::endl;
        } else {
            // Obstacle to side - yaw away
            yaw_correction = (obstacle.angle_deg > 0) ? -30.0f : 30.0f;
            std::cout << "AI Navigation: Obstacle to side - Yawing " << yaw_correction << "°" << std::endl;
        }
        
        // Execute maneuver
        std::vector<uint8_t> axis = {1, 1, 1};
        std::vector<float> angles = {roll_correction, pitch_correction, yaw_correction};
        
        adcs->rotate(axis, angles);
        
        std::cout << "AI Navigation: Avoidance maneuver executed!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "AI Navigation: Error executing avoidance: " << e.what() << std::endl;
    }
}

void AINavigationModule::handleAnomaly(Controller& controller,
                                       const AIInterface::AnomalyReport& anomaly) {
    if (anomaly.severity < 0.65) {
        return; // Nominal or low-severity state
    }

    try {
        if (anomaly.recommended_action == "safe_mode_attitude") {
            Actuator& actuator = controller.getActuator(_adcs_id);
            ADCS* adcs = dynamic_cast<ADCS*>(&actuator);
            if (!adcs) {
                std::cerr << "AI Navigation: ADCS unavailable for safe-mode hold" << std::endl;
                return;
            }

            // Command a gentle sun-pointing pitch to preserve power generation
            std::vector<uint8_t> axis = {1, 1, 1};
            std::vector<float> angles = {0.0f, 12.0f, 0.0f};
            adcs->rotate(axis, angles);
            std::cout << "AI Navigation: Applied safe-mode attitude hold" << std::endl;
        } else if (anomaly.recommended_action == "shed_noncritical") {
            // Placeholder: integrate with power manager once available
            std::cout << "AI Navigation: Recommend load shedding to protect power bus" << std::endl;
        } else if (anomaly.recommended_action == "boost_mppt") {
            std::cout << "AI Navigation: Recommend increasing MPPT duty cycle" << std::endl;
        } else if (anomaly.recommended_action == "balance_thermal") {
            std::cout << "AI Navigation: Recommend thermal balancing maneuver" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "AI Navigation: Failed to process anomaly mitigation: " << e.what() << std::endl;
    }
}

uint64_t AINavigationModule::getCurrentTimeMs() {
    using namespace std::chrono;
    auto now = steady_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch());
    return ms.count();
}
