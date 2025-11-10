/**
 * @file ai_navigation.h
 * @brief AI-driven navigation and obstacle avoidance module
 * 
 * Integrates AI server corrections for autonomous navigation and debris avoidance.
 * Communicates with ai_server.py to receive real-time navigation corrections
 * based on sensor telemetry and obstacle detection.
 * 
 * Features:
 * - Periodic AI communication (configurable interval)
 * - Automatic ADCS correction application
 * - Obstacle detection and evasive maneuvers
 * - Graceful degradation on AI server failures
 * 
 * @author CubeSat Autonomy Team
 * @date 2025-11-10
 */

#ifndef AI_NAVIGATION_H
#define AI_NAVIGATION_H

#include "module.h"
#include "../communication/ai_interface.h"
#include <cstdint>

/**
 * @class AINavigationModule
 * @brief Module for AI-driven autonomous navigation
 * 
 * Periodically sends telemetry to AI server and applies received
 * navigation corrections to ADCS. Implements obstacle avoidance
 * when debris is detected within critical distance.
 */
class AINavigationModule : public Module {
public:
    /**
     * @brief Construct AI navigation module
     * @param ai_interface Reference to AI communication interface
     * @param adcs_id ADCS actuator ID in controller
     * @param update_interval_ms How often to query AI (milliseconds)
     */
    AINavigationModule(AIInterface& ai_interface, 
                       uint32_t adcs_id,
                       uint32_t update_interval_ms = 1000);
    
    /**
     * @brief Check if AI navigation should execute
     * @param controller System controller
     * @return true if update interval elapsed and AI connected
     */
    bool condition(Controller& controller) override;
    
    /**
     * @brief Execute AI navigation update
     * 
     * Sends telemetry to AI, receives corrections, and applies
     * them to ADCS actuator.
     * 
     * @param controller System controller
     */
    void poll(Controller& controller) override;
    
    /**
     * @brief Check if obstacle is currently detected
     * @return true if obstacle within critical distance
     */
    bool isObstacleDetected() const { return _obstacle_detected; }
    
    /**
     * @brief Get last obstacle distance
     * @return Distance in meters (999.0 if none detected)
     */
    double getObstacleDistance() const { return _last_obstacle_distance; }

private:
    AIInterface& _ai_interface;         ///< AI communication interface
    uint32_t _adcs_id;                  ///< ADCS actuator ID
    uint32_t _update_interval_ms;       ///< Update period in milliseconds
    uint64_t _last_update_time_ms;      ///< Last successful update timestamp
    bool _obstacle_detected;            ///< True if obstacle within range
    double _last_obstacle_distance;     ///< Last detected obstacle distance
    
    /**
     * @brief Get current time in milliseconds
     * @return Monotonic timestamp
     */
    uint64_t getCurrentTimeMs();
    
    /**
     * @brief Apply AI corrections to ADCS
     * @param controller System controller
     * @param corrections Navigation corrections from AI
     */
    void applyCorrections(Controller& controller, 
                         const AIInterface::NavigationCorrections& corrections);
    
    /**
     * @brief Execute obstacle avoidance maneuver
     * @param controller System controller
     * @param obstacle Detected obstacle information
     */
    void executeAvoidanceManeuver(Controller& controller,
                                  const AIInterface::ObstacleInfo& obstacle);

    /**
     * @brief React to AI anomaly diagnostics and apply mitigations
     * @param controller System controller
     * @param anomaly AI-provided anomaly assessment
     */
    void handleAnomaly(Controller& controller,
                       const AIInterface::AnomalyReport& anomaly);
};

#endif // AI_NAVIGATION_H
