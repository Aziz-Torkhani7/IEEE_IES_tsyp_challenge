/**
 * @file ai_interface.h
 * @brief High-level AI communication interface for CubeSat autonomy
 * 
 * Provides structured communication with the AI server for:
 * - Telemetry streaming (sensors, battery, ADCS state)
 * - Navigation corrections (roll, pitch, yaw adjustments)
 * - Obstacle/debris detection alerts
 * - Autonomous maneuver execution
 * 
 * Integrates with existing AI server (ai_server.py on port 5050)
 * Protocol matches: {'sensors': {...}, 'frame': base64_string}
 * Receives: {'corrections': {roll, pitch, yaw}, 'obstacle': {...}}
 * 
 * @author CubeSat Autonomy Team
 * @date 2025-11-10
 */

#ifndef AI_INTERFACE_H
#define AI_INTERFACE_H

#include "tcp_client.h"
#include "../controller/controller.h"
#include <string>
#include <cstdint>
#include <array>

/**
 * @class AIInterface
 * @brief High-level abstraction for AI server communication
 * 
 * Handles JSON serialization/deserialization and provides typed interfaces
 * for telemetry and command exchange with the AI navigation/obstacle system.
 */
class AIInterface {
public:
    /**
     * @struct NavigationCorrections
     * @brief AI-computed attitude corrections for navigation
     */
    struct NavigationCorrections {
        double roll;   ///< Roll correction (radians)
        double pitch;  ///< Pitch correction (radians)
        double yaw;    ///< Yaw correction (radians)
        bool valid;    ///< True if corrections were successfully received
    };
    
    /**
     * @struct AnomalyReport
     * @brief AI-evaluated anomaly assessment and mitigation guidance
     */
    struct AnomalyReport {
        double severity;              ///< Normalized anomaly score (0-1)
        double confidence;            ///< Confidence level (0-1)
        std::string label;            ///< Diagnostic label (e.g., "thermal_rise")
    std::string recommended_action; ///< Suggested immediate response (e.g., "safe_mode_attitude")
        double timestamp;             ///< Server response timestamp (seconds)
        bool valid;                   ///< True if anomaly data was parsed
    };
    
    /**
     * @struct ObstacleInfo
     * @brief Detected obstacle/debris information
     */
    struct ObstacleInfo {
        std::string object_type;  ///< Type: "debris", "satellite", "unknown"
        double distance_m;        ///< Distance in meters
        double angle_deg;         ///< Angle from boresight (degrees)
        bool detected;            ///< True if obstacle detected
    };
    
    /**
     * @brief Construct AI interface
     * @param ai_host AI server address (e.g., "127.0.0.1" or "192.168.1.100")
     * @param ai_port TCP port (default: 5050 matching ai_server.py)
     */
    AIInterface(const std::string& ai_host, uint16_t ai_port = 5050);
    
    /**
     * @brief Initialize connection to AI server
     * @return true if connected successfully
     */
    bool initialize();
    
    /**
     * @brief Send telemetry and receive AI response
     * 
     * Sends current sensor readings to AI server and receives:
     * - Navigation corrections (roll/pitch/yaw)
    * - Obstacle detection results
    * - Anomaly diagnostics and recommended mitigations
     * 
     * @param controller Controller providing sensor access
     * @param corrections Output: AI-computed navigation corrections
     * @param obstacle Output: Detected obstacle information
    * @param anomaly Output: AI-evaluated anomaly report
    * @return true if communication successful
     */
    bool sendTelemetryAndReceive(Controller& controller, 
                                  NavigationCorrections& corrections,
                            ObstacleInfo& obstacle,
                            AnomalyReport& anomaly);
    
    /**
     * @brief Check if connected to AI server
     * @return true if connection is active
     */
    bool isConnected() const;
    
    /**
     * @brief Get last error message
     * @return Human-readable error description
     */
    std::string getLastError() const;

private:
    TCPClient _tcp_client;  ///< Underlying TCP client
    std::string _last_error; ///< Last error message
    
    /**
     * @brief Build telemetry JSON from controller sensors
     * @param controller Controller providing sensor data
     * @return JSON string formatted for ai_server.py
     */
    std::string buildTelemetryJSON(Controller& controller);
    
    /**
     * @brief Parse AI server response JSON
     * @param json_str JSON response from server
     * @param corrections Output: parsed navigation corrections
     * @param obstacle Output: parsed obstacle info
     * @return true if parsing successful
     */
    bool parseResponse(const std::string& json_str,
                       NavigationCorrections& corrections,
                       ObstacleInfo& obstacle,
                       AnomalyReport& anomaly);
    
    /**
     * @brief Safely extract floating-point value from JSON substring
     * @param json JSON string
     * @param key Key to search for
     * @param default_val Value to return if key not found
     * @return Extracted value or default
     */
    double extractDouble(const std::string& json, const std::string& key, double default_val = 0.0);
    
    /**
     * @brief Extract string value from JSON
     * @param json JSON string
     * @param key Key to search for
     * @param default_val Value to return if key not found
     * @return Extracted string or default
     */
    std::string extractString(const std::string& json, const std::string& key, const std::string& default_val = "");
};

#endif // AI_INTERFACE_H
