/**
 * @file ai_interface.cpp
 * @brief AI communication interface implementation
 * 
 * Implements JSON-based telemetry exchange with AI server.
 * Uses simple string parsing for lightweight JSON handling (no external libs).
 * 
 * @author CubeSat Autonomy Team
 * @date 2025-11-10
 */

#include "ai_interface.h"
#include "../controller/sensors/BQ34Z100.h"
#include "../controller/sensors/ina219.h"
#include "../controller/sensors/lidar.h"
#include <sstream>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <cstdlib>

AIInterface::AIInterface(const std::string& ai_host, uint16_t ai_port)
    : _tcp_client(ai_host, ai_port) {
}

bool AIInterface::initialize() {
    std::cout << "AI Interface: Initializing connection..." << std::endl;
    
    bool connected = _tcp_client.connect(5);
    
    if (!connected) {
        _last_error = "Failed to connect: " + _tcp_client.getLastError();
        std::cerr << "AI Interface: " << _last_error << std::endl;
        return false;
    }
    
    std::cout << "AI Interface: Connected successfully" << std::endl;
    return true;
}

bool AIInterface::sendTelemetryAndReceive(Controller& controller,
                                           NavigationCorrections& corrections,
                                           ObstacleInfo& obstacle,
                                           AnomalyReport& anomaly) {
    // Build telemetry JSON
    std::string telemetry = buildTelemetryJSON(controller);
    
    // Send to AI server
    if (!_tcp_client.sendJSON(telemetry)) {
        _last_error = "Failed to send telemetry: " + _tcp_client.getLastError();
        std::cerr << "AI Interface: " << _last_error << std::endl;
        corrections.valid = false;
        obstacle.detected = false;
        anomaly.valid = false;
        return false;
    }
    
    // Receive AI response
    std::string response = _tcp_client.receiveJSON(2000); // 2 second timeout
    
    if (response.empty()) {
        _last_error = "No response from AI server";
        std::cerr << "AI Interface: " << _last_error << std::endl;
        corrections.valid = false;
        obstacle.detected = false;
        anomaly.valid = false;
        return false;
    }
    
    // Parse response
    bool parsed = parseResponse(response, corrections, obstacle, anomaly);
    
    if (!parsed) {
        _last_error = "Failed to parse AI response";
        std::cerr << "AI Interface: " << _last_error << std::endl;
        return false;
    }
    
    return true;
}

bool AIInterface::isConnected() const {
    return _tcp_client.isConnected();
}

std::string AIInterface::getLastError() const {
    return _last_error;
}

std::string AIInterface::buildTelemetryJSON(Controller& controller) {
    /**
     * Build JSON matching ai_server.py expected format:
     * {
     *   "sensors": {
     *     "acc": [x, y, z],
     *     "gyro": [x, y, z],
     *     "mag": [x, y, z],
     *     "sun": [x, y, z],
     *     "temp": value,
     *     "press": value,
    *     "battery_soc": value,
    *     "battery_voltage": value,
    *     "battery_current": value,
    *     "bus_current": value,
    *     "panel_voltage": value,
    *     "panel_current": value,
    *     "mppt_power": value,
    *     "solar_power": value,
    *     "mppt_duty": value,
    *     "board_temp": value
     *   }
     * }
     * 
     * Note: Using simple string formatting instead of JSON library
     * for minimal dependencies and embedded compatibility
     */
    
    std::ostringstream json;
    json << std::fixed << std::setprecision(3);
    
    json << "{\"sensors\":{";
    
    // Accelerometer (placeholder - add actual IMU sensor when available)
    json << "\"acc\":[0.0,0.0,9.81],";
    
    // Gyroscope (placeholder - add actual IMU sensor when available)
    json << "\"gyro\":[0.0,0.0,0.0],";
    
    // Magnetometer (placeholder - add actual magnetometer when available)
    json << "\"mag\":[0.0,0.0,50.0],";
    
    // Sun sensor (placeholder - add actual sun sensor when available)
    json << "\"sun\":[0.0,0.0,1.0],";
    
    // Temperature and pressure (placeholder - add actual sensors when available)
    json << "\"temp\":25.0,";
    json << "\"press\":101.3,";
    
    try {
        // Battery telemetry from BQ34Z100
        // Note: getSensor returns reference, need to cast to specific type
        // For now using placeholder values - integrate actual sensor reading
        json << "\"battery_soc\":85.0,";
        json << "\"battery_voltage\":7.4,";
        json << "\"battery_current\":-0.300,";
        json << "\"bus_current\":0.420,";
        json << "\"panel_voltage\":8.200,";
        json << "\"panel_current\":0.600,";
        json << "\"mppt_power\":4.920,";
        json << "\"solar_power\":4.920,";
        json << "\"mppt_duty\":0.640,";
        json << "\"board_temp\":26.500";

    } catch (...) {
        // If sensors not available, use safe defaults
        json << "\"battery_soc\":50.0,";
        json << "\"battery_voltage\":7.0,";
        json << "\"battery_current\":0.000,";
        json << "\"bus_current\":0.150,";
        json << "\"panel_voltage\":7.100,";
        json << "\"panel_current\":0.300,";
    json << "\"mppt_power\":2.130,";
    json << "\"solar_power\":2.130,";
        json << "\"mppt_duty\":0.520,";
        json << "\"board_temp\":25.000";
    }
    
    json << "}}";
    
    return json.str();
}

bool AIInterface::parseResponse(const std::string& json_str,
                                 NavigationCorrections& corrections,
                                 ObstacleInfo& obstacle,
                                 AnomalyReport& anomaly) {
    /**
     * Parse AI response format:
     * {
     *   "corrections": {"roll": 0.1, "pitch": -0.05, "yaw": 0.02},
     *   "obstacle": {"object": "debris", "distance_m": 20.0, "angle_deg": 5.0}
     * }
     */
    
    // Initialize outputs
    corrections.valid = false;
    corrections.roll = 0.0;
    corrections.pitch = 0.0;
    corrections.yaw = 0.0;
    
    obstacle.detected = false;
    obstacle.object_type = "unknown";
    obstacle.distance_m = 999.0;
    obstacle.angle_deg = 0.0;
    
    // Initialize anomaly defaults
    anomaly.valid = false;
    anomaly.severity = 0.0;
    anomaly.confidence = 0.0;
    anomaly.label = "nominal";
    anomaly.recommended_action = "none";
    anomaly.timestamp = 0.0;
    
    // Check for corrections
    if (json_str.find("\"corrections\"") != std::string::npos) {
        corrections.roll = extractDouble(json_str, "\"roll\"");
        corrections.pitch = extractDouble(json_str, "\"pitch\"");
        corrections.yaw = extractDouble(json_str, "\"yaw\"");
        corrections.valid = true;
    }
    
    // Check for obstacle detection
    if (json_str.find("\"obstacle\"") != std::string::npos) {
        obstacle.object_type = extractString(json_str, "\"object\"");
        obstacle.distance_m = extractDouble(json_str, "\"distance_m\"", 999.0);
        obstacle.angle_deg = extractDouble(json_str, "\"angle_deg\"", 0.0);
        
        // Consider debris detected if within 50m
        if (obstacle.distance_m < 50.0) {
            obstacle.detected = true;
        }
    }

    // Parse anomaly report if present
    if (json_str.find("\"anomaly\"") != std::string::npos) {
        anomaly.severity = extractDouble(json_str, "\"score\"", 0.0);
        anomaly.confidence = extractDouble(json_str, "\"confidence\"", 0.0);
        anomaly.label = extractString(json_str, "\"label\"", "nominal");
        anomaly.recommended_action = extractString(json_str, "\"recommended_action\"", "none");
        anomaly.timestamp = extractDouble(json_str, "\"timestamp\"", 0.0);
        anomaly.valid = true;
    } else {
        anomaly.recommended_action = extractString(json_str, "\"action\"", "none");
        anomaly.timestamp = extractDouble(json_str, "\"timestamp\"", 0.0);
        anomaly.valid = (json_str.find("\"action\"") != std::string::npos);
    }
    
    return true;
}

double AIInterface::extractDouble(const std::string& json, const std::string& key, double default_val) {
    /**
     * Simple JSON value extraction (not a full parser, but sufficient)
     * Searches for: "key": value
     */
    size_t key_pos = json.find(key);
    if (key_pos == std::string::npos) {
        return default_val;
    }
    
    // Find the colon after the key
    size_t colon_pos = json.find(':', key_pos);
    if (colon_pos == std::string::npos) {
        return default_val;
    }
    
    // Extract number (skip whitespace)
    size_t start = colon_pos + 1;
    while (start < json.length() && std::isspace(json[start])) {
        start++;
    }
    
    // Parse number
    char* end;
    double value = std::strtod(json.c_str() + start, &end);
    
    return value;
}

std::string AIInterface::extractString(const std::string& json, const std::string& key, const std::string& default_val) {
    /**
     * Extract string value: "key": "value"
     */
    size_t key_pos = json.find(key);
    if (key_pos == std::string::npos) {
        return default_val;
    }
    
    // Find opening quote of value
    size_t colon_pos = json.find(':', key_pos);
    if (colon_pos == std::string::npos) {
        return default_val;
    }
    
    size_t quote1 = json.find('\"', colon_pos);
    if (quote1 == std::string::npos) {
        return default_val;
    }
    
    // Find closing quote
    size_t quote2 = json.find('\"', quote1 + 1);
    if (quote2 == std::string::npos) {
        return default_val;
    }
    
    return json.substr(quote1 + 1, quote2 - quote1 - 1);
}
