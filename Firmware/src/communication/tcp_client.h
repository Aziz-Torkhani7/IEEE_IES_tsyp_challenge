/**
 * @file tcp_client.h
 * @brief TCP client for AI server communication
 * 
 * Provides bidirectional TCP communication with the AI server for:
 * - Sending telemetry data (sensors, battery, attitude)
 * - Receiving AI corrections (navigation, obstacle avoidance)
 * - Debris avoidance maneuver commands
 * 
 * Protocol: JSON over TCP (newline-delimited)
 * Default AI Server: localhost:5050
 * 
 * Thread-safe for concurrent operations
 * 
 * @author CubeSat Autonomy Team
 * @date 2025-11-10
 */

#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <string>
#include <vector>
#include <cstdint>

/**
 * @class TCPClient
 * @brief TCP socket client for AI server communication
 * 
 * Manages persistent TCP connection to AI server. Handles reconnection
 * on failures and implements timeout mechanisms for reliability.
 * 
 * Example usage:
 * @code
 * TCPClient client("192.168.1.100", 5050);
 * if (client.connect()) {
 *     client.sendJSON("{\"sensors\":{\"temp\":25.3}}");
 *     std::string response = client.receiveJSON(1000);
 * }
 * @endcode
 */
class TCPClient {
public:
    /**
     * @brief Construct TCP client
     * @param host AI server IP address or hostname
     * @param port TCP port number (default: 5050 to match ai_server.py)
     */
    TCPClient(const std::string& host, uint16_t port = 5050);
    
    /**
     * @brief Destructor - closes connection if open
     */
    ~TCPClient();
    
    /**
     * @brief Establish connection to AI server
     * @param timeout_sec Connection timeout in seconds
     * @return true if connected successfully, false otherwise
     */
    bool connect(int timeout_sec = 5);
    
    /**
     * @brief Send JSON message to AI server
     * @param json_str JSON-formatted string (newline automatically added)
     * @return true if sent successfully
     */
    bool sendJSON(const std::string& json_str);
    
    /**
     * @brief Receive JSON message from AI server
     * @param timeout_ms Timeout in milliseconds (0 = non-blocking)
     * @return JSON string if available, empty string on timeout/error
     */
    std::string receiveJSON(int timeout_ms = 1000);
    
    /**
     * @brief Check connection status
     * @return true if currently connected
     */
    bool isConnected() const { return _connected; }
    
    /**
     * @brief Close connection gracefully
     */
    void disconnect();
    
    /**
     * @brief Get last error message
     * @return Human-readable error description
     */
    std::string getLastError() const { return _last_error; }

private:
    int _socket_fd;              ///< Socket file descriptor
    std::string _host;           ///< AI server address
    uint16_t _port;              ///< TCP port
    bool _connected;             ///< Connection status flag
    std::string _last_error;     ///< Last error message
    std::string _recv_buffer;    ///< Buffer for incomplete messages
    
    /**
     * @brief Set socket timeout for operations
     * @param timeout_ms Timeout in milliseconds
     * @return true if successful
     */
    bool setSocketTimeout(int timeout_ms);
    
    /**
     * @brief Receive data until newline delimiter
     * @param timeout_ms Timeout in milliseconds
     * @return Complete message or empty string
     */
    std::string receiveUntilNewline(int timeout_ms);
};

#endif // TCP_CLIENT_H
