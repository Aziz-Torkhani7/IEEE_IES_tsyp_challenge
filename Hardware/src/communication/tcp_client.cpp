/**
 * @file tcp_client.cpp
 * @brief TCP client implementation for AI server communication
 * 
 * Implements reliable TCP communication with JSON message framing.
 * Handles connection errors, timeouts, and automatic reconnection.
 * 
 * @author CubeSat Autonomy Team
 * @date 2025-11-10
 */

#include "tcp_client.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <cstring>
#include <errno.h>
#include <iostream>

TCPClient::TCPClient(const std::string& host, uint16_t port)
    : _socket_fd(-1), _host(host), _port(port), _connected(false) {
}

TCPClient::~TCPClient() {
    disconnect();
}

bool TCPClient::connect(int timeout_sec) {
    // Clean up existing connection
    if (_socket_fd >= 0) {
        disconnect();
    }
    
    // Create socket
    _socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (_socket_fd < 0) {
        _last_error = "Failed to create socket: " + std::string(strerror(errno));
        std::cerr << "TCP: " << _last_error << std::endl;
        return false;
    }
    
    // Set non-blocking for timeout control during connection
    int flags = fcntl(_socket_fd, F_GETFL, 0);
    fcntl(_socket_fd, F_SETFL, flags | O_NONBLOCK);
    
    // Setup server address
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(_port);
    
    // Convert IP address from string to binary
    if (inet_pton(AF_INET, _host.c_str(), &server_addr.sin_addr) <= 0) {
        _last_error = "Invalid address: " + _host;
        std::cerr << "TCP: " << _last_error << std::endl;
        close(_socket_fd);
        _socket_fd = -1;
        return false;
    }
    
    // Attempt connection
    int ret = ::connect(_socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr));
    
    if (ret < 0 && errno != EINPROGRESS) {
        _last_error = "Connection failed immediately: " + std::string(strerror(errno));
        std::cerr << "TCP: " << _last_error << std::endl;
        close(_socket_fd);
        _socket_fd = -1;
        return false;
    }
    
    // Wait for connection with timeout
    struct pollfd pfd;
    pfd.fd = _socket_fd;
    pfd.events = POLLOUT;
    
    ret = poll(&pfd, 1, timeout_sec * 1000);
    
    if (ret <= 0) {
        _last_error = ret == 0 ? "Connection timeout" : "Poll error: " + std::string(strerror(errno));
        std::cerr << "TCP: " << _last_error << std::endl;
        close(_socket_fd);
        _socket_fd = -1;
        return false;
    }
    
    // Check if connection succeeded
    int socket_error;
    socklen_t len = sizeof(socket_error);
    getsockopt(_socket_fd, SOL_SOCKET, SO_ERROR, &socket_error, &len);
    
    if (socket_error != 0) {
        _last_error = "Connection error: " + std::string(strerror(socket_error));
        std::cerr << "TCP: " << _last_error << std::endl;
        close(_socket_fd);
        _socket_fd = -1;
        return false;
    }
    
    // Set back to blocking mode
    fcntl(_socket_fd, F_SETFL, flags);
    
    _connected = true;
    std::cout << "TCP: Connected to " << _host << ":" << _port << std::endl;
    return true;
}

bool TCPClient::sendJSON(const std::string& json_str) {
    if (!_connected) {
        _last_error = "Not connected";
        return false;
    }
    
    // Add newline terminator if not present (required by ai_server.py)
    std::string msg = json_str;
    if (msg.empty() || msg.back() != '\n') {
        msg += '\n';
    }
    
    // Send data
    ssize_t sent = send(_socket_fd, msg.c_str(), msg.length(), 0);
    
    if (sent < 0) {
        _last_error = "Send failed: " + std::string(strerror(errno));
        std::cerr << "TCP: " << _last_error << std::endl;
        _connected = false;
        return false;
    }
    
    if (static_cast<size_t>(sent) != msg.length()) {
        _last_error = "Partial send: " + std::to_string(sent) + " of " + std::to_string(msg.length());
        std::cerr << "TCP: " << _last_error << std::endl;
        return false;
    }
    
    return true;
}

std::string TCPClient::receiveJSON(int timeout_ms) {
    if (!_connected) {
        return "";
    }
    
    return receiveUntilNewline(timeout_ms);
}

std::string TCPClient::receiveUntilNewline(int timeout_ms) {
    // Check if we already have a complete message in buffer
    size_t newline_pos = _recv_buffer.find('\n');
    if (newline_pos != std::string::npos) {
        std::string msg = _recv_buffer.substr(0, newline_pos);
        _recv_buffer.erase(0, newline_pos + 1);
        return msg;
    }
    
    // Poll for incoming data
    struct pollfd pfd;
    pfd.fd = _socket_fd;
    pfd.events = POLLIN;
    
    int ret = poll(&pfd, 1, timeout_ms);
    
    if (ret < 0) {
        _last_error = "Poll error: " + std::string(strerror(errno));
        return "";
    }
    
    if (ret == 0) {
        // Timeout - no data available
        return "";
    }
    
    // Receive data
    char buffer[4096];
    ssize_t received = recv(_socket_fd, buffer, sizeof(buffer) - 1, 0);
    
    if (received < 0) {
        _last_error = "Receive error: " + std::string(strerror(errno));
        std::cerr << "TCP: " << _last_error << std::endl;
        _connected = false;
        return "";
    }
    
    if (received == 0) {
        // Connection closed by server
        _last_error = "Connection closed by server";
        std::cerr << "TCP: " << _last_error << std::endl;
        _connected = false;
        return "";
    }
    
    // Append to buffer
    buffer[received] = '\0';
    _recv_buffer += buffer;
    
    // Check again for newline
    newline_pos = _recv_buffer.find('\n');
    if (newline_pos != std::string::npos) {
        std::string msg = _recv_buffer.substr(0, newline_pos);
        _recv_buffer.erase(0, newline_pos + 1);
        return msg;
    }
    
    // Incomplete message
    return "";
}

void TCPClient::disconnect() {
    if (_socket_fd >= 0) {
        close(_socket_fd);
        _socket_fd = -1;
    }
    _connected = false;
    _recv_buffer.clear();
    std::cout << "TCP: Disconnected" << std::endl;
}
