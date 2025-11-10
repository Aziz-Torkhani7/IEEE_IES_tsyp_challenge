/**
 * @file main.cpp
 * @brief CubeSat Autonomous System Main Entry Point
 * 
 * Integrates all subsystems:
 * - Controller (sensors + actuators)
 * - Energy Manager (battery monitoring, power distribution)
 * - MPPT Module (solar power optimization)
 * - AI Navigation Module (autonomous navigation + debris avoidance)
 * 
 * System Architecture:
 * 1. Initialize hardware (sensors, actuators)
 * 2. Connect to AI server for autonomous navigation
 * 3. Enter main control loop
 *    a. Energy manager monitors battery
 *    b. MPPT optimizes solar power
 *    c. AI navigation applies corrections and avoids obstacles
 * 
 * @author CubeSat Autonomy Team
 * @date 2025-11-10
 */

#include "controller/controller.h"
#include "energy/manager.h"
#include "modules/modules.h"
#include "modules/mppt.h"
#include "modules/ai_navigation.h"
#include "communication/ai_interface.h"
#include <iostream>
#include <vector>
#include <memory>
#include <unistd.h>

int main(int argc, char *argv[]) {
  std::cout << "==================================================" << std::endl;
  std::cout << "  CubeSat Autonomous Navigation & Power System   " << std::endl;
  std::cout << "  IES TSYP Challenge 2025                        " << std::endl;
  std::cout << "==================================================" << std::endl;
  
  // Parse command line arguments
  std::string ai_host = "127.0.0.1";  // Default: localhost
  uint16_t ai_port = 5050;             // Default: matches ai_server.py
  
  if (argc >= 2) {
    ai_host = argv[1];
  }
  if (argc >= 3) {
    ai_port = static_cast<uint16_t>(std::stoi(argv[2]));
  }
  
  std::cout << "\n[INITIALIZATION]" << std::endl;
  std::cout << "AI Server: " << ai_host << ":" << ai_port << std::endl;
  
  // Initialize controller with sensors and actuators
  // Note: In production, populate with actual hardware instances
  Controller controller;
  std::cout << "✓ Controller initialized" << std::endl;
  
  // Initialize AI interface
  AIInterface ai_interface(ai_host, ai_port);
  
  std::cout << "\n[CONNECTING TO AI SERVER]" << std::endl;
  if (!ai_interface.initialize()) {
    std::cerr << "⚠ WARNING: AI server not available" << std::endl;
    std::cerr << "   System will run in fallback mode (no AI navigation)" << std::endl;
    std::cerr << "   To enable AI: Start ai_server.py on " << ai_host << ":" << ai_port << std::endl;
  } else {
    std::cout << "✓ AI server connected" << std::endl;
  }
  
  // Initialize modules
  std::cout << "\n[INITIALIZING MODULES]" << std::endl;
  
  // MPPT Module - Solar power optimization
  // Parameters: sensor_id, actuator_id, step_size, duty_min, duty_max
  MPPTModule mppt(0, 0, 0.01, 0.05, 0.95);
  std::cout << "✓ MPPT Module (Solar Power Optimization)" << std::endl;
  
  // AI Navigation Module - Autonomous navigation + debris avoidance
  // Parameters: ai_interface, adcs_id, update_interval_ms
  AINavigationModule ai_nav(ai_interface, 1, 1000);  // Update every 1 second
  std::cout << "✓ AI Navigation Module (Obstacle Avoidance)" << std::endl;
  
  // Build module list (could be extended with more modules)
  std::vector<Module*> modules;
  modules.push_back(&mppt);
  modules.push_back(&ai_nav);
  
  std::cout << "✓ " << modules.size() << " modules loaded" << std::endl;
  
  // Main control loop
  std::cout << "\n[STARTING MAIN CONTROL LOOP]" << std::endl;
  std::cout << "Press Ctrl+C to exit\n" << std::endl;
  
  uint32_t loop_count = 0;
  
  while (true) {
    loop_count++;
    
    // Print status every 10 loops
    if (loop_count % 10 == 0) {
      std::cout << "\n--- Loop " << loop_count << " ---" << std::endl;
      std::cout << "AI Connected: " << (ai_interface.isConnected() ? "YES" : "NO") << std::endl;
      if (ai_nav.isObstacleDetected()) {
        std::cout << "⚠ OBSTACLE DETECTED: " << ai_nav.getObstacleDistance() << "m" << std::endl;
      }
    }
    
    // Execute each module if its condition is met
    for (Module* module : modules) {
      if (module->condition(controller)) {
        module->poll(controller);
      }
    }
    
    // Sleep to reduce CPU usage (100ms between loops)
    usleep(100000);  // 100ms
  }
  
  return 0;
}