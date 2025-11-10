# CubeSat Autonomous System - Technical Documentation

## Table of Contents
1. [System Overview](#system-overview)
2. [Architecture](#architecture)
3. [Module Documentation](#module-documentation)
4. [API Reference](#api-reference)
5. [Hardware Integration](#hardware-integration)
6. [AI Integration](#ai-integration)
7. [Build & Deployment](#build--deployment)
8. [Testing](#testing)
9. [Troubleshooting](#troubleshooting)

---

## System Overview

### Mission Objectives
1. **Primary:** Maximize solar energy absorption while maintaining Earth observation capability
2. **Secondary:** Autonomous debris avoidance using AI
3. **Tertiary:** Optimize battery life and predict degradation

### Key Performance Indicators
- **MPPT Efficiency:** >98%
- **AI Response Time:** <100ms
- **Debris Detection Range:** 100m
- **Power Margin:** 15% emergency reserve

---

## Architecture

### Component Hierarchy
```
main()
  └─ Controller
      ├─ Sensors
      │   ├─ BQ34Z100 (I2C 0x55) - Battery monitor
      │   ├─ INA219 (I2C 0x40) - Solar V/I sensor
      │   └─ LIDAR (I2C) - Distance measurement
      │
      └─ Actuators
          └─ ADCS (I2C/SPI) - Attitude control

  └─ Modules
      ├─ MPPT Module
      │   ├─ Reads: INA219 (voltage, current)
      │   ├─ Computes: Incremental conductance algorithm
      │   └─ Outputs: PWM duty cycle
      │
      └─ AI Navigation Module
          ├─ Reads: All sensors via controller
          ├─ Communicates: AI server (TCP 5050)
          ├─ Receives: Roll/pitch/yaw corrections
          └─ Outputs: ADCS commands

  └─ AI Interface
      ├─ TCP Client
      ├─ JSON Serialization
      └─ Response Parsing
```

### Data Flow
```
Sensors → Controller → Modules → Actuators
              ↓
        AI Interface
              ↓
         TCP Client
              ↓
        AI Server (Python)
              ↓
    Navigation Corrections
              ↓
         ADCS Actuator
```

---

## Module Documentation

### MPPT Module

**File:** `src/modules/mppt.cpp`

**Purpose:** Optimize solar power extraction using Incremental Conductance algorithm.

**Parameters:**
```cpp
MPPTModule(
    uint32_t sensorId,      // INA219 sensor ID
    uint32_t actuatorId,    // PWM actuator ID
    double step,            // Step size (0.01 = 1%)
    double dmin,            // Min duty cycle (0.05 = 5%)
    double dmax             // Max duty cycle (0.95 = 95%)
)
```

**Algorithm:**
```cpp
double dV = voltage - prevV;
double dI = current - prevI;

if (|dV| < ε) {
    if (dI > 0) duty -= step;
    else duty += step;
} else {
    double dIdV = dI / dV;
    double negIV = -current / voltage;
    
    if (|dIdV - negIV| < tolerance) {
        // At MPP, no change
    } else if (dIdV > negIV) {
        duty -= step;
    } else {
        duty += step;
    }
}
```

**Configuration Example:**
```cpp
// Conservative tracking (slow but stable)
MPPTModule mppt_slow(0, 0, 0.005, 0.05, 0.95);

// Aggressive tracking (fast but may oscillate)
MPPTModule mppt_fast(0, 0, 0.02, 0.05, 0.95);
```

---

### AI Navigation Module

**File:** `src/modules/ai_navigation.cpp`

**Purpose:** AI-driven autonomous navigation with obstacle avoidance.

**Parameters:**
```cpp
AINavigationModule(
    AIInterface& ai_interface,  // AI communication layer
    uint32_t adcs_id,           // ADCS actuator ID
    uint32_t update_interval_ms // AI query period (1000 = 1Hz)
)
```

**Operation Modes:**

1. **Normal Navigation:**
   - Sends telemetry every `update_interval_ms`
   - Receives small corrections (±5°)
   - Applies to ADCS smoothly

2. **Obstacle Detected (100m-30m):**
   - Prepares evasive maneuver
   - Notifies ground station
   - Continues tracking

3. **Critical Avoidance (<30m):**
   - Executes emergency maneuver
   - Overrides all other modules
   - Maximum correction: ±45° roll, ±30° pitch/yaw

**Avoidance Logic:**
```cpp
if (distance < 30m) {
    if (angle < 30°) {  // Head-on collision course
        roll = 45°;
        pitch = 20°;
    } else {  // Side approach
        yaw = (angle > 0) ? -30° : +30°;
    }
}
```

---

## API Reference

### Controller API

```cpp
class Controller {
public:
    // Get sensor by ID
    Sensor& getSensor(uint32_t id);
    
    // Get actuator by ID
    Actuator& getActuator(uint32_t id);
};
```

**Sensor IDs:**
- `0` - BQ34Z100 (Battery)
- `1` - INA219 (Solar)
- `2` - LIDAR

**Actuator IDs:**
- `0` - PWM (MPPT)
- `1` - ADCS

---

### Module API

```cpp
class Module {
public:
    // Check if module should execute
    virtual bool condition(Controller& controller) = 0;
    
    // Execute module logic
    virtual void poll(Controller& controller) = 0;
};
```

**Implementation Pattern:**
```cpp
class MyModule : public Module {
    bool condition(Controller& ctrl) override {
        // Return true if module should run
        return some_condition_met;
    }
    
    void poll(Controller& ctrl) override {
        // Perform module action
        Sensor& s = ctrl.getSensor(0);
        std::vector<uint8_t> data = s.read();
        // Process data...
    }
};
```

---

### AI Interface API

```cpp
class AIInterface {
public:
    // Initialize connection
    bool initialize();
    
    // Send telemetry and receive corrections
    bool sendTelemetryAndReceive(
        Controller& controller,
        NavigationCorrections& corrections,
        ObstacleInfo& obstacle
    );
    
    // Check connection status
    bool isConnected() const;
};
```

**Data Structures:**
```cpp
struct NavigationCorrections {
    double roll;    // Radians
    double pitch;   // Radians
    double yaw;     // Radians
    bool valid;     // True if received successfully
};

struct ObstacleInfo {
    std::string object_type;  // "debris", "satellite"
    double distance_m;        // Distance in meters
    double angle_deg;         // Angle from boresight
    bool detected;            // True if obstacle present
};
```

---

## Hardware Integration

### BQ34Z100 Battery Monitor

**I2C Address:** `0x55`

**Register Map:**
```cpp
enum class Command : uint8_t {
    StateOfCharge = 0x02,  // % (uint8)
    Voltage = 0x08,        // mV (uint16)
    Temperature = 0x0C,    // 0.1K (uint16)
    Current = 0x10,        // mA (int16)
};
```

**Usage:**
```cpp
BQ34Z100 battery("/dev/i2c-1");
uint8_t soc = battery.getSOC();         // 0-100%
uint16_t voltage = battery.getVoltage(); // mV
int16_t current = battery.getCurrent();  // mA
```

**Read Sequence:**
```
1. I2C Start
2. Write: Slave Address (0x55) + W
3. Write: Register Address (e.g., 0x02 for SoC)
4. I2C Repeated Start
5. Write: Slave Address (0x55) + R
6. Read: Data byte(s)
7. I2C Stop
```

---

### INA219 Solar Sensor

**I2C Address:** `0x40`

**Measurement Range:**
- Voltage: 0-26V (12mV resolution)
- Current: ±3.2A (0.8mA resolution)

**Usage:**
```cpp
INA219 solar("/dev/i2c-1", 0x40);
std::vector<uint8_t> data = solar.read();

// Data format: [V_low, V_high, I_low, I_high, P_low, P_high, status]
uint16_t voltage_mV = data[0] | (data[1] << 8);
int16_t current_mA = data[2] | (data[3] << 8);

double voltage = voltage_mV / 1000.0;  // Volts
double current = current_mA / 1000.0;  // Amps
double power = voltage * current;      // Watts
```

---

### ADCS Actuator

**Interface:** I2C or SPI (hardware-dependent)

**Command Format:**
```cpp
std::vector<uint8_t> axis = {1, 1, 1};  // [x, y, z] enable
std::vector<float> angles = {roll_deg, pitch_deg, yaw_deg};

adcs->rotate(axis, angles);
```

**Internal Implementation:**
```cpp
// Convert to hardware-specific format
// Example for reaction wheels:
uint8_t cmd[] = {
    ROTATE_CMD,
    static_cast<uint8_t>(roll_deg * 10),
    static_cast<uint8_t>(pitch_deg * 10),
    static_cast<uint8_t>(yaw_deg * 10)
};
i2c_write(ADCS_ADDR, cmd, sizeof(cmd));
```

---

## AI Integration

### Telemetry Format (Firmware → AI)

```json
{
  "sensors": {
    "acc": [ax, ay, az],           // Accelerometer (m/s²)
    "gyro": [gx, gy, gz],          // Gyroscope (rad/s)
    "mag": [mx, my, mz],           // Magnetometer (µT)
    "sun": [sx, sy, sz],           // Sun vector (unit)
    "temp": T,                     // Temperature (°C)
    "press": P,                    // Pressure (kPa)
    "battery_soc": SOC,            // State of Charge (%)
    "battery_voltage": V,          // Voltage (V)
    "solar_power": P               // Solar power (W)
  }
}
```

### AI Response Format (AI → Firmware)

```json
{
  "corrections": {
    "roll": 0.05,                  // Radians
    "pitch": -0.02,                // Radians
    "yaw": 0.01                    // Radians
  },
  "obstacle": {
    "object": "debris",            // Type
    "distance_m": 45.2,            // Distance (m)
    "angle_deg": 12.5              // Angle (°)
  }
}
```

### AI Server Extension

To add custom AI models:

```python
# In navigation_inference.py

def infer_custom_model(sensors, model_path):
    import tensorflow as tf
    interpreter = tf.lite.Interpreter(model_path)
    interpreter.allocate_tensors()
    
    # Prepare input
    input_data = preprocess(sensors)
    interpreter.set_tensor(input_idx, input_data)
    
    # Run inference
    interpreter.invoke()
    
    # Extract output
    output = interpreter.get_tensor(output_idx)
    return postprocess(output)
```

---

## Build & Deployment

### Development Build

```bash
cd Firmware
./build.sh build
```

### Production Build (Optimized)

```bash
cd Firmware
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
strip HardwareInterface  # Remove debug symbols
```

### Cross-Compilation (ARM)

```bash
# Install cross-compiler
sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

# Configure CMake
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-toolchain.cmake ..
make
```

### Deployment to CubeSat

```bash
# Via SSH
scp HardwareInterface cubesat@192.168.1.100:/home/cubesat/

# Set permissions
ssh cubesat@192.168.1.100 "chmod +x /home/cubesat/HardwareInterface"

# Run
ssh cubesat@192.168.1.100 "/home/cubesat/HardwareInterface 127.0.0.1 5050"
```

---

## Testing

### Unit Test Template

```cpp
#include <gtest/gtest.h>
#include "modules/mppt.h"

TEST(MPPTTest, InitialDuty) {
    MPPTModule mppt(0, 0, 0.01, 0.05, 0.95);
    EXPECT_EQ(mppt.getDuty(), 0.5);  // Starts at 50%
}

TEST(MPPTTest, DutySaturation) {
    MPPTModule mppt(0, 0, 0.01, 0.05, 0.95);
    // Test code to drive duty beyond limits
    // Verify duty stays within [0.05, 0.95]
}
```

### Integration Test

```bash
# Terminal 1: Start AI server
python AI-Data/CubeSat_AI_TCP_System/ai_server.py

# Terminal 2: Run firmware
./Firmware/build/HardwareInterface 127.0.0.1 5050

# Terminal 3: Monitor logs
tail -f /var/log/cubesat/system.log
```

---

## Troubleshooting

### Issue: "Failed to connect to AI server"

**Solution:**
1. Verify AI server is running: `netstat -an | grep 5050`
2. Check firewall: `sudo ufw allow 5050/tcp`
3. Test connectivity: `telnet 127.0.0.1 5050`

### Issue: "Sensor read failed"

**Solution:**
1. Check I2C bus: `i2cdetect -y 1`
2. Verify sensor address appears in scan
3. Check connections and pull-up resistors

### Issue: "MPPT not tracking optimally"

**Solution:**
1. Reduce step size for stability
2. Increase tolerance for MPP detection
3. Verify sensor readings are accurate

### Issue: "ADCS commands not executing"

**Solution:**
1. Verify ADCS actuator ID is correct
2. Check ADCS hardware interface (I2C/SPI)
3. Enable debug logging in `adcs.cpp`

---

## Performance Tuning

### MPPT Optimization

**For fast-changing conditions (clouds):**
```cpp
MPPTModule mppt(0, 0, 0.02, 0.05, 0.95);  // Larger step
```

**For stable conditions:**
```cpp
MPPTModule mppt(0, 0, 0.005, 0.05, 0.95);  // Smaller step
```

### AI Update Rate

**For high-speed maneuvers:**
```cpp
AINavigationModule ai_nav(ai_interface, 1, 500);  // 2Hz
```

**For power conservation:**
```cpp
AINavigationModule ai_nav(ai_interface, 1, 5000);  // 0.2Hz
```

---

## Safety Considerations

### Power Budget

Always maintain 15% emergency reserve:
```cpp
if (battery_soc < 15.0) {
    // Enter safe mode
    disable_non_critical_systems();
    mppt_module.setMaxDuty(0.5);  // Reduce power draw
}
```

### Debris Avoidance Priority

Debris avoidance overrides all other modules:
```cpp
if (obstacle_detected && distance < 30m) {
    mppt_module.pause();
    camera_module.pause();
    execute_avoidance_maneuver();
}
```

---

**Document Version:** 1.0  
**Last Updated:** November 10, 2025  
**Maintained by:** CubeSat Autonomy Team
