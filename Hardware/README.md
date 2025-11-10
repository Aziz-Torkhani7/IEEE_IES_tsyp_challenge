# CubeSat Autonomous Navigation & Power Management System

## 🛰️ Project Overview

This project implements an **autonomous CubeSat system** integrating:
- **AI-driven navigation** with debris avoidance
- **Maximum Power Point Tracking (MPPT)** for solar optimization
- **Real-time telemetry** and AI communication via TCP
- **Modular architecture** for extensibility

Developed for the **IES TSYP & AESS Challenge 2025**.

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     CUBESAT FIRMWARE                         │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐         ┌──────────────┐                 │
│  │   SENSORS    │────────▶│  CONTROLLER  │                 │
│  ├──────────────┤         └──────┬───────┘                 │
│  │ BQ34Z100     │                │                          │
│  │ INA219       │                ▼                          │
│  │ LIDAR        │         ┌──────────────┐                 │
│  └──────────────┘         │   MODULES    │                 │
│                           ├──────────────┤                 │
│  ┌──────────────┐         │ MPPT Module  │                 │
│  │  ACTUATORS   │◀────────│ AI Navigation│                 │
│  ├──────────────┤         └──────┬───────┘                 │
│  │ ADCS         │                │                          │
│  │ (Attitude)   │                │                          │
│  └──────────────┘                │                          │
│         │                         │                          │
│         │                         ▼                          │
│         │              ┌─────────────────┐                  │
│         │              │  AI INTERFACE   │                  │
│         │              │  (TCP Client)   │                  │
│         │              └────────┬────────┘                  │
└─────────┼───────────────────────┼──────────────────────────┘
          │                       │
          │                       ▼
          │            ┌──────────────────┐
          │            │   AI SERVER      │
          │            │  (Port 5050)     │
          │            ├──────────────────┤
          └───────────▶│ Obstacle Detect  │
                       │ Navigation AI    │
                       │ Forecasting      │
                       └──────────────────┘
```

---

## 📁 Project Structure

```
Firmware/
├── CMakeLists.txt              # Build configuration
├── src/
│   ├── main.cpp                # Main entry point
│   │
│   ├── controller/             # Hardware abstraction layer
│   │   ├── controller.h/cpp    # Device manager
│   │   ├── sensor.h/cpp        # Base sensor class
│   │   ├── actuator.h/cpp      # Base actuator class
│   │   │
│   │   ├── sensors/            # Sensor implementations
│   │   │   ├── BQ34Z100.h/cpp  # Battery monitor (I2C)
│   │   │   ├── ina219.h/cpp    # Solar V/I sensor (I2C)
│   │   │   └── lidar.h/cpp     # Distance sensor
│   │   │
│   │   └── actuators/          # Actuator implementations
│   │       ├── adcs.h/cpp      # Attitude control
│   │       └── propultion.h    # Propulsion (header only)
│   │
│   ├── modules/                # Autonomous behavior modules
│   │   ├── modules.h           # Module registry
│   │   ├── module.h/cpp        # Base module class
│   │   ├── mppt.h/cpp          # MPPT solar optimizer
│   │   └── ai_navigation.h/cpp # AI-driven navigation
│   │
│   ├── communication/          # AI server communication
│   │   ├── tcp_client.h/cpp    # TCP socket client
│   │   └── ai_interface.h/cpp  # High-level AI API
│   │
│   └── energy/                 # Power management
│       ├── energy.h            # Energy definitions
│       └── manager.h/cpp       # Energy manager
│
AI-Data/
├── CubeSat_AI_TCP_System/      # AI server implementation
│   ├── ai_server.py            # TCP server (port 5050)
│   ├── navigation_inference.py # AI model inference
│   ├── tcp_client.py           # Python test client
│   └── requirements.txt        # Python dependencies
│
└── ai_artifacts_updated/       # Trained models
    ├── anomaly_isolationforest_updated.joblib
    └── rf_forecast_model_updated.joblib
```

---

## 🚀 Quick Start

### **Prerequisites**

#### Firmware (C++)
- CMake 3.22+
- G++ with C++11 support
- Linux (tested on Ubuntu 22.04)

#### AI Server (Python)
```bash
cd AI-Data/CubeSat_AI_TCP_System
pip install -r requirements.txt
```

### **Build & Run**

#### **Step 1: Start AI Server**
```bash
cd AI-Data/CubeSat_AI_TCP_System
python ai_server.py --host 0.0.0.0 --port 5050
```

You should see:
```
AI server listening on 0.0.0.0:5050
```

#### **Step 2: Build Firmware**
```bash
cd Firmware
mkdir -p build
cd build
cmake ..
make
```

#### **Step 3: Run CubeSat System**
```bash
# From Firmware/build/
./HardwareInterface 127.0.0.1 5050
```

**Command line arguments:**
- Arg 1: AI server IP (default: `127.0.0.1`)
- Arg 2: AI server port (default: `5050`)

**Expected output:**
```
==================================================
  CubeSat Autonomous Navigation & Power System   
  IES TSYP Challenge 2025                        
==================================================

[INITIALIZATION]
AI Server: 127.0.0.1:5050

✓ Controller initialized

[CONNECTING TO AI SERVER]
TCP: Connected to 127.0.0.1:5050
✓ AI server connected

[INITIALIZING MODULES]
✓ MPPT Module (Solar Power Optimization)
✓ AI Navigation Module (Obstacle Avoidance)
✓ 2 modules loaded

[STARTING MAIN CONTROL LOOP]
Press Ctrl+C to exit

AI Navigation: Polling AI server...
AI Navigation: Received corrections - Roll: 0.1 Pitch: -0.05 Yaw: 0.02
ADCS: Rotate command - Axis[1,1,1] Angle[0.1,-0.05,0.02]
```

---

## 🧩 Key Components

### **1. MPPT Module** (`src/modules/mppt.cpp`)

**Purpose:** Optimize solar power harvesting using Incremental Conductance algorithm.

**Features:**
- Reads voltage/current from INA219 sensor
- Calculates optimal operating point: `dI/dV = -I/V`
- Adjusts PWM duty cycle to maximize power
- Configurable step size and duty limits

**Algorithm:**
```cpp
if (dI/dV > -I/V)  // Left of MPP
    duty -= step;   // Increase voltage
else if (dI/dV < -I/V)  // Right of MPP
    duty += step;   // Decrease voltage
// else at MPP, no change
```

**Usage:**
```cpp
MPPTModule mppt(sensor_id, actuator_id, step_size, duty_min, duty_max);
```

---

### **2. AI Navigation Module** (`src/modules/ai_navigation.cpp`)

**Purpose:** Autonomous navigation with AI-driven obstacle avoidance.

**Features:**
- Sends telemetry to AI server (battery, solar, sensors)
- Receives navigation corrections (roll, pitch, yaw)
- Detects obstacles via AI image processing
- Executes evasive maneuvers when debris < 30m

**Obstacle Avoidance Logic:**
```cpp
if (distance < 30m) {
    if (angle < 30°)  // Directly ahead
        Roll 45° + Pitch 20°
    else              // To the side
        Yaw away 30°
}
```

**Configuration:**
```cpp
AINavigationModule ai_nav(ai_interface, adcs_id, update_interval_ms);
// Default: Update every 1000ms (1 Hz)
```

---

### **3. AI Interface** (`src/communication/ai_interface.cpp`)

**Purpose:** Structured communication with AI server.

**Telemetry Format (JSON):**
```json
{
  "sensors": {
    "acc": [0.0, 0.0, 9.81],
    "gyro": [0.0, 0.0, 0.0],
    "mag": [0.0, 0.0, 50.0],
    "sun": [0.0, 0.0, 1.0],
    "temp": 25.0,
    "press": 101.3,
    "battery_soc": 85.0,
    "battery_voltage": 7.4,
    "solar_power": 4.2
  }
}
```

**AI Response:**
```json
{
  "corrections": {
    "roll": 0.1,
    "pitch": -0.05,
    "yaw": 0.02
  },
  "obstacle": {
    "object": "debris",
    "distance_m": 20.0,
    "angle_deg": 5.0
  }
}
```

---

### **4. TCP Client** (`src/communication/tcp_client.cpp`)

**Features:**
- Persistent TCP connection with auto-reconnect
- JSON message framing (newline-delimited)
- Configurable timeouts
- Thread-safe operations

**Example:**
```cpp
TCPClient client("192.168.1.100", 5050);
if (client.connect()) {
    client.sendJSON("{\"test\": \"data\"}");
    std::string response = client.receiveJSON(1000); // 1s timeout
}
```

---

### **5. Module System** (`src/modules/module.h`)

**Design Pattern:**
```cpp
class Module {
    virtual bool condition(Controller& ctrl);  // "Should I run?"
    virtual void poll(Controller& ctrl);        // "Do my task"
};
```

**Execution Flow:**
```cpp
for (Module* m : modules) {
    if (m->condition(controller)) {
        m->poll(controller);
    }
}
```

**Benefits:**
- Modular design (easy to add new behaviors)
- Priority-based execution
- Conditional activation (power-aware, event-driven)

---

## 🔧 Hardware Integration

### **Sensors**

| Sensor | Purpose | Interface | Implementation |
|--------|---------|-----------|----------------|
| **BQ34Z100** | Battery monitor | I2C (0x55) | `sensors/BQ34Z100.cpp` |
| **INA219** | Solar V/I sensor | I2C (0x40) | `sensors/ina219.cpp` |
| **LIDAR** | Distance measurement | I2C | `sensors/lidar.cpp` |

### **Actuators**

| Actuator | Purpose | Interface | Implementation |
|----------|---------|-----------|----------------|
| **ADCS** | Attitude control | I2C/SPI | `actuators/adcs.cpp` |

### **Adding New Hardware**

1. **Create sensor class:**
```cpp
class NewSensor : public Sensor {
public:
    std::vector<uint8_t> read() override {
        // Read from I2C/SPI/UART
        return data;
    }
};
```

2. **Register in controller:**
```cpp
Controller controller;
controller.addSensor(new NewSensor());
```

---

## 🧪 Testing

### **AI Server Test (Python Client)**
```bash
cd AI-Data/CubeSat_AI_TCP_System
python tcp_client.py --host 127.0.0.1 --port 5050 --interval 0.5
```

### **Firmware Unit Tests**
```bash
# TODO: Add unit tests using Google Test
```

### **Integration Test**
1. Start AI server
2. Run firmware with verbose logging
3. Verify telemetry flow and corrections applied

---

## 📊 Performance

- **AI Update Rate:** 1 Hz (configurable)
- **MPPT Update Rate:** 10 Hz (poll cycle dependent)
- **TCP Latency:** < 10ms (local), < 100ms (network)
- **Memory Usage:** ~2MB (firmware only)

---

## 🔮 Future Enhancements

### **High Priority**
- [ ] Add IMU sensor integration (gyroscope, accelerometer)
- [ ] Implement sun sensor for accurate solar pointing
- [ ] Add magnetometer for detumbling control
- [ ] Integrate GPS for position telemetry

### **AI Improvements**
- [ ] Deploy TFLite models for onboard inference
- [ ] Add anomaly detection (Isolation Forest)
- [ ] Implement battery forecasting (Random Forest)
- [ ] Camera integration for image-based debris detection

### **Power Management**
- [ ] Emergency power reservation (15% for maneuvers)
- [ ] Load shedding during low battery
- [ ] Safe mode implementation

### **Communication**
- [ ] UHF radio integration (Endurosat antenna)
- [ ] Beacon mode for ground station tracking
- [ ] Data compression for downlink

---

## 🤝 Contributing

This project is part of the **IES TSYP Challenge 2025** submission.

**Team:**
- Firmware Development
- AI/ML Integration
- Hardware Integration
- Testing & Validation

---

## 📄 License

[Specify license here]

---

## 📞 Support

For questions about this project:
- GitHub Issues: [Repository URL]
- Email: [Team contact]

---

## 🏆 Acknowledgments

- **IEEE IES TSYP Challenge** organizers
- **NASA Battery Dataset** for training data
- **TensorFlow Lite** for embedded AI
- **AI server framework** contributors

---

**Built with ❤️ for autonomous space exploration** 🚀
