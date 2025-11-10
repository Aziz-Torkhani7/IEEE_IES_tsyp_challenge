# CubeSat AI Server - LIDAR-Based Debris Avoidance
=================================

## What This Does

AI server that receives LIDAR sensor data and telemetry from the CubeSat firmware, then returns:
- **Navigation corrections** (roll, pitch, yaw adjustments)
- **Obstacle detection** (debris location and collision risk)
- **Evasive maneuver commands** (automatic debris avoidance)

---

## Key Features

✅ **LIDAR Debris Detection** - Analyzes distance and angle measurements  
✅ **AI-Driven Decisions** - Neural network predicts optimal evasion paths  
✅ **Real-Time Response** - <100ms from detection to decision  
✅ **Multi-Sensor Fusion** - Combines LIDAR + IMU + battery data  
✅ **Fallback Mode** - Rule-based navigation if AI models unavailable  

---

## Files

- `ai_server.py` - Main TCP server (listens on port 5050)
- `tcp_client.py` - Test client for debugging
- `navigation_inference.py` - TensorFlow Lite AI inference
- `obstacle_detection_stub.py` - LIDAR data processing for debris detection
- `sensors_stub.py` - Simulated sensors for testing
- `requirements.txt` - Python dependencies
- `run.sh` - Quick demo script

---

## Quick Start

### 1. Install Requirements
```bash
pip install -r requirements.txt
```

### 2. Run AI Server
```bash
python ai_server.py
```
**Output:** `AI server listening on 0.0.0.0:5050`

### 3. Test with Client (separate terminal)
```bash
python tcp_client.py --host 127.0.0.1 --port 5050
```

---

## How It Works

### **1. Firmware Sends LIDAR Data**
```json
{
  "sensors": {
    "lidar_distance_m": 25.0,
    "lidar_angle_deg": 15.0,
    "acc": [0.0, 0.0, 9.81],
    "gyro": [0.0, 0.0, 0.0],
    "mag": [0.0, 0.0, 50.0],
    "battery_soc": 85.0,
    "solar_power": 4.2
  }
}
```

### **2. AI Server Analyzes**
- Checks LIDAR distance (debris within 40m?)
- Evaluates approach angle (head-on or side?)
- Runs neural network to predict best evasion
- Calculates collision risk (LOW/MEDIUM/HIGH)

### **3. Server Responds**
```json
{
  "corrections": {"roll": 0.1, "pitch": -0.05, "yaw": 0.02},
  "obstacle": {
    "detected": true,
    "object": "debris",
    "distance_m": 25.0,
    "angle_deg": 15.0,
    "collision_risk": "HIGH"
  }
}
```

### **4. Firmware Executes Evasive Maneuver**
- If distance < 30m → EMERGENCY MODE
- Head-on (angle < 30°) → Roll 45° + Pitch 20°
- Side approach → Yaw 30° away

---

## LIDAR Integration

### **Current:** Stub Implementation
The `obstacle_detection_stub.py` simulates LIDAR readings for testing:
```python
def detect_obstacles(lidar_distance=None):
    if lidar_distance and lidar_distance < 30.0:
        return {
            "detected": True,
            "object": "debris",
            "distance_m": lidar_distance,
            "angle_deg": random.uniform(0, 30),
            "collision_risk": "HIGH" if lidar_distance < 15 else "MEDIUM"
        }
```

### **Future:** Real LIDAR Integration
Replace with actual VL53L0X or similar sensor:
```python
import board
import adafruit_vl53l0x

i2c = board.I2C()
lidar = adafruit_vl53l0x.VL53L0X(i2c)

def detect_obstacles():
    distance_mm = lidar.range
    distance_m = distance_mm / 1000.0
    # ... process and return obstacle info
```

---

## AI Models

### **Navigation AI (Optional TFLite)**
Place these files in the server directory to enable:
- `navigation_ai_model_fp32.tflite` or `navigation_ai_model_int8.tflite`
- `norm_meta_nav.json` (normalization metadata)

**If not present:** Falls back to rule-based corrections

### **Anomaly Detection (Loaded)**
- File: `../models/ai_artifacts_updated/anomaly_isolationforest_updated.joblib`
- Detects: Sensor failures, battery issues
- Used to verify data quality before making decisions

### **Battery Forecasting (Loaded)**
- File: `../models/ai_artifacts_updated/rf_forecast_model_updated.joblib`
- Predicts: Battery state 30 minutes ahead
- Used to ensure sufficient power for evasive maneuvers

---

## Deployment on CubeSat

### **Step 1:** Replace Sensor Stubs
Edit `ai_server.py` to use real hardware:
```python
# Instead of sensors_stub.py
from real_sensor_interface import read_lidar, read_imu
```

### **Step 2:** Install TFLite Models
Copy trained models to server directory:
```bash
cp navigation_model.tflite ./
cp norm_meta_nav.json ./
```

### **Step 3:** Configure Network
Update firmware to connect to onboard server:
```bash
./HardwareInterface localhost 5050  # Same device
# or
./HardwareInterface 192.168.1.100 5050  # Network connection
```

---

## Testing

### **Test 1: Server Startup**
```bash
python ai_server.py
```
Expected: `AI server listening on 0.0.0.0:5050`

### **Test 2: Client Connection**
```bash
python tcp_client.py --host 127.0.0.1 --port 5050
```
Expected: JSON exchange with corrections and obstacle data

### **Test 3: LIDAR Simulation**
Edit `sensors_stub.py` to simulate close debris:
```python
def read_lidar_sim():
    return {"distance_m": 20.0, "angle_deg": 10.0}  # Debris at 20m!
```
Restart server and verify HIGH collision risk response

---

## Configuration

Edit `ai_server.py` settings:
```python
HOST = '0.0.0.0'      # Listen on all network interfaces
PORT = 5050           # TCP port
TIMEOUT = 30          # Connection timeout (seconds)
```

---

## Troubleshooting

### Server won't start
```bash
# Check if port is in use
netstat -an | grep 5050

# Try different port
python ai_server.py --port 5051
```

### Models not loading
```bash
# Verify file paths
ls ../models/ai_artifacts_updated/

# Check Python version (need 3.8+)
python --version
```

### Client can't connect
```bash
# Check firewall
sudo ufw allow 5050/tcp

# Test with localhost first
python tcp_client.py --host 127.0.0.1 --port 5050
```

---

## Performance

| Metric | Value |
|--------|-------|
| Response Time | <100ms |
| Max Clients | 5 concurrent |
| LIDAR Update Rate | 10 Hz supported |
| Memory Usage | ~500 MB with models |

---

## Notes

- TCP JSON protocol is human-readable for debugging
- For production: Add TLS encryption and authentication
- Server is single-threaded but handles multiple clients
- LIDAR stub uses random data - replace with real sensor interface

---

**Repository:** https://github.com/Aziz-Torkhani7/IEEE_IES_tsyp_challenge
