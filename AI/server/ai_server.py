#!/usr/bin/env python3
"""AI Server
============

This service receives CubeSat telemetry over TCP, executes AI workloads
(navigation inference, obstacle detection, anomaly monitoring) and responds with
sub-second guidance plus corrective actions that can be applied immediately at
the electronics-control layer.
"""

import argparse
import json
import socket
import threading
import time
from collections import deque
from pathlib import Path

from navigation_inference import infer_navigation, infer_obstacle_from_frame

try:  # Optional heavy dependency – fall back to heuristics if unavailable
    from joblib import load  # type: ignore
except Exception:  # pragma: no cover - joblib not mandatory on flight unit
    load = None

HOST = '0.0.0.0'
PORT = 5050
BUFFER = 65536
HISTORY = deque(maxlen=32)

ARTIFACT_DIR = Path(__file__).resolve().parents[1] / "ai_artifacts_updated"
ANOMALY_MODEL_PATH = ARTIFACT_DIR / "anomaly_isolationforest_updated.joblib"

if load and ANOMALY_MODEL_PATH.exists():
    try:
        ANOMALY_MODEL = load(ANOMALY_MODEL_PATH)
    except Exception:  # pragma: no cover - corrupt artifact fallback
        ANOMALY_MODEL = None
else:
    ANOMALY_MODEL = None


def build_feature_vector(sensors):
    """Extract the subset of features expected by the isolation forest model."""
    keys = [
        "panel_voltage",
        "battery_voltage",
        "panel_current",
        "battery_current",
        "mppt_power",
        "battery_soc",
        "bus_current",
        "board_temp",
        "mppt_duty",
    ]
    return [float(sensors.get(k, 0.0)) for k in keys]


def evaluate_hybrid_failure(sensors):
    """Assess anomaly severity and recommended mitigation action."""
    timestamp = time.time()
    HISTORY.append({
        "timestamp": timestamp,
        "battery_voltage": float(sensors.get("battery_voltage", 0.0)),
        "battery_current": float(sensors.get("battery_current", 0.0)),
        "battery_soc": float(sensors.get("battery_soc", 0.0)),
        "bus_current": float(sensors.get("bus_current", 0.0)),
        "panel_voltage": float(sensors.get("panel_voltage", 0.0)),
        "panel_current": float(sensors.get("panel_current", 0.0)),
        "mppt_power": float(sensors.get("mppt_power", 0.0)),
        "board_temp": float(sensors.get("board_temp", sensors.get("temp", 0.0))),
    })

    severity = 0.0
    label = "nominal"
    confidence = 0.5
    action = "none"
    features = build_feature_vector(sensors)

    if ANOMALY_MODEL:
        try:
            # Isolation Forest returns negative scores for anomalies
            decision = float(ANOMALY_MODEL.score_samples([features])[0])
            severity = max(0.0, min(1.0, 1.0 - (decision + 0.5)))
            confidence = 0.6 + 0.4 * severity
            if severity > 0.6:
                label = "model_flagged"
        except Exception:
            severity = 0.0

    latest = HISTORY[-1]
    prev = HISTORY[-2] if len(HISTORY) >= 2 else latest
    dv = latest["battery_voltage"] - prev["battery_voltage"]
    dbus = latest["bus_current"] - prev["bus_current"]

    if latest["battery_soc"] < 25.0 or latest["battery_voltage"] < 6.8:
        severity = max(severity, 0.75)
        label = "deep_discharge"
        action = "shed_noncritical"
    if latest["board_temp"] > 50.0:
        severity = max(severity, 0.7)
        label = "thermal_rise"
        if action == "none":
            action = "balance_thermal"
    if latest["battery_current"] > 0.6 and latest["board_temp"] > 45.0:
        # Hybrid: thermal + power runaway
        severity = max(severity, 0.85)
        label = "thermal_power_hybrid"
        action = "safe_mode_attitude"
    if dv < -0.15 and dbus > 0.2:
        severity = max(severity, 0.8)
        label = "power_bus_surge"
        action = "shed_noncritical"
    if latest["mppt_power"] < 1.5 and latest["panel_voltage"] < 6.5:
        severity = max(severity, 0.65)
        label = "solar_occlusion"
        action = "boost_mppt"

    confidence = min(1.0, max(confidence, 0.55 + severity / 3))
    return {
        "score": round(severity, 3),
        "label": label,
        "confidence": round(confidence, 3),
        "recommended_action": action,
    }

def handle_conn(conn, addr):
    try:
        data = conn.recv(BUFFER)
        if not data:
            return
        msg = json.loads(data.decode())
        # msg may contain 'frame' (base64) or sensor dict
        response = {}
        if 'frame' in msg:
            # obstacle detection from frame (stub or real)
            obj = infer_obstacle_from_frame(msg['frame'])
            response['obstacle'] = obj
        if 'sensors' in msg:
            sensors = msg['sensors']
            anomaly = evaluate_hybrid_failure(sensors)
            corr = infer_navigation(sensors, obstacle_info=response.get('obstacle'))
            response['corrections'] = corr
            response['anomaly'] = anomaly
            response['action'] = anomaly.get('recommended_action', 'none')
            response['timestamp'] = time.time()
        conn.sendall(json.dumps(response).encode())
    except Exception as e:
        err = {'error': str(e)}
        try:
            conn.sendall(json.dumps(err).encode())
        except Exception:
            pass
    finally:
        conn.close()

def start_server(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    s.listen(5)
    print(f"AI server listening on {host}:{port}")
    try:
        while True:
            conn, addr = s.accept()
            t = threading.Thread(target=handle_conn, args=(conn, addr), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print('Shutting down server.')
    finally:
        s.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default=HOST)
    parser.add_argument('--port', type=int, default=PORT)
    args = parser.parse_args()
    start_server(args.host, args.port)
