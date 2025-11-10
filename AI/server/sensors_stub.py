#!/usr/bin/env python3
"""Sensor read stubs for testing client locally. Replace with real sensor code for microcontroller or Pi."""
import random
import time

try:  # Optional dependency used only for frame stub generation
    from PIL import Image  # type: ignore
    _HAS_PIL = True
except ImportError:  # pragma: no cover - PIL not required for telemetry tests
    Image = None  # type: ignore
    _HAS_PIL = False

def read_all_sensors():
    """Generate representative telemetry for integration testing."""
    panel_voltage = random.uniform(7.0, 8.5)
    panel_current = random.uniform(0.3, 0.8)
    mppt_power = panel_voltage * panel_current
    battery_soc = random.uniform(60.0, 95.0)
    battery_voltage = random.uniform(7.0, 8.4)
    battery_current = random.uniform(-0.8, 0.4)  # negative = charging
    bus_current = random.uniform(0.1, 0.6)
    board_temp = 20 + random.uniform(-2, 6)

    return {
        'acc':[random.uniform(-0.02,0.02) for _ in range(3)],
        'gyro':[random.uniform(-0.01,0.01) for _ in range(3)],
        'mag':[random.uniform(-0.5,0.5) for _ in range(3)],
        'sun':[random.uniform(0,1) for _ in range(3)],
        'temp': board_temp,
        'press': 1000 + random.uniform(-5,5),
        'battery_soc': battery_soc,
        'battery_voltage': battery_voltage,
        'battery_current': battery_current,
        'bus_current': bus_current,
        'panel_voltage': panel_voltage,
        'panel_current': panel_current,
        'mppt_power': mppt_power,
        'mppt_duty': random.uniform(0.55, 0.75)
    }

def capture_frame_stub():
    if not _HAS_PIL:
        # Fallback: return static placeholder bytes when Pillow is unavailable
        return b""

    import io

    img = Image.new('RGB', (320, 240), (30, 30, 30))
    bio = io.BytesIO()
    img.save(bio, 'PNG')
    return bio.getvalue()
