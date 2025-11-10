#!/usr/bin/env python3
"""Navigation inference utilities: loads TFLite models (if present) and runs inference.
This file contains simple fallbacks so the package runs without models installed.
Place your tflite models in the same folder and update MODEL_* paths.
"""
import os, json, numpy as np
from pathlib import Path
BASE = Path(__file__).resolve().parents[0]
MODEL_NAV_FP32 = BASE / 'navigation_ai_model_fp32.tflite'
MODEL_NAV_INT8 = BASE / 'navigation_ai_model_int8.tflite'
META = BASE / 'norm_meta_nav.json'

def infer_obstacle_from_frame(frame_b64):
    # frame_b64: base64-encoded bytes of an image; here we return a stub detection
    return {'object': 'debris', 'distance_m': 20.0, 'angle_deg': 0.0}

def infer_navigation(sensors, obstacle_info=None):
    try:
        import tensorflow as tf
        from tensorflow.lite.python.interpreter import Interpreter
    except Exception:
        acc = sensors.get('acc',[0,0,0])
        gyro = sensors.get('gyro',[0,0,0])
        corr_roll = -0.1 * gyro[0]
        corr_pitch = -0.1 * gyro[1]
        corr_yaw = -0.05 * gyro[2]
        if obstacle_info and obstacle_info.get('distance_m',999) < 10:
            corr_yaw += 0.5
        return {'roll': corr_roll, 'pitch': corr_pitch, 'yaw': corr_yaw}
    nav_model_path = MODEL_NAV_INT8 if MODEL_NAV_INT8.exists() else MODEL_NAV_FP32 if MODEL_NAV_FP32.exists() else None
    if nav_model_path is None:
        return {'roll':0,'pitch':0,'yaw':0}
    meta = json.load(open(META)) if META.exists() else None
    interp = Interpreter(str(nav_model_path))
    interp.allocate_tensors()
    id_in = interp.get_input_details()[0]['index']
    id_out = interp.get_output_details()[0]['index']
    x = np.array([*(sensors.get('acc',[0,0,0])),
                  *(sensors.get('gyro',[0,0,0])),
                  *(sensors.get('mag',[0,0,0])),
                  *(sensors.get('sun',[0,0,0])),
                  sensors.get('temp',0.0),
                  sensors.get('press',0.0)], dtype=np.float32)
    if meta:
        mean = np.array(meta['mean'],dtype=np.float32)
        std = np.array(meta['std'],dtype=np.float32)+1e-9
        seq_len = meta.get('seq_len',10)
        window = np.tile(x, (seq_len,1))
        wnd = (window - mean)/std
    else:
        wnd = np.tile(x, (10,1)).astype(np.float32)
    inp = np.expand_dims(wnd,axis=0).astype(np.float32)
    interp.set_tensor(id_in, inp)
    interp.invoke()
    out = interp.get_tensor(id_out).flatten()
    return {'roll': float(out[0]), 'pitch': float(out[1]), 'yaw': float(out[2])}
