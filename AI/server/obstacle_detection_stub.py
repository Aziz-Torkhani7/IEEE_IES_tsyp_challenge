#!/usr/bin/env python3
"""Simple obstacle detection stub: placeholder for YOLO/TFLite model.
Replace with real detector that takes an image and returns bbox + distance."""
import base64, io
from PIL import Image
import numpy as np

def detect_from_bytes(img_bytes):
    try:
        img = Image.open(io.BytesIO(img_bytes)).convert('RGB')
        w,h = img.size
    except Exception:
        return []
    return [{'label':'debris','bbox':[w//4,h//4,3*w//4,3*h//4],'score':0.9,'distance_m':25.0}]

if __name__ == '__main__':
    print('Obstacle detection stub ready.')
