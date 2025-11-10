CubeSat AI TCP System - README
=================================

This package contains a reference TCP-based integration for a CubeSat AI system.
It includes:
 - ai_server.py            : AI server that listens for telemetry and returns corrections
 - tcp_client.py           : CubeSat-side client that sends sensors + frame to AI server
 - navigation_inference.py : Inference utilities (TFLite or fallback rule-based)
 - obstacle_detection_stub.py : Placeholder for image-based detector
 - sensors_stub.py         : Sensor read stubs for local testing
 - requirements.txt        : Python dependencies
 - run.sh                  : quick demo run script

USAGE (local test):
1. Install requirements: pip install -r requirements.txt
2. Run server: python ai_server.py
3. In another terminal run client: python tcp_client.py --host 127.0.0.1 --port 5050

Deployment:
 - Place TFLite models (navigation_ai_model_fp32.tflite or navigation_ai_model_int8.tflite)
   and norm_meta_nav.json into this folder to enable real model inference.
 - Replace sensors_stub.py with actual sensor reading code on your CubeSat board.
 - Replace obstacle_detection_stub with a real YOLO/TFLite detector for camera frames.

Notes:
 - TCP JSON messages are simple and human-readable; secure and robust transport (TLS, retransmit) should be added for real missions.
 - This repo is a starting point for the TSYP IES & AESS challenge deliverables.
