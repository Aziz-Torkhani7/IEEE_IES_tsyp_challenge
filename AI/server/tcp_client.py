#!/usr/bin/env python3
"""TCP client for CubeSat main board: reads sensors (stub), sends to AI server, receives corrections."""
import socket, json, time, argparse, base64
from sensors_stub import read_all_sensors, capture_frame_stub

def send_and_receive(host, port, sensors, frame_b64=None):
    payload = {}
    if sensors is not None:
        payload['sensors'] = sensors
    if frame_b64 is not None:
        payload['frame'] = frame_b64
    data = json.dumps(payload).encode()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(5)
        s.connect((host, port))
        s.sendall(data)
        resp = s.recv(65536)
        return json.loads(resp.decode())

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=5050)
    parser.add_argument('--interval', type=float, default=0.2)
    args = parser.parse_args()
    while True:
        sensors = read_all_sensors()
        frame = capture_frame_stub()  # returns bytes
        frame_b64 = base64.b64encode(frame).decode()
        try:
            out = send_and_receive(args.host, args.port, sensors, frame_b64)
            print('Server response:', out)
        except Exception as e:
            print('Comm error:', e)
        time.sleep(args.interval)
