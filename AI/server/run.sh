#!/bin/bash
echo "Start AI server in background..."
python3 ai_server.py &
sleep 1
echo "Start TCP client..."
python3 tcp_client.py --host 127.0.0.1 --port 5050
