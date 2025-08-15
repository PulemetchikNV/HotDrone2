#!/bin/bash
export DRONE_ID=18
export DRONE_NAME=drone18
export PYTHONPATH=.
export WORKER_DRONES="http://192.168.1.3:3000"

echo "Making Stockfish executable..."
chmod +x drone/chess/stockfish/stockfish

echo "Starting master drone..."
python3 drone/parallel_master.py
