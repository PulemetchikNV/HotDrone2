#!/bin/bash
export DRONE_ID=3
export DRONE_NAME=drone3
export PYTHONPATH=.

echo "Downloading and setting up Stockfish..."
./scripts/download_arm_stockfish.sh

echo "Starting FastAPI worker server..."
python3 drone/parallel_worker_server.py
