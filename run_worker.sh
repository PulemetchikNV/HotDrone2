#!/bin/bash
export DRONE_ID=3
export DRONE_NAME=drone3
export PYTHONPATH=.

echo "Making Stockfish executable..."
chmod +x drone/chess/stockfish/stockfish

echo "Starting FastAPI worker server..."
python3 drone/parallel_worker_server.py
