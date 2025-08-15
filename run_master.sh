#!/bin/bash
export DRONE_ID=18
export DRONE_NAME=drone18
export PYTHONPATH=.
export WORKER_DRONES="http://192.168.1.3:3000"

echo "Setting script permissions..."
chmod +x scripts/*.sh

echo "Cleaning Python cache..."
./scripts/clean_pycache.sh

echo "Installing/updating Python packages..."
pip3 install -r requirements.txt

echo "Downloading and setting up Stockfish..."
./scripts/download_arm_stockfish.sh

echo "Starting master drone..."
python3 drone/parallel_master.py
