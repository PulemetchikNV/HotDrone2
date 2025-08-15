#!/bin/bash
export DRONE_ID=18
export DRONE_NAME=drone18
export PYTHONPATH=.
export WORKER_DRONES="http://192.168.1.3:3000"

echo "Setting script permissions..."
chmod +x scripts/*.sh

echo "Cleaning Python cache..."
./scripts/clean_pycache.sh

echo "Force-reinstalling Python packages..."
pip3 uninstall -y python-chess chess
pip3 install -r requirements.txt

echo "Searching for conflicting chess.py files..."
find . -name "chess.py"

echo "Downloading and setting up Stockfish..."
./scripts/download_stockfish.sh

echo "Starting master drone..."
python3 drone/parallel_master.py
