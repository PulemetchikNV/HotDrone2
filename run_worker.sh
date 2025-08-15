#!/bin/bash
export DRONE_ID=3
export DRONE_NAME=drone3
export PYTHONPATH=.

echo "Setting script permissions..."
chmod +x scripts/*.sh

echo "Cleaning Python cache..."
./scripts/clean_pycache.sh

echo "Installing/updating Python packages..."
pip3 install -r requirements.txt

echo "Downloading and setting up Stockfish..."
./scripts/download_stockfish.sh

echo "Starting FastAPI worker server..."
python3 drone/parallel_worker_server.py
