#!/usr/bin/env bash

echo "Starting all services..."

# Start the worker drones in the background
./scripts/start_workers.sh

echo "Waiting for workers to initialize..."
sleep 5

# Start the master drone
./run.sh

echo "All services started."
