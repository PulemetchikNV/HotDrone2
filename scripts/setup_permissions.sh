#!/usr/bin/env bash

# Move to the project's root directory
cd "$(dirname "$0")/.."

echo "--- Setting up script and binary permissions ---"

# Grant execute permissions to all shell scripts in the scripts/ directory
echo "Setting permissions for shell scripts..."
chmod +x ./scripts/*.sh

# Grant execute permissions to the Stockfish binary
STOCKFISH_PATH="./drone/chess/stockfish/stockfish-ubuntu-x86-64-avx2"
if [ -f "$STOCKFISH_PATH" ]; then
    echo "Setting permissions for Stockfish binary..."
    chmod +x "$STOCKFISH_PATH"
else
    echo "Warning: Stockfish binary not found at $STOCKFISH_PATH"
fi

echo "--- Permissions set successfully. ---"
