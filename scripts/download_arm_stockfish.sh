#!/usr/bin/env bash

# SCRIPT_VERSION=v4

# Move to the project's root directory
cd "$(dirname "$0")/.."

STOCKFISH_DIR="./drone/chess/stockfish"
FINAL_STOCKFISH_PATH="$STOCKFISH_DIR/stockfish"

echo "--- Installing Stockfish using APT (v4) ---"

# 1. Check for sudo privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run with sudo privileges to install packages."
   echo "Attempting to run with sudo..."
   sudo "$0" "$@"
   exit $?
fi

# 2. Update package list and install Stockfish
echo "Updating package list..."
apt-get update -y
echo "Installing Stockfish..."
apt-get install -y stockfish
if [ $? -ne 0 ]; then
    echo "Error: Failed to install Stockfish via apt-get."
    exit 1
fi

# 3. Find the installed Stockfish binary
SYSTEM_STOCKFISH_PATH=$(which stockfish)
if [ -z "$SYSTEM_STOCKFISH_PATH" ]; then
    echo "Error: Could not find Stockfish binary after installation."
    exit 1
fi
echo "Stockfish installed at: $SYSTEM_STOCKFISH_PATH"

# 4. Create a symbolic link to the expected path
mkdir -p "$STOCKFISH_DIR"
# Remove the old file/link if it exists
rm -f "$FINAL_STOCKFISH_PATH"
echo "Creating symbolic link from $SYSTEM_STOCKFISH_PATH to $FINAL_STOCKFISH_PATH"
ln -s "$SYSTEM_STOCKFISH_PATH" "$FINAL_STOCKFISH_PATH"

# 5. Verify the link and permissions
if [ ! -L "$FINAL_STOCKFISH_PATH" ]; then
    echo "Error: Failed to create symbolic link."
    exit 1
fi

echo "--- Stockfish has been installed and linked successfully! ---"
