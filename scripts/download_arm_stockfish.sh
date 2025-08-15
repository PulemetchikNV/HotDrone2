#!/usr/bin/env bash

# SCRIPT_VERSION=v2

# Move to the project's root directory
cd "$(dirname "$0")/.."

STOCKFISH_DIR="./drone/chess/stockfish"
OLD_STOCKFISH_BINARY="$STOCKFISH_DIR/stockfish-ubuntu-x86-64-avx2"
FINAL_STOCKFISH_PATH="$STOCKFISH_DIR/stockfish"
# New, direct download link to the binary
DOWNLOAD_URL="https://github.com/d-ronin/Stockfish-armv7/releases/download/15.1/stockfish_15.1_linux_armv7"

echo "--- Downloading and installing Stockfish for ARM architecture (v2) ---"

# 1. Check for wget
if ! command -v wget &> /dev/null; then
    echo "Error: 'wget' is required. Please install it."
    echo "On Debian/Raspbian: sudo apt-get update && sudo apt-get install wget"
    exit 1
fi

# 2. Create directory if it doesn't exist
mkdir -p "$STOCKFISH_DIR"

# 3. Remove the old, incompatible binary
if [ -f "$OLD_STOCKFISH_BINARY" ]; then
    echo "Removing old x86-64 Stockfish binary..."
    rm "$OLD_STOCKFISH_BINARY"
fi

# 4. Download the ARM binary directly
echo "Downloading Stockfish for ARM from $DOWNLOAD_URL..."
wget -O "$FINAL_STOCKFISH_PATH" "$DOWNLOAD_URL"
if [ $? -ne 0 ]; then
    echo "Error: Failed to download Stockfish."
    exit 1
fi

# 5. Set execute permissions
echo "Setting execute permissions for the new binary..."
chmod +x "$FINAL_STOCKFISH_PATH"

echo "--- Stockfish for ARM has been installed successfully! ---"
