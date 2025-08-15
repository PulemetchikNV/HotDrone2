#!/usr/bin/env bash

# SCRIPT_VERSION=v1

# Move to the project's root directory
cd "$(dirname "$0")/.."

STOCKFISH_DIR="./drone/chess/stockfish"
FINAL_STOCKFISH_PATH="$STOCKFISH_DIR/stockfish"
REPO_URL="https://github.com/official-stockfish/Stockfish.git"
TEMP_DIR="stockfish_source"

echo "--- Compiling Stockfish from source for ARM architecture ---"

# 1. Check for required tools
if ! command -v git &> /dev/null || ! command -v make &> /dev/null || ! command -v g++ &> /dev/null; then
    echo "Error: 'git', 'make', and 'g++' are required. Please install them."
    echo "On Debian/Raspbian: sudo apt-get update && sudo apt-get install git make g++"
    exit 1
fi

# 2. Clean up previous attempts
rm -rf "$TEMP_DIR"
rm -f "$FINAL_STOCKFISH_PATH"

# 3. Clone the repository
echo "Cloning Stockfish source code from $REPO_URL..."
git clone --depth 1 "$REPO_URL" "$TEMP_DIR"
if [ $? -ne 0 ]; then
    echo "Error: Failed to clone Stockfish repository."
    exit 1
fi

# 4. Compile the binary
echo "Compiling Stockfish for ARMv7..."
cd "$TEMP_DIR/src"
# We specify the architecture for Raspberry Pi
make build ARCH=armv7
if [ $? -ne 0 ]; then
    echo "Error: Failed to compile Stockfish."
    cd ../..
    rm -rf "$TEMP_DIR"
    exit 1
fi

# 5. Move the compiled binary to the correct location
echo "Moving compiled binary to $FINAL_STOCKFISH_PATH"
mv stockfish "$FINAL_STOCKFISH_PATH"
if [ $? -ne 0 ]; then
    echo "Error: Failed to move the compiled binary."
    cd ../..
    rm -rf "$TEMP_DIR"
    exit 1
fi

# 6. Clean up the source code directory
cd ../..
rm -rf "$TEMP_DIR"

# 7. Set execute permissions (should be set by 'make', but we ensure it)
echo "Setting execute permissions..."
chmod +x "$FINAL_STOCKFISH_PATH"

echo "--- Stockfish has been compiled and installed successfully! ---"
