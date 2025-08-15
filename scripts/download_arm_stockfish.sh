#!/usr/bin/env bash

# Move to the project's root directory
cd "$(dirname "$0")/.."

STOCKFISH_DIR="./drone/chess/stockfish"
OLD_STOCKFISH_BINARY="$STOCKFISH_DIR/stockfish-ubuntu-x86-64-avx2"
NEW_STOCKFISH_BINARY_NAME="stockfish" # Новое, простое имя
FINAL_STOCKFISH_PATH="$STOCKFISH_DIR/$NEW_STOCKFISH_BINARY_NAME"
DOWNLOAD_URL="https://stockfishchess.org/files/stockfish_15_linux_armv7.zip"
ZIP_FILE="stockfish.zip"

echo "--- Downloading and installing Stockfish for ARM architecture ---"

# 1. Check for required tools
if ! command -v wget &> /dev/null || ! command -v unzip &> /dev/null; then
    echo "Error: 'wget' and 'unzip' are required. Please install them."
    echo "On Debian/Raspbian: sudo apt-get update && sudo apt-get install wget unzip"
    exit 1
fi

# 2. Remove the old, incompatible binary
if [ -f "$OLD_STOCKFISH_BINARY" ]; then
    echo "Removing old x86-64 Stockfish binary..."
    rm "$OLD_STOCKFISH_BINARY"
fi

# 3. Download the ARM version
echo "Downloading Stockfish for ARMv7 from $DOWNLOAD_URL..."
wget -O "$ZIP_FILE" "$DOWNLOAD_URL"
if [ $? -ne 0 ]; then
    echo "Error: Failed to download Stockfish."
    exit 1
fi

# 4. Unzip the archive
echo "Unzipping the archive..."
unzip -o "$ZIP_FILE" -d "$STOCKFISH_DIR"
if [ $? -ne 0 ]; then
    echo "Error: Failed to unzip the archive."
    rm "$ZIP_FILE"
    exit 1
fi

# 5. The binary is usually in a sub-directory. Find it and move it.
# The binary inside the zip is typically named 'stockfish_15_linux_armv7'
UNZIPPED_BINARY=$(find "$STOCKFISH_DIR" -type f -name "stockfish_*_linux_armv7" | head -n 1)

if [ -f "$UNZIPPED_BINARY" ]; then
    echo "Found unzipped binary at $UNZIPPED_BINARY"
    echo "Moving and renaming to $FINAL_STOCKFISH_PATH"
    mv "$UNZIPPED_BINARY" "$FINAL_STOCKFISH_PATH"
else
    echo "Error: Could not find the Stockfish binary in the unzipped archive."
    rm "$ZIP_FILE"
    exit 1
fi

# 6. Clean up the zip file and any empty directories
rm "$ZIP_FILE"
# Remove the directory that was created by unzip, if it's empty
find "$STOCKFISH_DIR" -type d -empty -delete

# 7. Set execute permissions
echo "Setting execute permissions for the new binary..."
chmod +x "$FINAL_STOCKFISH_PATH"

echo "--- Stockfish for ARM has been installed successfully! ---"
