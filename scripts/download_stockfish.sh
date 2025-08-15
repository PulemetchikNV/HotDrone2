#!/usr/bin/env bash

# SCRIPT_VERSION=v5
# Enhanced Stockfish download script with multi-architecture support

# Move to the project's root directory
cd "$(dirname "$0")/.."

STOCKFISH_DIR="./drone/chess/stockfish"
FINAL_STOCKFISH_PATH="$STOCKFISH_DIR/stockfish"

echo "--- Installing Stockfish (Multi-Architecture Support) ---"

# Detect architecture
ARCH=$(uname -m)
echo "Detected architecture: $ARCH"

# Function to download Stockfish binary
download_stockfish_binary() {
    local arch=$1
    local url=""
    local filename=""
    
    case $arch in
        "x86_64"|"amd64")
            url="https://github.com/official-stockfish/Stockfish/releases/latest/download/stockfish-ubuntu-x86-64-avx2.tar"
            filename="stockfish-ubuntu-x86-64-avx2"
            ;;
        "aarch64"|"arm64"|"armv8l")
            url="https://github.com/official-stockfish/Stockfish/releases/latest/download/stockfish-android-armv8.tar"
            filename="stockfish-android-armv8"
            ;;
        "armv7l")
            url="https://github.com/official-stockfish/Stockfish/releases/latest/download/stockfish-android-armv7.tar"
            filename="stockfish-android-armv7"
            ;;
        *)
            echo "Unsupported architecture: $arch"
            return 1
            ;;
    esac
    
    echo "Downloading Stockfish for $arch from: $url"
    
    # Create directory
    mkdir -p "$STOCKFISH_DIR"
    cd "$STOCKFISH_DIR"
    
    # Download and extract
    if curl -L -o stockfish.tar "$url"; then
        echo "Download successful, extracting..."
        tar -xf stockfish.tar
        if [ -f "$filename" ]; then
            mv "$filename" stockfish
            chmod +x stockfish
            rm stockfish.tar
            echo "Stockfish binary installed successfully at: $STOCKFISH_DIR/stockfish"
            return 0
        elif [ -d "$filename" ]; then
            # Handle case where tar contains a directory
            if [ -f "$filename/$filename" ]; then
                mv "$filename/$filename" stockfish
                chmod +x stockfish
                rm -rf "$filename"
                rm stockfish.tar
                echo "Stockfish binary installed successfully at: $STOCKFISH_DIR/stockfish"
                return 0
            else
                echo "Error: Binary not found in extracted directory"
                echo "Contents of $filename directory:"
                ls -la "$filename"
                return 1
            fi
        else
            echo "Error: Extracted file not found"
            echo "Contents of directory:"
            ls -la
            return 1
        fi
    else
        echo "Error: Failed to download Stockfish"
        return 1
    fi
}

# Function to install via package manager
install_via_package_manager() {
    echo "Attempting to install Stockfish via package manager..."
    
    # Check for sudo privileges
    if [[ $EUID -ne 0 ]]; then
        echo "This script must be run with sudo privileges to install packages."
        echo "Attempting to run with sudo..."
        sudo "$0" "$@"
        exit $?
    fi
    
    # Update package list and install Stockfish
    echo "Updating package list..."
    apt-get update -y
    echo "Installing Stockfish..."
    apt-get install -y stockfish
    if [ $? -ne 0 ]; then
        echo "Error: Failed to install Stockfish via apt-get."
        return 1
    fi
    
    # Find the installed Stockfish binary
    SYSTEM_STOCKFISH_PATH=""
    POSSIBLE_PATHS=("/usr/games/stockfish" "/usr/bin/stockfish")
    for path in "${POSSIBLE_PATHS[@]}"; do
        if [ -f "$path" ]; then
            SYSTEM_STOCKFISH_PATH="$path"
            break
        fi
    done
    
    if [ -z "$SYSTEM_STOCKFISH_PATH" ]; then
        echo "Error: Could not find Stockfish binary after installation."
        return 1
    fi
    
    echo "Stockfish installed at: $SYSTEM_STOCKFISH_PATH"
    
    # Create a symbolic link to the expected path
    mkdir -p "$STOCKFISH_DIR"
    rm -f "$FINAL_STOCKFISH_PATH"
    echo "Creating symbolic link from $SYSTEM_STOCKFISH_PATH to $FINAL_STOCKFISH_PATH"
    ln -s "$SYSTEM_STOCKFISH_PATH" "$FINAL_STOCKFISH_PATH"
    
    if [ ! -L "$FINAL_STOCKFISH_PATH" ]; then
        echo "Error: Failed to create symbolic link."
        return 1
    fi
    
    echo "Stockfish has been installed and linked successfully!"
    return 0
}

# Main installation logic
echo "Attempting to download Stockfish binary for $ARCH..."

# Try downloading binary first
if download_stockfish_binary "$ARCH"; then
    echo "--- Stockfish binary download successful! ---"
    exit 0
else
    echo "Binary download failed, trying package manager..."
    
    # Fallback to package manager
    if install_via_package_manager; then
        echo "--- Stockfish installation via package manager successful! ---"
        exit 0
    else
        echo "Error: All installation methods failed."
        echo "Please install Stockfish manually and ensure it's available at: $FINAL_STOCKFISH_PATH"
        exit 1
    fi
fi
