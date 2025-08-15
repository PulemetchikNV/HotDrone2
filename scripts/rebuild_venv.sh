#!/usr/bin/env bash

echo "--- Rebuilding Python virtual environment ---"

# Move to the project's root directory
cd "$(dirname "$0")/.."

echo "Current directory: $(pwd)"

VENV_DIR="myvenv"

if [ -d "$VENV_DIR" ]; then
    echo "Removing existing virtual environment: $VENV_DIR"
    rm -rf "$VENV_DIR"
    if [ $? -ne 0 ]; then
        echo "Error: Failed to remove the existing virtual environment."
        exit 1
    fi
fi

echo "Creating new virtual environment..."
python3 -m venv "$VENV_DIR"
if [ $? -ne 0 ]; then
    echo "Error: Failed to create the virtual environment."
    exit 1
fi

echo "Activating virtual environment and installing requirements..."
source "$VENV_DIR/bin/activate"
pip install -r requirements.txt
if [ $? -ne 0 ]; then
    echo "Error: Failed to install requirements."
    deactivate
    exit 1
fi

deactivate
echo "--- Virtual environment has been rebuilt successfully. ---"
