#!/usr/bin/env bash

# SCRIPT_VERSION=v1

# Move to the project's root directory
cd "$(dirname "$0")/.."

# --- Colors ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=== Installing requirements globally for the current user (v1) ===${NC}"

# Ensure pip is available
python3 -m ensurepip --user

# Install requirements
python3 -m pip install --user -r requirements.txt

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Successfully installed requirements.${NC}"
    echo "Please add ~/.local/bin to your PATH if it is not already."
    echo "You can do this by adding the following line to your ~/.bashrc or ~/.zshrc:"
    echo 'export PATH="$HOME/.local/bin:$PATH"'
else
    echo -e "${RED}✗ Failed to install requirements.${NC}"
fi
