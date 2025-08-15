#!/usr/bin/env bash

# SCRIPT_VERSION=v2_local

# Move to the project's root directory
cd "$(dirname "$0")/.."

# --- Colors ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=== Rebuilding local venv and installing requirements (v2_local) ===${NC}"

# Make the rebuild script executable
chmod +x ./scripts/rebuild_venv.sh

# Execute the rebuild script locally
./scripts/rebuild_venv.sh

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Successfully completed.${NC}"
else
    echo -e "${RED}✗ Failed to rebuild virtual environment.${NC}"
fi
