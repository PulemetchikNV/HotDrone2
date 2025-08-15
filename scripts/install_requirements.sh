#!/usr/bin/env bash

# --- Configuration ---
SSH_USER="pi"
SSH_PASS="raspberry"
DRONE_DIR="/home/pi/HotDrone2"
DRONES_FILE="./drones.txt"

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Move to the project's root directory
cd "$(dirname "$0")/.."

echo -e "${YELLOW}=== Rebuilding venv and installing requirements on all drones ===${NC}"

if [ ! -f "$DRONES_FILE" ]; then
    echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
    exit 1
fi

# --- Function to execute the rebuild script on a drone ---
rebuild_on_drone() {
    local drone_name=$1
    local drone_ip=$2
    
    echo -e "${YELLOW}--- Processing $drone_name ($drone_ip) ---${NC}"
    
    # Commands to execute on the remote drone
    local remote_commands="
        echo 'Connecting to $drone_name...';
        cd $DRONE_DIR || { echo 'Directory not found: $DRONE_DIR'; exit 1; };
        
        echo 'Making rebuild script executable...';
        chmod +x ./scripts/rebuild_venv.sh;

        echo 'Executing rebuild script...';
        ./scripts/rebuild_venv.sh;
    "
    
    # Execute commands via SSH
    if command -v sshpass &> /dev/null; then
        sshpass -p "$SSH_PASS" ssh -t -o StrictHostKeyChecking=no -o ConnectTimeout=60 "$SSH_USER@$drone_ip" "$remote_commands"
    else
        echo -e "${YELLOW}Warning: sshpass not found. You may need to enter the password manually.${NC}"
        ssh -t -o StrictHostKeyChecking=no -o ConnectTimeout=60 "$SSH_USER@$drone_ip" "$remote_commands"
    fi

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Successfully completed for $drone_name${NC}"
    else
        echo -e "${RED}✗ Failed on $drone_name${NC}"
    fi
    echo ""
}

# --- Main loop ---
while IFS=':' read -r drone_name drone_ip || [[ -n "$drone_name" ]]; do
    # Trim whitespace and semicolons
    drone_name=$(echo "$drone_name" | xargs)
    drone_ip=$(echo "$drone_ip" | sed 's/;//' | xargs)
    
    if [[ -z "$drone_name" || -z "$drone_ip" ]]; then
        continue
    fi
    
    rebuild_on_drone "$drone_name" "$drone_ip"
done < <(tr -d '\r' < "$DRONES_FILE")

echo -e "${GREEN}=== Script finished. ===${NC}"
