#!/usr/bin/env bash

# SCRIPT_VERSION=v3

# Move to the project's root directory
cd "$(dirname "$0")/.."

# --- Configuration ---
SSH_USER="pi"
SSH_PASS="raspberry"
DRONE_DIR="/home/pi/HotDrone2"
WORKERS_FILE="./workers.txt"
WORKER_PORT=3000

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=== Starting workers on all drones (v3) ===${NC}"

if [ ! -f "$WORKERS_FILE" ]; then
    echo -e "${RED}Error: Workers file not found: $WORKERS_FILE${NC}"
    exit 1
fi

# --- Function to start a worker on a drone ---
start_worker_on_drone() {
    local drone_name=$1
    local drone_ip=$2
    
    echo -e "${YELLOW}--- Starting worker on $drone_name ($drone_ip) ---${NC}"
    
    # Commands to execute on the remote drone
    # IMPORTANT: This is a multi-line string. Do not use 'local' inside it.
    remote_commands="
        echo 'Connecting to $drone_name...';
        cd $DRONE_DIR || { echo 'Directory not found: $DRONE_DIR'; exit 1; };
        
        echo 'Stopping old worker process (if any)...';
        pkill -f 'drone.run_worker' || echo 'Old process not found, continuing.';
        sleep 1;

        echo 'Starting new worker...';
        export DRONE_NAME='$drone_name';
        
        source myvenv/bin/activate;
        
        echo '--- Python script output will follow ---';
        python3 -m drone.run_worker --host 0.0.0.0 --port $WORKER_PORT;
        echo '--- Python script finished ---';

        deactivate;
    "
    
    # Execute commands via SSH
    if command -v sshpass &> /dev/null; then
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=15 "$SSH_USER@$drone_ip" "$remote_commands"
    else
        echo -e "${YELLOW}Warning: sshpass not found. You may need to enter the password manually.${NC}"
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=15 "$SSH_USER@$drone_ip" "$remote_commands"
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
    
    start_worker_on_drone "$drone_name" "$drone_ip"
done < <(tr -d '\r' < "$WORKERS_FILE")

echo -e "${GREEN}=== Worker launch process finished. ===${NC}"
