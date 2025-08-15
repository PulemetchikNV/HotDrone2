#!/usr/bin/env bash

# SCRIPT_VERSION=v4_final

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

echo -e "${YELLOW}=== Starting workers on all drones (v4_final) ===${NC}"

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
    remote_commands="
        echo 'Connecting to $drone_name...';
        cd $DRONE_DIR || { echo 'Directory not found: $DRONE_DIR'; exit 1; };
        
        echo 'Stopping old worker process (if any)...';
        pkill -f 'drone.run_worker' || echo 'Old process not found, continuing.';
        sleep 1;

        echo 'Starting new worker in background on port $WORKER_PORT...';
        export DRONE_NAME='$drone_name';
        
        source myvenv/bin/activate;
        nohup python3 -m drone.run_worker --host 0.0.0.0 --port $WORKER_PORT > /dev/null 2>&1 &
        deactivate;
        
        sleep 2; # Give it time to start
        
        # Check if the process has started
        if pgrep -f 'drone.run_worker' > /dev/null; then
            echo '✓ Worker started successfully on $drone_name.';
        else
            echo '✗ Failed to start worker on $drone_name.';
        fi
    "
    
    # Execute commands via SSH
    if command -v sshpass &> /dev/null; then
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$remote_commands"
    else
        echo -e "${YELLOW}Warning: sshpass not found. You may need to enter the password manually.${NC}"
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$remote_commands"
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
    
    start_worker_on_drone "$drone_name" "$drone_ip" &
done < <(tr -d '\r' < "$WORKERS_FILE")

echo "Waiting for all SSH sessions to complete..."
wait

echo -e "${GREEN}=== All workers have been launched. ===${NC}"
