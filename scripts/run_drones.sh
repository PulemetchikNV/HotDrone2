#!/usr/bin/env bash

# Переходим в корневую директорию проекта
cd "$(dirname "$0")/.."

# Скрипт для запуска основного скрипта на всех дронах
# Использование: ./scripts/run_drones.sh [script_name]
# Примеры:
#   ./scripts/run_drones.sh main.py
#   ./scripts/run_drones.sh main_stage1_mod.py

# Конфигурация SSH
SSH_USER="pi"
SSH_PASS="raspberry"  # Или ваш пароль
DRONE_DIR="/home/pi/HotDrone2"
DRONES_FILE="./drones.txt"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Определяем какой скрипт запускать
SCRIPT_NAME=${1:-"main.py"}
#SCRIPT_NAME=${1:-"example.py"}
PYTHON_CMD="python3"
LOG_SERVER_PID=""

echo -e "${GREEN}=== Drone Run Script ===${NC}"
echo "Script to run: $SCRIPT_NAME"
echo "Reading drones from: $DRONES_FILE"

# Функция для запуска log server
start_log_server() {
    echo -e "${BLUE}Starting log server...${NC}"
    
    # Проверяем есть ли уже запущенный log server
    if pgrep -f "log_server.py" > /dev/null; then
        echo -e "${YELLOW}Log server already running${NC}"
        return 0
    fi
    
    # Запускаем log server в фоне
    if [ -f "./drone/log_server.py" ]; then
        $PYTHON_CMD ./drone/log_server.py &
        LOG_SERVER_PID=$!
        sleep 1
        
        # Проверяем что запустился
        if kill -0 $LOG_SERVER_PID 2>/dev/null; then
            echo -e "${GREEN}✓ Log server started (PID: $LOG_SERVER_PID)${NC}"
        else
            echo -e "${RED}✗ Failed to start log server${NC}"
            LOG_SERVER_PID=""
        fi
    else
        echo -e "${YELLOW}Warning: log_server.py not found, continuing without log server${NC}"
    fi
    echo ""
}

# Функция для остановки log server
stop_log_server() {
    if [ -n "$LOG_SERVER_PID" ] && kill -0 $LOG_SERVER_PID 2>/dev/null; then
        echo -e "${BLUE}Stopping log server (PID: $LOG_SERVER_PID)...${NC}"
        kill $LOG_SERVER_PID
        sleep 1
        echo -e "${GREEN}✓ Log server stopped${NC}"
    fi
}

# Функция для обработки сигналов (Ctrl+C)
cleanup() {
    echo -e "\n${YELLOW}Received interrupt signal, cleaning up...${NC}"
    stop_log_server
    exit 0
}

# Устанавливаем обработчик сигналов
trap cleanup SIGINT SIGTERM

# Проверяем существование файла со списком дронов
if [ ! -f "$DRONES_FILE" ]; then
    echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
    exit 1
fi

# Запускаем log server
start_log_server

# Функция для запуска скрипта на дроне через SSH
run_on_drone() {
    local drone_name=$1
    local drone_ip=$2
    local is_leader=$3
    local worker_urls=$4
    
    echo -e "${YELLOW}--- Starting $drone_name ($drone_ip) ---${NC}"
    
    # Команды для выполнения на дроне
    local commands="
        echo 'Connected to $drone_name';
        cd $DRONE_DIR || { echo 'Directory not found: $DRONE_DIR'; exit 1; };

        echo 'Current directory:' \$(pwd);
        echo 'Setting DRONE_NAME environment variable to: $drone_name';
        export DRONE_NAME='$drone_name';
        
        if [ \"$is_leader\" = "true" ]; then
            echo 'This is the LEADER drone. Setting WORKER_DRONES...';
            export WORKER_DRONES='$worker_urls';
            echo 'WORKER_DRONES set to: $WORKER_DRONES';
        fi

        echo 'Starting script: $SCRIPT_NAME';
        local module_name=\$(echo "$SCRIPT_NAME" | sed 's/\.py//');
        echo "Command: DRONE_NAME=$drone_name $PYTHON_CMD -m drone.\$module_name";
        echo '--- Script Output ---';
        export DRONE_NAME='$drone_name';
        $PYTHON_CMD -m drone.\$module_name;
        
        echo '--- Script Finished ---';
        echo 'Execution completed for $drone_name';
    "
    
    # Выполняем команды через SSH в фоне
    if command -v sshpass &> /dev/null; then
        # Используем sshpass если доступен
        echo -e "${BLUE}Running on $drone_name in background...${NC}"
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands" &
    else
        # Обычный SSH (потребует ввода пароля)
        echo -e "${YELLOW}Note: sshpass not found, running without password automation${NC}"
        echo -e "${BLUE}Running on $drone_name in background...${NC}"
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands" &
    fi
    
    # Сохраняем PID процесса
    local ssh_pid=$!
    echo -e "${GREEN}✓ Started $drone_name (PID: $ssh_pid)${NC}"
    echo ""
}

# Функция для синхронного запуска (по одному)
run_on_drone_sync() {
    local drone_name=$1
    local drone_ip=$2
    local is_leader=$3
    local worker_urls=$4

    echo -e "${YELLOW}--- Running on $drone_name ($drone_ip) ---${NC}"
    
    # Команды для выполнения на дроне
    local commands="
        echo 'Connected to $drone_name';
        cd $DRONE_DIR || { echo 'Directory not found: $DRONE_DIR'; exit 1; };

        # Source ROS environment. This is critical for non-interactive SSH sessions.
        if [ -f /opt/ros/noetic/setup.bash ]; then
            source /opt/ros/noetic/setup.bash
            echo 'ROS environment sourced.';
        else
            echo 'Warning: ROS setup script not found. Proceeding without it.';
        fi

        echo 'Current directory:' \$(pwd);
        echo 'Setting DRONE_NAME environment variable to: $drone_name';
        export DRONE_NAME='$drone_name';

        if [ \"$is_leader\" = "true" ]; then
            echo 'This is the LEADER drone. Setting WORKER_DRONES...';
            export WORKER_DRONES='$worker_urls';
            echo 'WORKER_DRONES set to: $WORKER_DRONES';
        fi

        echo 'Starting script: $SCRIPT_NAME';
        local module_name=\$(echo "$SCRIPT_NAME" | sed 's/\.py//');
        echo "Command: DRONE_NAME=$drone_name $PYTHON_CMD -m drone.\$module_name";
        echo '--- Script Output ---';
        DRONE_NAME='$drone_name' $PYTHON_CMD -m drone.\$module_name;
        local exit_code=\$?;
        echo '--- Script Finished (exit code: '\$exit_code') ---';
        echo 'Execution completed for $drone_name';
        exit \$exit_code;
    "
    
    # Выполняем команды через SSH синхронно
    if command -v sshpass &> /dev/null; then
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    else
        echo -e "${YELLOW}Note: sshpass not found, you'll need to enter password manually${NC}"
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    fi
    
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}✓ Successfully completed $drone_name${NC}"
    else
        echo -e "${RED}✗ Failed on $drone_name (exit code: $exit_code)${NC}"
    fi
    echo ""
}

# Собираем URL-ы воркеров
WORKER_URLS=""
if [ -f "./workers.txt" ]; then
    while IFS=':' read -r worker_name worker_ip || [[ -n "$worker_name" ]]; do
        worker_name=$(echo "$worker_name" | xargs)
        worker_ip=$(echo "$worker_ip" | sed 's/;//' | xargs)
        if [[ -n "$worker_name" && -n "$worker_ip" ]]; then
            if [ -n "$WORKER_URLS" ]; then
                WORKER_URLS="$WORKER_URLS,"
            fi
            WORKER_URLS="$WORKER_URLS http://$worker_ip:3000"
        fi
    done < <(tr -d '\r' < "./workers.txt")
    echo -e "${BLUE}Worker URLs for leader: $WORKER_URLS${NC}"
else
    echo -e "${YELLOW}Warning: workers.txt not found. Leader will not have WORKER_DRONES set.${NC}"
fi
echo ""

# Проверяем режим запуска
echo "Choose execution mode:"
echo "1) Parallel (all drones simultaneously)"
echo "2) Sequential (one by one)"
read -p "Enter choice (1 or 2, default: 1): " choice
choice=${choice:-1}

# Определяем лидера (первый дрон в списке)
LEADER_NAME=$(head -n 1 "$DRONES_FILE" | cut -d':' -f1 | xargs)
echo -e "${BLUE}Leader drone identified as: $LEADER_NAME${NC}"

if [ "$choice" = "2" ]; then
    echo -e "${BLUE}Running in sequential mode...${NC}"
    
    # Читаем содержимое файла в переменную
    DRONES_DATA=$(cat "$DRONES_FILE")
    
    # Разбиваем строку по точке с запятой
    IFS=';' read -ra DRONE_ENTRIES <<< "$DRONES_DATA"
    
    # Обрабатываем каждую запись
    for entry in "${DRONE_ENTRIES[@]}"; do
        # Разбиваем запись на имя и IP по двоеточию
        IFS=':' read -r drone_name drone_ip <<< "$entry"
        
        # Пропускаем пустые строки
        if [[ -z "$drone_name" || -z "$drone_ip" ]]; then
            continue
        fi
        
        # Убираем лишние пробелы
        drone_name=$(echo "$drone_name" | xargs)
        drone_ip=$(echo "$drone_ip" | xargs)
        
        is_leader="false"
        if [ "$drone_name" = "$LEADER_NAME" ]; then
            is_leader="true"
        fi
        
        run_on_drone_sync "$drone_name" "$drone_ip" "$is_leader" "$WORKER_URLS"
    done
else
    echo -e "${BLUE}Running in parallel mode...${NC}"
    
    # Читаем содержимое файла в переменную
    DRONES_DATA=$(cat "$DRONES_FILE")
    
    # Разбиваем строку по точке с запятой
    IFS=';' read -ra DRONE_ENTRIES <<< "$DRONES_DATA"
    
    # Обрабатываем каждую запись
    for entry in "${DRONE_ENTRIES[@]}"; do
        # Разбиваем запись на имя и IP по двоеточию
        IFS=':' read -r drone_name drone_ip <<< "$entry"
        
        # Пропускаем пустые строки
        if [[ -z "$drone_name" || -z "$drone_ip" ]]; then
            continue
        fi
        
        # Убираем лишние пробелы
        drone_name=$(echo "$drone_name" | xargs)
        drone_ip=$(echo "$drone_ip" | xargs)
        
        is_leader="false"
        if [ "$drone_name" = "$LEADER_NAME" ]; then
            is_leader="true"
        fi
        
        run_on_drone "$drone_name" "$drone_ip" "$is_leader" "$WORKER_URLS"
    done
    
    echo -e "${GREEN}All drones started. Press Ctrl+C to stop all processes.${NC}"
    echo -e "${YELLOW}Waiting for all background processes to complete...${NC}"
    wait
fi

echo -e "${GREEN}=== Execution completed ===${NC}"

# Останавливаем log server
stop_log_server

# Инструкция по установке sshpass если нужно
if ! command -v sshpass &> /dev/null; then
    echo -e "${YELLOW}Tip: Install sshpass for automatic password handling:${NC}"
    echo "  macOS: brew install hudochenkov/sshpass/sshpass"
    echo "  Ubuntu: sudo apt-get install sshpass"
fi
