#!/usr/bin/env bash

# Скрипт для ОСТАНОВКИ FastAPI воркеров на всех дронах.

# Конфигурация
SSH_USER="pi"
SSH_PASS="raspberry"
DRONES_FILE="./drones.txt"

# Цвета
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=== Остановка воркеров на всех дронах ===${NC}"

if [ ! -f "$DRONES_FILE" ]; then
    echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
    exit 1
fi

# Функция для остановки воркера на дроне
stop_worker_on_drone() {
    local drone_name=$1
    local drone_ip=$2
    
    echo -e "${YELLOW}--- Остановка воркера на $drone_name ($drone_ip) ---${NC}"
    
    # Команда для убийства процесса воркера
    local commands="
        echo 'Подключение к $drone_name...';
        echo 'Поиск и остановка процесса воркера...';
        pkill -f 'drone.run_worker' && echo '✓ Процесс воркера остановлен.' || echo 'Процесс воркера не найден.';
    "
    
    # Выполняем команды через SSH
    if command -v sshpass &> /dev/null; then
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    else
        echo -e "${YELLOW}Внимание: sshpass не найден. Возможно, потребуется ввести пароль вручную.${NC}"
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    fi
    echo ""
}

# Читаем дронов из файла и останавливаем воркеры
while IFS=':' read -r drone_name drone_ip || [[ -n "$drone_name" ]]; do
    # Убираем лишние пробелы и символы возврата каретки
    drone_name=$(echo "$drone_name" | xargs)
    drone_ip=$(echo "$drone_ip" | sed 's/;//' | xargs)
    
    if [[ -z "$drone_name" || -z "$drone_ip" ]]; then
        continue
    fi
    
    stop_worker_on_drone "$drone_name" "$drone_ip" &
done < <(tr -d '\r' < "$DRONES_FILE")

echo "Ожидание завершения всех SSH сессий..."
wait

echo -e "${GREEN}=== Все воркеры остановлены. ===${NC}"
