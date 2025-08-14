#!/usr/bin/env bash

# Скрипт для ЗАПУСКА FastAPI воркеров на всех дронах в фоновом режиме.

# Конфигурация
SSH_USER="pi"
SSH_PASS="raspberry"
DRONE_DIR="/home/pi/HotDrone2"
DRONES_FILE="./workers.txt"
WORKER_PORT=8080

# Цвета
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=== Запуск воркеров на всех дронах ===${NC}"

if [ ! -f "$DRONES_FILE" ]; then
    echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
    exit 1
fi

# Функция для запуска воркера на дроне
start_worker_on_drone() {
    local drone_name=$1
    local drone_ip=$2
    
    echo -e "${YELLOW}--- Запуск воркера на $drone_name ($drone_ip) ---${NC}"
    
    # Команды для выполнения на дроне
    # 1. Убиваем любой старый процесс воркера, чтобы избежать конфликта портов.
    # 2. Запускаем новый воркер с помощью nohup, чтобы он продолжал работать после закрытия SSH.
    # 3. Перенаправляем вывод в /dev/null, чтобы не создавать nohup.out файлы.
    local commands="
        echo 'Подключение к $drone_name...';
        cd $DRONE_DIR || { echo 'Директория не найдена: $DRONE_DIR'; exit 1; };
        
        echo 'Остановка старого процесса воркера (если есть)...';
        pkill -f 'drone.run_worker' || echo 'Старый процесс не найден, все в порядке.';
        sleep 1;

        echo 'Запуск нового воркера в фоновом режиме на порту $WORKER_PORT...';
        export DRONE_NAME='$drone_name';
        export LOG_SERVER_IP='$(hostname -i | awk '{print $1}')'; # Автоматически определяем IP хоста
        
        source myvenv/bin/activate;
        nohup python3 -m drone.run_worker --host 0.0.0.0 --port $WORKER_PORT > /dev/null 2>&1 &
        deactivate;
        
        sleep 2; # Даем время на запуск
        
        # Проверяем, что процесс запустился
        if pgrep -f 'drone.run_worker' > /dev/null; then
            echo '✓ Воркер успешно запущен на $drone_name.';
        else
            echo '✗ Не удалось запустить воркер на $drone_name.';
        fi
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

# Читаем дронов из файла и запускаем воркеры
while IFS=':' read -r drone_name drone_ip || [[ -n "$drone_name" ]]; do
    # Убираем лишние пробелы и символы возврата каретки
    drone_name=$(echo "$drone_name" | xargs)
    drone_ip=$(echo "$drone_ip" | sed 's/;//' | xargs)
    
    if [[ -z "$drone_name" || -z "$drone_ip" ]]; then
        continue
    fi
    
    start_worker_on_drone "$drone_name" "$drone_ip" &
done < <(tr -d '\r' < "$DRONES_FILE")

echo "Ожидание завершения всех SSH сессий..."
wait

echo -e "${GREEN}=== Все воркеры запущены. Теперь вы можете запустить ./check.sh для проверки. ===${NC}"
