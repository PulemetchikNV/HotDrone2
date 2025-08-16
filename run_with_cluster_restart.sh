#!/bin/bash

# Скрипт-обертка для автоматического перезапуска процесса при изменении кластера
# Использование: ./run_with_cluster_restart.sh [args для main.py]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_SCRIPT="$SCRIPT_DIR/drone/main.py"
RESTART_FLAG_FILE="$SCRIPT_DIR/.restart_required"
MAX_RESTARTS=10
RESTART_COUNT=0

# Функция логирования
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Функция проверки exit кода
check_exit_code() {
    local exit_code=$1
    case $exit_code in
        0)
            log "Process ended normally"
            return 0
            ;;
        42)
            log "Process requested cluster restart (exit code 42)"
            return 1
            ;;
        *)
            log "Process ended with error code $exit_code"
            return 2
            ;;
    esac
}

# Функция проверки флага перезапуска
check_restart_flag() {
    if [ -f "$RESTART_FLAG_FILE" ]; then
        local restart_reason
        restart_reason=$(python3 -c "
import json
try:
    with open('$RESTART_FLAG_FILE', 'r') as f:
        data = json.load(f)
        print(data.get('reason', 'unknown'))
except:
    print('file_exists')
" 2>/dev/null)
        
        log "Restart flag detected: $restart_reason"
        rm -f "$RESTART_FLAG_FILE"
        return 0
    fi
    return 1
}

# Функция ожидания с прогресс-баром
wait_with_progress() {
    local seconds=$1
    local reason=${2:-"restart"}
    
    log "Waiting ${seconds}s for $reason..."
    
    for ((i=1; i<=seconds; i++)); do
        local progress=$((i * 50 / seconds))
        local bar=""
        for ((j=0; j<progress; j++)); do
            bar="${bar}#"
        done
        for ((j=progress; j<50; j++)); do
            bar="${bar}."
        done
        printf "\r[%s] %d/%ds" "$bar" "$i" "$seconds"
        sleep 1
    done
    printf "\n"
}

# Функция проверки зависимостей
check_dependencies() {
    if [ ! -f "$MAIN_SCRIPT" ]; then
        log "ERROR: Main script not found: $MAIN_SCRIPT"
        exit 1
    fi
    
    if ! command -v python3 &> /dev/null; then
        log "ERROR: python3 not found"
        exit 1
    fi
    
    # Проверяем наличие mpirun для кластерного режима
    if [ "${ALG_MODE:-api}" = "cluster" ] && ! command -v mpirun &> /dev/null; then
        log "WARNING: mpirun not found - cluster mode may not work"
    fi
}

# Функция настройки окружения для кластера
setup_cluster_env() {
    # Создаем cluster_hosts файл если его нет
    local cluster_hosts_file="${CLUSTER_HOSTFILE:-cluster_hosts}"
    if [ ! -f "$cluster_hosts_file" ] && [ "${ALG_MODE:-api}" = "cluster" ]; then
        log "Creating initial cluster_hosts file"
        echo "# Initial cluster hosts - will be updated automatically" > "$cluster_hosts_file"
        echo "localhost" >> "$cluster_hosts_file"
    fi
}

# Основная функция запуска
run_main_process() {
    local args=("$@")
    
    log "Starting main process: python3 $MAIN_SCRIPT ${args[*]}"
    log "Environment: ALG_MODE=${ALG_MODE:-api}, DRONE_NAME=${DRONE_NAME:-unknown}"
    
    # Запускаем процесс и ждем завершения
    python3 "$MAIN_SCRIPT" "${args[@]}"
    return $?
}

# Функция уборки при завершении
cleanup() {
    log "Cleanup: removing restart flag and temp files"
    rm -f "$RESTART_FLAG_FILE"
    
    # Убиваем дочерние процессы если есть
    jobs -p | xargs -r kill 2>/dev/null
}

# Обработчик сигналов
signal_handler() {
    log "Received signal - shutting down gracefully"
    cleanup
    exit 0
}

# Главная логика
main() {
    local args=("$@")
    
    log "=== Cluster-aware process wrapper started ==="
    log "Script directory: $SCRIPT_DIR"
    log "Max restarts allowed: $MAX_RESTARTS"
    
    # Настраиваем обработчики сигналов
    trap signal_handler SIGINT SIGTERM
    
    # Проверяем зависимости
    check_dependencies
    
    # Настраиваем окружение кластера
    setup_cluster_env
    
    while [ $RESTART_COUNT -lt $MAX_RESTARTS ]; do
        log "=== Starting attempt $((RESTART_COUNT + 1))/$MAX_RESTARTS ==="
        
        # Запускаем главный процесс
        run_main_process "${args[@]}"
        local exit_code=$?
        
        RESTART_COUNT=$((RESTART_COUNT + 1))
        
        # Проверяем причину завершения
        check_exit_code $exit_code
        local exit_status=$?
        
        case $exit_status in
            0)
                # Нормальное завершение
                if check_restart_flag; then
                    log "Restart flag found - restarting process"
                    wait_with_progress 3 "restart delay"
                    RESTART_COUNT=$((RESTART_COUNT - 1))  # Не считаем как рестарт
                    continue
                else
                    log "Process ended normally without restart request"
                    break
                fi
                ;;
            1)
                # Запрошен перезапуск кластера
                log "Cluster restart requested"
                if check_restart_flag; then
                    wait_with_progress 5 "cluster reconfiguration"
                else
                    wait_with_progress 2 "restart delay"
                fi
                RESTART_COUNT=$((RESTART_COUNT - 1))  # Не считаем как рестарт
                continue
                ;;
            2)
                # Ошибка
                log "Process failed - waiting before restart"
                wait_with_progress 5 "error recovery"
                continue
                ;;
        esac
    done
    
    if [ $RESTART_COUNT -ge $MAX_RESTARTS ]; then
        log "ERROR: Maximum restart count ($MAX_RESTARTS) reached"
        cleanup
        exit 1
    fi
    
    log "=== Process wrapper ended successfully ==="
    cleanup
    exit 0
}

# Запускаем с переданными аргументами
main "$@"
