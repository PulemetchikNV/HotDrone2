#!/usr/bin/env bash

# Скрипт для быстрой настройки .env файлов для дронов
# Использование: ./scripts/setup_env.sh [drone_name]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

declare -a NAMES
declare -a HOSTS

function load_drones() {
    local drones_file="$PROJECT_ROOT/drones.txt"
    if [[ ! -f "$drones_file" ]]; then
        echo "❌ Файл со списком дронов не найден: $drones_file"
        return 1
    fi
    local content
    content="$(cat "$drones_file")"
    NAMES=()
    HOSTS=()
    IFS=';' read -ra entries <<< "$content"
    for entry in "${entries[@]}"; do
        [[ -z "$entry" ]] && continue
        local name="${entry%%:*}"
        local host="${entry#*:}"
        if [[ -n "$name" && -n "$host" ]]; then
            NAMES+=("$name")
            HOSTS+=("$host")
        fi
    done
    if [[ ${#NAMES[@]} -eq 0 ]]; then
        echo "❌ Не удалось распарсить ни одного дрона из $drones_file"
        return 1
    fi
    return 0
}

function show_help() {
    echo "Использование: $0 [drone_name]"
    echo ""
    echo "Доступные дроны (из drones.txt):"
    if [[ ${#NAMES[@]} -gt 0 ]]; then
        for i in "${!NAMES[@]}"; do
            echo "  ${NAMES[$i]}  - ${HOSTS[$i]}"
        done
    else
        echo "  (список пуст)"
    fi
    echo ""
    echo "Без параметров - показать интерактивное меню"
    echo ""
    echo "Примеры:"
    local example_drone
    example_drone="${NAMES[0]:-drone_name}"
    echo "  $0 $example_drone    # Настроить .env для $example_drone"
    echo "  $0                   # Интерактивное меню"
}

function setup_drone_env() {
    local drone_name="$1"
    local env_example_file="$PROJECT_ROOT/env.${drone_name}.example"
    local env_file="$PROJECT_ROOT/.env"
    
    # Фолбэк: если нет специализированного примера, используем общий env.example
    if [[ ! -f "$env_example_file" ]]; then
        if [[ -f "$PROJECT_ROOT/env.example" ]]; then
            env_example_file="$PROJECT_ROOT/env.example"
        else
            echo "❌ Пример env-файла не найден: $env_example_file"
            return 1
        fi
    fi
    
    echo "📋 Настройка .env для $drone_name..."
    
    # Создаём резервную копию, если .env уже существует
    if [[ -f "$env_file" ]]; then
        backup_file="$env_file.backup.$(date +%Y%m%d_%H%M%S)"
        echo "📦 Создаём резервную копию: $backup_file"
        cp "$env_file" "$backup_file"
    fi
    
    # Копируем пример в .env
    cp "$env_example_file" "$env_file"
    echo "✅ .env файл создан для $drone_name"
    
    # Показываем содержимое
    echo ""
    echo "📄 Содержимое .env файла:"
    echo "=========================="
    cat "$env_file"
    echo "=========================="
    echo ""
    echo "🚀 Готово! Теперь можно запускать дрон без export команд."
    echo "   Переменные будут автоматически загружены из .env файла."
}

function interactive_menu() {
    echo "🤖 Интерактивная настройка .env для дронов"
    echo "==========================================="
    echo ""
    echo "Выберите дрон:"
    local i
    for i in "${!NAMES[@]}"; do
        local idx=$((i+1))
        echo "$idx) ${NAMES[$i]}  (${HOSTS[$i]})"
    done
    local show_idx=$(( ${#NAMES[@]} + 1 ))
    local exit_idx=$(( ${#NAMES[@]} + 2 ))
    echo "$show_idx) Показать текущий .env"
    echo "$exit_idx) Выход"
    echo ""
    
    read -p "Ваш выбор (1-$exit_idx): " choice
    
    if [[ "$choice" =~ ^[0-9]+$ ]]; then
        if (( choice >= 1 && choice <= ${#NAMES[@]} )); then
            local sel_idx=$((choice-1))
            setup_drone_env "${NAMES[$sel_idx]}"
            return
        elif (( choice == show_idx )); then
            local env_file="$PROJECT_ROOT/.env"
            if [[ -f "$env_file" ]]; then
                echo "📄 Текущий .env файл:"
                echo "===================="
                cat "$env_file"
                echo "===================="
            else
                echo "❌ .env файл не найден"
            fi
            return
        elif (( choice == exit_idx )); then
            echo "👋 До свидания!"
            exit 0
        fi
    fi
    echo "❌ Неверный выбор. Попробуйте снова."
    interactive_menu
}

# Проверяем аргументы командной строки
if ! load_drones; then
    exit 1
fi

if [[ $# -eq 0 ]]; then
    # Интерактивный режим
    interactive_menu
elif [[ $# -eq 1 ]]; then
    if [[ "$1" == "-h" || "$1" == "--help" ]]; then
        show_help
    else
        # Проверяем, что дрон есть в списке
        found=0
        for i in "${!NAMES[@]}"; do
            if [[ "${NAMES[$i]}" == "$1" ]]; then
                found=1
                break
            fi
        done
        if [[ $found -eq 1 ]]; then
            setup_drone_env "$1"
        else
            echo "❌ Дрон '$1' не найден в $PROJECT_ROOT/drones.txt"
            show_help
            exit 1
        fi
    fi
else
    echo "❌ Слишком много аргументов"
    show_help
    exit 1
fi
