#!/usr/bin/env bash

# Скрипт для запуска test_taras.py с использованием виртуального окружения.

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Определяем корневую директорию проекта
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

VENV_DIR="$PROJECT_ROOT/myvenv"
SCRIPT_TO_RUN_NAME="test_taras.py"
SCRIPT_TO_RUN_PATH="$PROJECT_ROOT/$SCRIPT_TO_RUN_NAME"

echo -e "${GREEN}=== Running $SCRIPT_TO_RUN_NAME ===${NC}"

# 1. Проверка и активация виртуального окружения
if [ ! -d "$VENV_DIR" ]; then
    echo -e "${RED}Error: Virtual environment '$VENV_DIR' not found.${NC}"
    echo -e "${YELLOW}Please run './scripts/setup_ubuntu.sh' first to create it.${NC}"
    exit 1
fi

echo -e "${YELLOW}--- Activating virtual environment ---${NC}"
source $VENV_DIR/bin/activate
echo -e "${GREEN}✓ Virtual environment activated.${NC}\n"

# 2. Проверка существования скрипта
if [ ! -f "$SCRIPT_TO_RUN_PATH" ]; then
    echo -e "${RED}Error: Script '$SCRIPT_TO_RUN_NAME' not found at '$SCRIPT_TO_RUN_PATH'!${NC}"
    deactivate
    exit 1
fi

# 3. Запуск скрипта
echo -e "${YELLOW}--- Executing $SCRIPT_TO_RUN_NAME ---${NC}"
python3 "$SCRIPT_TO_RUN_PATH"
echo -e "${GREEN}--- Script execution finished ---${NC}\n"

# Деактивация окружения
deactivate
echo -e "${GREEN}=== All done. ===${NC}"
