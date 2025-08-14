#!/usr/bin/env bash

# Скрипт для настройки и установки зависимостей проекта на Ubuntu.

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Ubuntu Project Setup Script ===${NC}"

# 1. Установка системных зависимостей
echo -e "${YELLOW}--- Installing system dependencies (stockfish, python3-venv) ---${NC}"
sudo apt-get update
sudo apt-get install -y stockfish python3-venv
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Failed to install system dependencies.${NC}"
    exit 1
fi
echo -e "${GREEN}✓ System dependencies installed successfully.${NC}\n"

# 2. Создание и активация виртуального окружения
VENV_DIR="myvenv"
PYTHON_CMD="python3"

if [ ! -d "$VENV_DIR" ]; then
    echo -e "${YELLOW}--- Creating Python virtual environment in '$VENV_DIR' ---${NC}"
    $PYTHON_CMD -m venv $VENV_DIR
    if [ $? -ne 0 ]; then
        echo -e "${RED}Error: Failed to create virtual environment.${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ Virtual environment created.${NC}\n"
else
    echo -e "${YELLOW}--- Virtual environment '$VENV_DIR' already exists. Skipping creation. ---${NC}\n"
fi

echo -e "${YELLOW}--- Activating virtual environment ---${NC}"
source $VENV_DIR/bin/activate
echo -e "${GREEN}✓ Virtual environment activated.${NC}\n"

# 3. Установка Python-библиотек
REQUIREMENTS_FILE="requirements.txt"

if [ ! -f "$REQUIREMENTS_FILE" ]; then
    echo -e "${RED}Error: $REQUIREMENTS_FILE not found!${NC}"
    exit 1
fi

echo -e "${YELLOW}--- Installing Python requirements from $REQUIREMENTS_FILE ---${NC}"
pip install --upgrade pip
pip install -r $REQUIREMENTS_FILE
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Failed to install Python requirements.${NC}"
    deactivate
    exit 1
fi
echo -e "${GREEN}✓ Python requirements installed successfully.${NC}\n"

echo -e "${GREEN}=== Setup finished successfully! ===${NC}"
echo -e "To activate the virtual environment in your shell, run:"
echo -e "${YELLOW}source $VENV_DIR/bin/activate${NC}"
