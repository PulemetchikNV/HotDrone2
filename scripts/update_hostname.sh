#!/bin/bash

# Скрипт: обновление hostname на выбранном дроне по SSH и правка локального drones.txt
# Требования: ssh, (опционально) sshpass для безынтерактивного ввода пароля

set -e

# --- Конфигурация по умолчанию ---
SSH_USER=${SSH_USER:-pi}
SSH_PASS=${SSH_PASS:-raspberry}
DRONES_FILE=${DRONES_FILE:-./drones.txt}

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

if [ ! -f "$DRONES_FILE" ]; then
  echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
  exit 1
fi

DRONES_DATA=$(cat "$DRONES_FILE")
IFS=';' read -ra DRONE_ENTRIES <<< "$DRONES_DATA"

valid_entries=()
for entry in "${DRONE_ENTRIES[@]}"; do
  IFS=':' read -r dn dip <<< "$entry"
  dn=$(echo "$dn" | xargs)
  dip=$(echo "$dip" | xargs)
  if [[ -n "$dn" && -n "$dip" ]]; then
    valid_entries+=("$dn:$dip")
  fi
done

if [ ${#valid_entries[@]} -eq 0 ]; then
  echo -e "${RED}Error: No valid entries in $DRONES_FILE${NC}"
  exit 1
fi

echo -e "${GREEN}=== Update drone hostname ===${NC}"
echo "Found ${#valid_entries[@]} drones in $DRONES_FILE"

for i in "${!valid_entries[@]}"; do
  IFS=':' read -r dn dip <<< "${valid_entries[$i]}"
  printf "%2d) %-12s -> %s\n" "$((i+1))" "$dn" "$dip"
done

read -p "Select drone by number: " sel
if ! [[ "$sel" =~ ^[0-9]+$ ]] || [ "$sel" -lt 1 ] || [ "$sel" -gt ${#valid_entries[@]} ]; then
  echo -e "${RED}Invalid selection${NC}"
  exit 1
fi

idx=$((sel-1))
IFS=':' read -r DRONE_NAME DRONE_ADDR <<< "${valid_entries[$idx]}"

DEFAULT_NEW_HOST="$DRONE_NAME"
read -p "New hostname for $DRONE_NAME (default: $DEFAULT_NEW_HOST): " NEW_HOST
NEW_HOST=${NEW_HOST:-$DEFAULT_NEW_HOST}

echo -e "${YELLOW}Target: ${DRONE_NAME} (${DRONE_ADDR}) -> hostname '${NEW_HOST}'${NC}"
read -p "Proceed with SSH update and reboot? (y/N): " confirm
confirm=${confirm:-n}
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
  echo "Aborted"
  exit 0
fi

echo -e "${BLUE}Connecting to $DRONE_ADDR as $SSH_USER ...${NC}"

# Используем sshpass если доступен
SSH_BASE=(ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10)
if command -v sshpass >/dev/null 2>&1; then
  SSH_BASE=(sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10)
fi

# Выполняем скрипт на удалённой машине: backup, запись hostname, правка /etc/hosts, hostnamectl, reboot
${SSH_BASE[@]} "$SSH_USER@$DRONE_ADDR" "NEW_HOST='$NEW_HOST' PASS='$SSH_PASS' bash -s" <<'EOS'
set -e
echo "Connected to $(hostname)"

# Определяем префикс для sudo (если нужен пароль)
SUDO="sudo"
if [ -n "$PASS" ]; then
  if sudo -n true 2>/dev/null; then
    SUDO="sudo"
  else
    SUDO="echo \"$PASS\" | sudo -S"
  fi
fi

eval $SUDO cp -f /etc/hostname /etc/hostname.bak || true
eval $SUDO cp -f /etc/hosts /etc/hosts.bak || true

echo "$NEW_HOST" | eval $SUDO tee /etc/hostname >/dev/null

if grep -qE '^127\.0\.1\.1' /etc/hosts; then
  eval $SUDO sed -i "s/^127\\.0\\.1\\.1.*/127.0.1.1    $NEW_HOST/" /etc/hosts
else
  echo -e "127.0.1.1\t$NEW_HOST" | eval $SUDO tee -a /etc/hosts >/dev/null
fi

# Попробуем применить сразу
eval $SUDO hostnamectl set-hostname "$NEW_HOST" || true
echo "Hostname set to '$NEW_HOST'. Rebooting in 2s..."
sleep 2
eval $SUDO reboot || true
EOS

echo -e "${GREEN}✓ Remote update command sent. Device will reboot.${NC}"

# Обновляем локальный drones.txt: для выбранного дрона ставим .local
NEW_ADDR="$NEW_HOST.local"
rebuilt=""
for entry in "${valid_entries[@]}"; do
  IFS=':' read -r dn dip <<< "$entry"
  if [ "$dn" = "$DRONE_NAME" ]; then
    rebuilt+="$dn:$NEW_ADDR;"
  else
    rebuilt+="$dn:$dip;"
  fi
done

echo "$rebuilt" > "$DRONES_FILE"
echo -e "${GREEN}✓ Updated $DRONES_FILE -> ${DRONE_NAME}:$NEW_ADDR${NC}"

echo -e "${GREEN}Done.${NC}"



