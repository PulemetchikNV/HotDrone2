# Быстрый старт кластерного режима

## 🚀 Установка (на каждом дроне)

```bash
# 1. Установить MPI
sudo apt update && sudo apt install openmpi-bin openmpi-dev

# 2. Скачать и разместить Stockfish бинарники
mkdir -p drone/chess/stockfish
# Поместить stockfish бинарники в drone/chess/stockfish/

# 3. Настроить IP адреса дронов в drone/const.py
# Убедиться что каждый дрон имеет 'raw_ip' в DRONES_CONFIG
```

## ⚡ Запуск

### На master дроне (лидере):
```bash
export ALG_MODE=cluster
export DRONE_NAME=drone21  # имя вашего дрона
./run_with_cluster_restart.sh
```

### На worker дронах:
```bash
export ALG_MODE=api  # или cluster тоже работает
export DRONE_NAME=drone15  # имя вашего дрона
python3 drone/main.py
```

## 🔧 Тестирование

```bash
# Проверить конфигурацию
./test_cluster.sh

# Проверить MPI вручную
mpirun --hostfile cluster_hosts -np 2 hostname

# Проверить ping дронов
python3 -c "
import sys; sys.path.append('drone')
from esp import create_comm_controller
esp = create_comm_controller(None, 'test')
alive = esp.ping_all_drones(['drone21', 'drone15'])
print('Alive drones:', alive)
"
```

## 📊 Мониторинг

```bash
# Текущий кластер
cat cluster_hosts

# Логи перезапуска
cat .restart_required

# Процессы
ps aux | grep python3
```

## 🛠️ Отладка

**Проблема**: MPI не найден
```bash
sudo apt install openmpi-bin openmpi-dev
which mpirun
```

**Проблема**: Stockfish не найден  
```bash
ls -la drone/chess/stockfish/
chmod +x drone/chess/stockfish/*
```

**Проблема**: Ошибка версии Stockfish dev (invalid literal for int)
```bash
# Это происходит с dev-версиями типа "dev20240410b4ac3d6b"
# Наш код автоматически создает UCI wrapper для таких версий
# Если проблема остается:
python3 test_stockfish_dev.py  # проверить работу
```

**Проблема**: Дроны не пингуются
```bash
# Проверить IP в const.py
# Проверить сеть: ping 192.168.1.59
```

**Проблема**: Частые перезапуски
```bash
export CLUSTER_RESTART_DELAY=20  # увеличить задержку
```

## 📋 Переменные окружения

```bash
# Обязательные
export ALG_MODE=cluster
export DRONE_NAME=drone21

# Опциональные
export CLUSTER_HOSTFILE=cluster_hosts
export CLUSTER_NP=4
export CLUSTER_RESTART_DELAY=10
export STOCKFISH_DEPTH=20
export STOCKFISH_SKILL_LEVEL=20
export COMM_IMPL=wifi
```

## 🎯 Быстрая проверка готовности

```bash
./test_cluster.sh && echo "✅ Готов к запуску!" || echo "❌ Нужна настройка"
```

Если все тесты проходят - можно запускать кластерный режим!
