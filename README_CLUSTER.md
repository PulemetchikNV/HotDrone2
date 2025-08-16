# Кластерное вычисление с MPI и Stockfish

Этот документ описывает, как настроить и использовать кластерное вычисление для расчета ходов в шахматной игре с использованием MPI и Stockfish.

## Обзор архитектуры

### Как работает кластерное вычисление

1. **MPI кластер**: Используется `mpirun` для запуска Stockfish на нескольких дронах одновременно
2. **Автоматическое управление**: Система автоматически обновляет файл `cluster_hosts` при изменении списка живых дронов
3. **Graceful restart**: При изменении состава кластера лидер уведомляет ведомых и перезапускается
4. **Фолбэк**: Если кластер недоступен, система автоматически переключается на локальный Stockfish

### Компоненты системы

- **`cluster_manager.py`** - управление составом кластера и перезапусками
- **`alg2_stockfish.py`** - интеграция с MPI Stockfish и проверка изменений
- **`esp.py`** - расширена командами уведомления о перезапуске лидера
- **`chess.py`** - минимальные изменения для поддержки кластера
- **`run_with_cluster_restart.sh`** - wrapper-скрипт для автоматического перезапуска

## Установка и настройка

### 1. Установка MPI

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install openmpi-bin openmpi-dev

# Проверка установки
mpirun --version
```

### 2. Настройка дронов в конфигурации

Убедитесь, что в `drone/const.py` у каждого дрона указан `raw_ip`:

```python
DRONES_CONFIG = {
    'drone21': {
        'role': 'king', 
        'color': 'black',
        'initial_letter': 'e',
        'team': 'black',
        'raw_ip': '192.168.1.59'  # IP адрес дрона
    },
    'drone15': {
        'role': 'queen',
        'color': 'black', 
        'initial_letter': 'd',
        'team': 'black',
        'raw_ip': '192.168.1.60'  # IP адрес дрона
    }
    # ... другие дроны
}
```

### 3. Установка Stockfish

Поместите бинарники Stockfish в папку `drone/chess/stockfish/`:

```bash
drone/chess/stockfish/
├── stockfish                    # основной бинарник
├── stockfish-ubuntu-x86-64-avx2 # для x86_64
├── stockfish-android-armv8      # для ARM64
└── stockfish-android-armv7      # для ARM32
```

### 4. Тестирование настройки

```bash
# Проверить конфигурацию
./test_cluster.sh

# Должен показать:
# - Конфигурацию дронов с IP адресами
# - Статус MPI
# - Наличие Stockfish
# - Тест cluster_manager
```

## Использование

### Запуск в кластерном режиме

```bash
# Установить режим работы
export ALG_MODE=cluster

# Запустить с автоматическим перезапуском при изменении кластера
./run_with_cluster_restart.sh

# Или напрямую (без автоперезапуска)
python3 drone/main.py
```

### Переменные окружения

```bash
# Основные настройки
export ALG_MODE=cluster                    # включить кластерный режим
export CLUSTER_HOSTFILE=cluster_hosts      # файл со списком IP дронов
export CLUSTER_NP=4                        # количество процессов (по умолчанию = количество живых дронов)
export CLUSTER_RESTART_DELAY=10            # задержка перезапуска в секундах

# Настройки Stockfish
export STOCKFISH_DEPTH=20                  # глубина анализа
export STOCKFISH_SKILL_LEVEL=20            # уровень игры (1-20)

# Настройки коммуникации
export COMM_IMPL=wifi                      # или esp
export DRONE_NAME=drone21                  # имя текущего дрона
```

## Логика работы

### 1. Обнаружение изменений в кластере

При каждом ходе лидер:
1. Пингует все дроны из `DRONE_LIST`
2. Сравнивает список живых дронов с текущим `cluster_hosts`
3. Если есть изменения - инициирует обновление кластера

### 2. Процесс обновления кластера

```
Лидер обнаружил изменения
    ↓
Уведомляет ведомых о предстоящем перезапуске (ESP broadcast)
    ↓
Обновляет файл cluster_hosts с новыми IP
    ↓
Устанавливает флаг перезапуска
    ↓  
Завершает процесс с кодом 42
    ↓
Wrapper-скрипт перезапускает процесс
    ↓
Новый процесс уведомляет ведомых о завершении перезапуска
    ↓
Продолжается вычисление ходов на обновленном кластере
```

### 3. Команды ESP для координации

- **`leader_restart_notify`** - лидер уведомляет о предстоящем перезапуске
- **`leader_restart_complete`** - лидер уведомляет о завершении перезапуска

### 4. MPI команда для вычисления

```bash
mpirun --hostfile cluster_hosts -map-by node -np 4 stockfish bench
```

Где:
- `cluster_hosts` - файл с IP живых дронов (по одному на строку)
- `-map-by node` - по одному процессу на узел
- `-np 4` - общее количество процессов
- `stockfish` - путь к бинарнику Stockfish

## Файлы кластера

### cluster_hosts
Автоматически генерируемый файл со списком IP живых дронов:
```
# Auto-generated cluster hosts file
# Updated at 2024-01-15 14:30:15
192.168.1.59
192.168.1.60
192.168.1.61
```

### .restart_required
JSON файл с информацией о причине перезапуска:
```json
{
  "reason": "drone_list_changed",
  "timestamp": 1705320615.123,
  "alive_drones": ["drone21", "drone15"],
  "restart_delay": 10
}
```

## Мониторинг и отладка

### Логи
Все компоненты логируют свою активность:
- **cluster_manager**: обновления кластера, перезапуски
- **alg2_stockfish**: кластерные вычисления, fallback
- **esp**: уведомления о перезапуске между дронами
- **chess**: координация процесса

### Проверка состояния
```bash
# Просмотр текущего кластера
cat cluster_hosts

# Проверка живых дронов (в коде)
# alive_drones = esp.ping_all_drones(available_drones)

# Ручное тестирование MPI
mpirun --hostfile cluster_hosts -np 2 hostname
```

### Типичные проблемы

1. **MPI не найден**
   ```bash
   sudo apt install openmpi-bin openmpi-dev
   ```

2. **Stockfish не найден**
   - Проверить пути в `_find_stockfish_binary_for_cluster()`
   - Убедиться что файл исполняемый: `chmod +x stockfish`

3. **Ошибка версии Stockfish dev (invalid literal for int)**
   - Возникает с dev-версиями типа `dev20240410b4ac3d6b`
   - Система автоматически создает UCI wrapper для обхода проблемы
   - Проверить работу: `python3 test_stockfish_dev.py`

3. **IP дронов недоступны**
   - Проверить `raw_ip` в `DRONES_CONFIG`
   - Проверить сетевое соединение

4. **Частые перезапуски**
   - Увеличить `CLUSTER_RESTART_DELAY`
   - Проверить стабильность пинг-соединений

## Режимы работы

### Локальный режим (по умолчанию)
```bash
export ALG_MODE=api  # или не устанавливать
python3 drone/main.py
```
Использует локальный Stockfish на одном дроне.

### Кластерный режим
```bash
export ALG_MODE=cluster
./run_with_cluster_restart.sh
```
Использует MPI кластер из живых дронов с автоматическим обновлением.

### Мок режим
```bash
export ALG_MODE=llm  # или любое другое значение
python3 drone/main.py
```
Использует простой мок-алгоритм без Stockfish.

## Расширение

Система спроектирована для легкого расширения:

1. **Новые алгоритмы обнаружения дронов** - модифицировать `ping_all_drones()`
2. **Дополнительные параметры MPI** - изменить команду в `_cluster_analyze_position()`
3. **Другие движки** - добавить поддержку в `_generate_stockfish_move()`
4. **Мониторинг производительности** - добавить метрики в `cluster_manager`

Вся логика кластера инкапсулирована в `cluster_manager.py`, что облегчает поддержку и разработку.
