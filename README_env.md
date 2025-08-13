# 🤖 Конфигурация дронов через .env файлы

Больше не нужно каждый раз писать `export DRONE_NAME=drone7`! Теперь все настройки хранятся в удобном `.env` файле.

## 🚀 Быстрый старт

### 1. Автоматическая настройка (рекомендуется)

```bash
# Интерактивное меню выбора дрона
./scripts/setup_env.sh

# Или сразу для конкретного дрона
./scripts/setup_env.sh drone7
./scripts/setup_env.sh drone6
./scripts/setup_env.sh drone19
```

### 2. Ручная настройка

```bash
# Скопируйте пример для нужного дрона
cp env.drone7.example .env    # для лидера
cp env.drone6.example .env    # для diamond дрона  
cp env.drone19.example .env   # для stick дрона

# Или универсальный пример
cp env.example .env
```

## 📋 Доступные переменные

| Переменная | Описание | Значение по умолчанию |
|------------|----------|-----------------------|
| `DRONE_NAME` | Имя дрона | `drone` |
| `DRONE_ROLE` | Роль в крафте: `D`, `S`, `ANY` | `ANY` |
| `GRID_CELL_SIZE` | Размер ячейки сетки (м) | `0.5` |
| `TARGET_Z` | Целевая высота полёта (м) | `1.2` |
| `NAV_SPEED` | Скорость навигации (м/с) | `0.3` |
| `ARUCO_GRID_TYPE` | Тип ArUco сетки: `main`/`alt` | `main` |
| `RECIPE` | Fallback рецепт | `025SDD` |
| `LOG_LEVEL` | Уровень логирования | `INFO` |
| `SKYROS_DEBUG` | Отладка Skyros | `false` |

## 🎯 Примеры использования

### Drone7 (лидер)
```bash
./scripts/setup_env.sh drone7
python3 drone/main_2.py  # Переменные загрузятся автоматически!
```

### Drone6 (diamond фоловер)
```bash
./scripts/setup_env.sh drone6
python3 drone/main_2.py
```

### Drone19 (stick фоловер)
```bash
./scripts/setup_env.sh drone19
python3 drone/main_2.py
```

## 🔧 Настройка под свои нужды

Отредактируйте `.env` файл:

```bash
nano .env
```

Пример содержимого:
```env
DRONE_NAME=drone7
DRONE_ROLE=ANY
TARGET_Z=1.5
NAV_SPEED=0.4
RECIPE=125DDS
```

## 📦 Структура файлов

```
DroneCraft/
├── .env                    # Основной конфиг (создаётся автоматически)
├── env.example             # Универсальный пример
├── env.drone7.example      # Пример для лидера
├── env.drone6.example      # Пример для diamond дрона
├── env.drone19.example     # Пример для stick дрона
├── drone/
│   ├── env_loader.py       # Модуль загрузки .env
│   ├── stage1_mod.py       # Поддерживает .env
│   └── stage2.py           # Поддерживает .env
└── scripts/
    └── setup_env.sh        # Автоматическая настройка
```

## 🎨 Рецепты крафта

| Код | Название | Матрица |
|-----|----------|---------|
| `025SDD` | Diamond Pickaxe | `[D,D,D] [_,S,_] [_,S,_]` |
| `125DDS` | Diamond Axe | `[D,D,_] [D,S,_] [_,S,_]` |
| `285SDD` | Diamond Mace | `[_,D,D] [_,D,D] [S,_,_]` |

## ⚡ Преимущества

- ✅ **Не нужно export команды** каждый раз
- ✅ **Конфигурация в одном месте** 
- ✅ **Быстрое переключение** между дронами
- ✅ **Автоматическая загрузка** при запуске
- ✅ **Резервные копии** старых настроек
- ✅ **Интерактивное меню** для новичков

## 🔒 Безопасность

`.env` файл добавлен в `.gitignore` и не попадёт в репозиторий. Каждый дрон может иметь свои настройки без конфликтов.

## 🐛 Отладка

Если что-то не работает:

1. Проверьте что `.env` файл существует: `ls -la .env`
2. Посмотрите содержимое: `cat .env`
3. Проверьте логи загрузки при запуске программы
4. Убедитесь что переменные не переопределены в системе

## 🆘 Помощь

```bash
./scripts/setup_env.sh --help
```
