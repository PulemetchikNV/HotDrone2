DRONE_LIST='drone2,drone4,drone6'
# DRONE_LIST = 'drone15,drone21,drone11,drone18'
#,drone18,drone21,drone21'
LEADER_DRONE = 'drone2'
DRONES_TOTAL = 3
OUR_TEAM = 'white'

# Режим алгоритма выбора ходов
ALG_MODE = 'cluster2'

# Централизованная конфигурация дронов
DRONES_CONFIG = {
    'drone2': {
        'role': 'king',
        'color': 'white',
        'initial_letter': 'h',
        'team': 'white',
        'raw_ip': '192.168.1.85'
    },
    'drone4': {
        'role': 'queen',
        'color': 'white',
        'initial_letter': 'd',
        'team': 'white',
        'raw_ip': '192.168.1.115'
    },
    'drone6': {
        'role': 'knight',
        'color': 'white',
        'initial_letter': 'b',
        'team': 'white',
        'raw_ip': '192.168.1.89'
    },
    # Добавляйте новые дроны здесь по мере необходимости
}


def get_drone_config(drone_name: str = None) -> dict:
    """
    Получает конфигурацию дрона по имени.
    
    Args:
        drone_name: Имя дрона. Если None, берется из переменной окружения DRONE_NAME.
        
    Returns:
        dict: Конфигурация дрона с ключами: role, color, initial_letter, team
        
    Raises:
        ValueError: Если дрон не найден в конфигурации
    """
    import os
    
    if drone_name is None:
        drone_name = os.getenv('DRONE_NAME', 'drone15')
    
    if drone_name not in DRONES_CONFIG:
        raise ValueError(f"Drone '{drone_name}' not found in DRONES_CONFIG. Available drones: {list(DRONES_CONFIG.keys())}")
    
    return DRONES_CONFIG[drone_name].copy()


def get_current_drone_config() -> dict:
    """Получает конфигурацию текущего дрона из переменной окружения DRONE_NAME."""
    return get_drone_config()

rovers = {
    'rover_b': {
        'ip': '192.168.1.53',
        'port': 80,
        'initial_letter': 'c',
        'number': 16,
    },
}