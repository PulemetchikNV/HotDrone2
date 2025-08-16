DRONE_LIST = 'drone21'
#,drone18,drone21,drone21'
LEADER_DRONE = 'drone21'
DRONES_TOTAL = 1
OUR_TEAM = 'black'

# Централизованная конфигурация дронов
DRONES_CONFIG = {
    # 'drone15': {
    #     'role': 'queen',
    #     'color': 'black',
    #     'initial_letter': 'd',
    #     'team': 'black'
    # },
    'drone21': {
        'role': 'king', 
        'color': 'black',
        'initial_letter': 'e',
        'team': 'black'
    }
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
    # 'rover_a': {
    #     'ip': '192.168.1.97',
    #     'port': 12345,
    #     'initial_letter': 'a',
    #     'number': 2,
    # },
    'rover_b': {
        'ip': '192.168.1.98',
        'port': 12345,
        'initial_letter': 'b',
        'number': 11,
    }
    # '2': {
    #     'ip': '192.168.1.97',
    #     'port': 12345,
    #     'initial_letter': 'a'
    # },
    # '1': {
    #     'ip': '192.168.1.71',
    #     'port': 12345,
    #     'initial_letter': 'b'
    # },
    # '3': {
    #     'ip': '192.168.1.73',
    #     'port': 12345,
    #     'initial_letter': 'c'
    # },
    # '4': {
    #     'ip': '192.168.1.74',
    #     'port': 12345,
    #     'initial_letter': 'd'
    # }
}