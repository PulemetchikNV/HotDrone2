import logging
import socket
import os

try:
    from .helpers import setup_logging
    from .const import rovers
except ImportError:
    from helpers import setup_logging
    from const import rovers

class RoverControllerMain:
    def __init__(self, initial_x=0, initial_y=0, initial_yaw=0, logger=None):
        self.logger = logger or setup_logging('rover')
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Словарь для хранения текущих позиций каждого ровера
        self.rover_positions = {}
        for rover_id in rovers.keys():
            self.rover_positions[rover_id] = {'x': 0, 'y': 0, 'yaw': 0}

    def send_command(self, rover_id, command_str):
        if rover_id not in rovers:
            self.logger.error(f"Неизвестный ID ровера: {rover_id}")
            return False
            
        rover_config = rovers[rover_id]
        robot_ip = rover_config['ip']
        robot_port = rover_config['port']
        
        self.logger.info(f"Отправка команды роверу {rover_id} ({robot_ip}:{robot_port}): {command_str}")
        try:
            self.sock.sendto(command_str.encode('utf-8'), (robot_ip, robot_port))
            return True
        except Exception as e:
            self.logger.error(f"Ошибка отправки команды роверу {rover_id}: {e}")
            return False

    def get_pose(self, rover_id):
        if rover_id not in rovers:
            self.logger.error(f"Неизвестный ID ровера: {rover_id}")
            return None
            
        self.logger.info(f"Запрос позиции ровера {rover_id}")
        return self.send_command(rover_id, "POSE")

    def navigate(self, rover_id, current_x=0, current_y=0, current_yaw=0, x=0, y=0):
        if rover_id not in rovers:
            self.logger.error(f"Неизвестный ID ровера: {rover_id}")
            return False
            
        self.logger.info(f"Начало отправки команды роверу {rover_id} -> ({x}, {y}, {current_yaw})")
        
        # Отправляем цель
        if not self.send_command(rover_id, f"TARGET:{x},{y}"):
            return False
        
        # Отправляем текущую позицию ровера для начала движения
        current_pos = self.rover_positions[rover_id]
        if not self.send_command(rover_id, f"POSE:{current_x},{current_y},{current_yaw}"):
            return False
            
        self.logger.info(f"Команды отправлены роверу {rover_id}. Робот должен начать движение")

        # Обновляем сохраненную позицию ровера
        return True
    
    def set_rover_position(self, rover_id, x, y, yaw):
        """Устанавливает текущую позицию ровера (для обновления из внешних источников)"""
        if rover_id not in rovers:
            self.logger.error(f"Неизвестный ID ровера: {rover_id}")
            return False
            
        self.rover_positions[rover_id] = {'x': x, 'y': y, 'yaw': yaw}
        self.logger.info(f"Обновлена позиция ровера {rover_id}: ({x}, {y}, {yaw})")
        return True


class RoverControllerMock:
    def __init__(self, drone_name=None, logger=None):
        self.drone_name = drone_name or os.environ.get('DRONE_NAME', 'unknown_drone')
        self.logger = logger or setup_logging(self.drone_name)

        # Словарь для хранения текущих позиций каждого ровера
        self.rover_positions = {}
        for rover_id in rovers.keys():
            self.rover_positions[rover_id] = {'x': 0, 'y': 0, 'yaw': 0}

    def send_command(self, rover_id, command_str):
        if rover_id not in rovers:
            self.logger.error(f"🤖 MOCK: Неизвестный ID ровера: {rover_id}")
            return False
            
        self.logger.info(f"🤖 MOCK: Отправка команды роверу {rover_id}: {command_str}")
        self.logger.info(f"🤖 MOCK: Команда роверу {rover_id}: {command_str} - отправлена")
        return True

    def get_pose(self, rover_id):
        if rover_id not in rovers:
            self.logger.error(f"🤖 MOCK: Неизвестный ID ровера: {rover_id}")
            return None
            
        self.logger.info(f"🤖 MOCK: Запрос позиции ровера {rover_id}")
        return True

    def navigate(self, rover_id, x=0, y=0, yaw=0):
        if rover_id not in rovers:
            self.logger.error(f"🤖 MOCK: Неизвестный ID ровера: {rover_id}")
            return False
            
        self.logger.info(f"🤖 MOCK: Начало навигации ровера {rover_id} -> ({x}, {y}, {yaw})")
        
        self.send_command(rover_id, f"TARGET:{x},{y}")
        current_pos = self.rover_positions[rover_id]
        self.send_command(rover_id, f"POSE:{current_pos['x']},{current_pos['y']},{current_pos['yaw']}")
        
        self.logger.info(f"🤖 MOCK: Навигация ровера {rover_id} завершена")

        # Обновляем сохраненную позицию ровера
        self.rover_positions[rover_id] = {'x': x, 'y': y, 'yaw': yaw}
        return True
    
    def set_rover_position(self, rover_id, x, y, yaw):
        """Устанавливает текущую позицию ровера (для обновления из внешних источников)"""
        if rover_id not in rovers:
            self.logger.error(f"🤖 MOCK: Неизвестный ID ровера: {rover_id}")
            return False
            
        self.rover_positions[rover_id] = {'x': x, 'y': y, 'yaw': yaw}
        self.logger.info(f"🤖 MOCK: Обновлена позиция ровера {rover_id}: ({x}, {y}, {yaw})")
        return True


# Выбор реализации по переменной окружения
_impl = os.getenv('ROVER_IMPL', 'main').lower()
if _impl == 'mock':
    RoverController = RoverControllerMock
else:
    RoverController = RoverControllerMain