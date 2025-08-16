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
    def __init__(self, logger=None):
        self.logger = logger or setup_logging('rover')
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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

    def navigate(self, rover_id, current_x, current_y, current_yaw, target_x, target_y):
        """
        Навигация ровера согласно API документации.
        
        Args:
            rover_id: ID ровера
            current_x, current_y: Текущая позиция в метрах
            current_yaw: Текущий угол в градусах (-180 до 180)
            target_x, target_y: Целевая позиция в метрах
        """
        if rover_id not in rovers:
            self.logger.error(f"Неизвестный ID ровера: {rover_id}")
            return False
            
        self.logger.info(f"Навигация ровера {rover_id}: ({current_x:.3f}, {current_y:.3f}, {current_yaw:.1f}°) -> ({target_x:.3f}, {target_y:.3f})")
        
        # 1. Отправляем цель (TARGET)
        if not self.send_command(rover_id, f"TARGET:{target_x:.3f},{target_y:.3f}"):
            return False
        
        # 2. Отправляем текущую позицию для начала движения (POSE)
        if not self.send_command(rover_id, f"POSE:{current_x:.3f},{current_y:.3f},{current_yaw:.1f}"):
            return False
            
        self.logger.info(f"Команды отправлены роверу {rover_id}. Робот должен начать движение")
        return True
    



class RoverControllerMock:
    def __init__(self, drone_name=None, logger=None):
        self.drone_name = drone_name or os.environ.get('DRONE_NAME', 'unknown_drone')
        self.logger = logger or setup_logging(self.drone_name)

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

    def navigate(self, rover_id, current_x, current_y, current_yaw, target_x, target_y):
        """
        Навигация ровера согласно API документации (MOCK версия).
        
        Args:
            rover_id: ID ровера
            current_x, current_y: Текущая позиция в метрах
            current_yaw: Текущий угол в градусах (-180 до 180)
            target_x, target_y: Целевая позиция в метрах
        """
        if rover_id not in rovers:
            self.logger.error(f"🤖 MOCK: Неизвестный ID ровера: {rover_id}")
            return False
            
        self.logger.info(f"🤖 MOCK: Навигация ровера {rover_id}: ({current_x:.3f}, {current_y:.3f}, {current_yaw:.1f}°) -> ({target_x:.3f}, {target_y:.3f})")
        
        # 1. Отправляем цель (TARGET)
        self.send_command(rover_id, f"TARGET:{target_x:.3f},{target_y:.3f}")
        
        # 2. Отправляем текущую позицию для начала движения (POSE)
        self.send_command(rover_id, f"POSE:{current_x:.3f},{current_y:.3f},{current_yaw:.1f}")
        
        self.logger.info(f"🤖 MOCK: Навигация ровера {rover_id} завершена")
        return True
    



# Выбор реализации по переменной окружения
_impl = os.getenv('ROVER_IMPL', 'main').lower()
if _impl == 'mock':
    RoverController = RoverControllerMock
else:
    RoverController = RoverControllerMain