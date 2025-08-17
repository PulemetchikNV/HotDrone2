import logging
import requests
import os
import math
import time
from typing import Optional, Dict, Any

try:
    from .helpers import setup_logging
    from .const import rovers
except ImportError:
    from helpers import setup_logging
    from const import rovers

class RoverControllerMain:
    def __init__(self, logger=None):
        self.logger = logger or setup_logging('rover')
        self.session = requests.Session()

    def _get_rover_url(self, rover_id):
        """Получить базовый URL для ровера"""
        if rover_id not in rovers:
            return None
        rover_config = rovers[rover_id]
        return f"http://{rover_config['ip']}:{rover_config['port']}"

    def get_status(self, rover_id) -> Dict[str, Any]:
        """
        Получить статус ровера
        
        Args:
            rover_id: ID ровера
            
        Returns:
            Dictionary с информацией о статусе ровера
        """
        if rover_id not in rovers:
            self.logger.error(f"Неизвестный ID ровера: {rover_id}")
            return {}
            
        base_url = self._get_rover_url(rover_id)
        try:
            response = self.session.get(f"{base_url}/status")
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            self.logger.error(f"Ошибка получения статуса ровера {rover_id}: {e}")
            return {}

    def send_command(self, rover_id, command: str, distance: int = 0, angle: int = 0) -> bool:
        """
        Отправить команду роверу через HTTP API
        
        Args:
            rover_id: ID ровера
            command: Тип команды ('forward', 'turn', 'stop')
            distance: Расстояние в мм (для команды forward)
            angle: Угол в градусах (для команды turn)
            
        Returns:
            True если команда отправлена успешно, False иначе
        """
        if rover_id not in rovers:
            self.logger.error(f"Неизвестный ID ровера: {rover_id}")
            return False
            
        base_url = self._get_rover_url(rover_id)
        data = {
            "command": command,
            "distance": distance,
            "angle": angle
        }
        
        self.logger.info(f"Отправка команды роверу {rover_id}: {command}, дистанция: {distance}мм, угол: {angle}°")
        try:
            response = self.session.post(
                f"{base_url}/command",
                json=data,
                headers={'Content-Type': 'application/json'}
            )
            response.raise_for_status()
            result = response.json()
            self.logger.info(f"Результат команды роверу {rover_id}: {result.get('status', 'Unknown')}")
            return True
        except requests.RequestException as e:
            self.logger.error(f"Ошибка отправки команды роверу {rover_id}: {e}")
            return False

    def move_forward(self, rover_id, distance_mm: int) -> bool:
        """
        Движение ровера вперед на указанное расстояние
        
        Args:
            rover_id: ID ровера
            distance_mm: Расстояние в миллиметрах
            
        Returns:
            True если команда отправлена успешно
        """
        self.logger.info(f"Движение ровера {rover_id} вперед на {distance_mm}мм...")
        return self.send_command(rover_id, "forward", distance=distance_mm)

    def turn(self, rover_id, angle: int) -> bool:
        """
        Поворот ровера на указанный угол
        
        Args:
            rover_id: ID ровера
            angle: Угол в градусах (положительный = направо, отрицательный = налево)
            
        Returns:
            True если команда отправлена успешно
        """
        direction = "направо" if angle > 0 else "налево"
        self.logger.info(f"Поворот ровера {rover_id} {direction} на {abs(angle)} градусов...")
        return self.send_command(rover_id, "turn", angle=angle)

    def stop(self, rover_id) -> bool:
        """
        Остановка ровера
        
        Args:
            rover_id: ID ровера
            
        Returns:
            True если команда отправлена успешно
        """
        self.logger.info(f"Остановка ровера {rover_id}...")
        return self.send_command(rover_id, "stop")

    def wait_for_completion(self, rover_id, timeout: int = 60) -> bool:
        """
        Ожидание завершения текущего движения
        
        Args:
            rover_id: ID ровера
            timeout: Максимальное время ожидания в секундах
            
        Returns:
            True если движение завершено, False при тайм-ауте
        """
        start_time = time.time()
        self.logger.info(f"Ожидание завершения движения ровера {rover_id}...")
        
        while time.time() - start_time < timeout:
            status = self.get_status(rover_id)
            if status.get('status') == 'Ready':
                self.logger.info(f"Движение ровера {rover_id} завершено!")
                return True
            time.sleep(0.5)
        
        self.logger.warning(f"Тайм-аут ожидания завершения движения ровера {rover_id}")
        return False

    def execute_sequence(self, rover_id, commands: list) -> bool:
        """
        Выполнить последовательность команд
        
        Args:
            rover_id: ID ровера
            commands: Список словарей с командами
                     Каждая команда должна содержать 'type', 'value', и опционально 'wait'
                     
        Returns:
            True если все команды выполнены успешно
        """
        for i, cmd in enumerate(commands):
            self.logger.info(f"Выполнение команды {i+1}/{len(commands)} для ровера {rover_id}: {cmd}")
            
            if cmd['type'] == 'forward':
                success = self.move_forward(rover_id, cmd['value'])
            elif cmd['type'] == 'turn':
                success = self.turn(rover_id, cmd['value'])
            elif cmd['type'] == 'stop':
                success = self.stop(rover_id)
            else:
                self.logger.error(f"Неизвестный тип команды: {cmd['type']}")
                continue
            
            if not success:
                self.logger.error(f"Команда {i+1} для ровера {rover_id} не выполнена")
                return False
            
            # Ожидание завершения если указано
            if cmd.get('wait', True):
                if not self.wait_for_completion(rover_id):
                    self.logger.error(f"Команда {i+1} для ровера {rover_id} не завершилась вовремя")
                    return False
            
            # Дополнительная задержка если указана
            if 'delay' in cmd:
                self.logger.info(f"Ожидание {cmd['delay']} секунд...")
                time.sleep(cmd['delay'])
        
        self.logger.info(f"Все команды для ровера {rover_id} выполнены успешно!")
        return True

    def move_diagonal(self, rover_id, distance_mm: int, angle: int = 45) -> bool:
        """
        Диагональное движение: поворот на угол + движение на расстояние катета
        
        Args:
            rover_id: ID ровера
            distance_mm: Расстояние катета в миллиметрах
            angle: Угол поворота в градусах (по умолчанию 45°)
            
        Returns:
            True если все команды выполнены успешно
        """
        self.logger.info(f"Диагональное движение ровера {rover_id}: поворот на {angle}°, движение на {distance_mm}мм")
        
        # Последовательность команд для диагонального движения
        commands = [
            {'type': 'turn', 'value': angle, 'wait': True},
            {'type': 'forward', 'value': distance_mm, 'wait': True}
        ]
        
        return self.execute_sequence(rover_id, commands)

    def move_diagonal_and_return(self, rover_id, distance_mm: int, angle: int = 45) -> bool:
        """
        Диагональное движение с возвратом в исходную ориентацию
        
        Args:
            rover_id: ID ровера
            distance_mm: Расстояние катета в миллиметрах
            angle: Угол поворота в градусах (по умолчанию 45°)
            
        Returns:
            True если все команды выполнены успешно
        """
        self.logger.info(f"Диагональное движение ровера {rover_id} с возвратом: поворот {angle}°, движение {distance_mm}мм, поворот -{angle}°")
        
        # Последовательность команд для диагонального движения с возвратом
        commands = [
            {'type': 'turn', 'value': angle, 'wait': True},
            {'type': 'forward', 'value': distance_mm, 'wait': True},
            {'type': 'turn', 'value': -angle, 'wait': True}
        ]
        
        return self.execute_sequence(rover_id, commands)

    def get_pose(self, rover_id):
        """
        Запрос позиции ровера (legacy метод для совместимости)
        
        Args:
            rover_id: ID ровера
            
        Returns:
            True если команда отправлена успешно
        """
        self.logger.info(f"Запрос позиции ровера {rover_id}")
        return self.get_status(rover_id)

    def navigate(self, rover_id, current_x, current_y, current_yaw, target_x, target_y):
        """
        Навигация ровера (legacy метод для совместимости с старым API).
        Теперь преобразует координаты в команды движения.
        
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
        
        # Вычисляем дистанцию и угол для движения
        dx = target_x - current_x
        dy = target_y - current_y
        distance_m = math.sqrt(dx*dx + dy*dy)
        distance_mm = int(distance_m * 1000)  # Конвертируем в мм
        
        if distance_mm == 0:
            self.logger.info(f"Ровер {rover_id} уже находится в целевой позиции")
            return True
        
        # Вычисляем угол поворота
        target_angle = math.degrees(math.atan2(dy, dx))
        turn_angle = target_angle - current_yaw
        
        # Нормализуем угол поворота в диапазон [-180, 180]
        while turn_angle > 180:
            turn_angle -= 360
        while turn_angle < -180:
            turn_angle += 360
        
        # Выполняем команды движения
        commands = []
        if abs(turn_angle) > 1:  # Поворачиваем только если угол значительный
            commands.append({'type': 'turn', 'value': int(turn_angle), 'wait': True})
        
        commands.append({'type': 'forward', 'value': distance_mm, 'wait': True})
        
        return self.execute_sequence(rover_id, commands)


class RoverControllerMock:
    def __init__(self, drone_name=None, logger=None):
        self.drone_name = drone_name or os.environ.get('DRONE_NAME', 'unknown_drone')
        self.logger = logger or setup_logging(self.drone_name)

    def get_status(self, rover_id) -> Dict[str, Any]:
        """MOCK версия получения статуса ровера"""
        if rover_id not in rovers:
            self.logger.error(f"🤖 MOCK: Неизвестный ID ровера: {rover_id}")
            return {}
        
        self.logger.info(f"🤖 MOCK: Получение статуса ровера {rover_id}")
        return {'status': 'Ready', 'battery': 85, 'position': {'x': 0, 'y': 0, 'angle': 0}}

    def send_command(self, rover_id, command: str, distance: int = 0, angle: int = 0) -> bool:
        """MOCK версия отправки команды"""
        if rover_id not in rovers:
            self.logger.error(f"🤖 MOCK: Неизвестный ID ровера: {rover_id}")
            return False
            
        self.logger.info(f"🤖 MOCK: Отправка команды роверу {rover_id}: {command}, дистанция: {distance}мм, угол: {angle}°")
        return True

    def move_forward(self, rover_id, distance_mm: int) -> bool:
        """MOCK версия движения вперед"""
        self.logger.info(f"🤖 MOCK: Движение ровера {rover_id} вперед на {distance_mm}мм")
        return True

    def turn(self, rover_id, angle: int) -> bool:
        """MOCK версия поворота"""
        direction = "направо" if angle > 0 else "налево"
        self.logger.info(f"🤖 MOCK: Поворот ровера {rover_id} {direction} на {abs(angle)} градусов")
        return True

    def stop(self, rover_id) -> bool:
        """MOCK версия остановки"""
        self.logger.info(f"🤖 MOCK: Остановка ровера {rover_id}")
        return True

    def wait_for_completion(self, rover_id, timeout: int = 60) -> bool:
        """MOCK версия ожидания завершения"""
        self.logger.info(f"🤖 MOCK: Ожидание завершения движения ровера {rover_id} (мгновенно)")
        return True

    def execute_sequence(self, rover_id, commands: list) -> bool:
        """MOCK версия выполнения последовательности команд"""
        for i, cmd in enumerate(commands):
            self.logger.info(f"🤖 MOCK: Выполнение команды {i+1}/{len(commands)} для ровера {rover_id}: {cmd}")
        self.logger.info(f"🤖 MOCK: Все команды для ровера {rover_id} выполнены")
        return True

    def move_diagonal(self, rover_id, distance_mm: int, angle: int = 45) -> bool:
        """MOCK версия диагонального движения"""
        self.logger.info(f"🤖 MOCK: Диагональное движение ровера {rover_id}: поворот на {angle}°, движение на {distance_mm}мм")
        return True

    def move_diagonal_and_return(self, rover_id, distance_mm: int, angle: int = 45) -> bool:
        """MOCK версия диагонального движения с возвратом"""
        self.logger.info(f"🤖 MOCK: Диагональное движение ровера {rover_id} с возвратом: {angle}°, {distance_mm}мм, -{angle}°")
        return True

    def get_pose(self, rover_id):
        """MOCK версия запроса позиции"""
        if rover_id not in rovers:
            self.logger.error(f"🤖 MOCK: Неизвестный ID ровера: {rover_id}")
            return None
            
        self.logger.info(f"🤖 MOCK: Запрос позиции ровера {rover_id}")
        return True

    def navigate(self, rover_id, current_x, current_y, current_yaw, target_x, target_y):
        """MOCK версия навигации"""
        if rover_id not in rovers:
            self.logger.error(f"🤖 MOCK: Неизвестный ID ровера: {rover_id}")
            return False
            
        self.logger.info(f"🤖 MOCK: Навигация ровера {rover_id}: ({current_x:.3f}, {current_y:.3f}, {current_yaw:.1f}°) -> ({target_x:.3f}, {target_y:.3f})")
        self.logger.info(f"🤖 MOCK: Навигация ровера {rover_id} завершена")
        return True


# Выбор реализации по переменной окружения
_impl = os.getenv('ROVER_IMPL', 'main').lower()
if _impl == 'mock':
    RoverController = RoverControllerMock
else:
    RoverController = RoverControllerMain