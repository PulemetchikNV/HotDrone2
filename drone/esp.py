import json
import time
import uuid
import os
import threading

try:
    from .helpers import setup_logging
    from .const import LEADER_DRONE
except ImportError:
    from helpers import setup_logging
    from const import LEADER_DRONE

def drone_name_to_short(drone_name):
    """
    Конвертирует полное имя дрона в короткий номер для экономии места в сообщениях
    drone6 -> 6, drone19 -> 19, drone123 -> 123
    """
    if drone_name.startswith('drone'):
        return drone_name[5:]  # убираем префикс "drone"
    return drone_name  # если нет префикса, возвращаем как есть

class EspController:
    def __init__(self, swarm, drone_name=None):
        self.drone_name = drone_name or os.environ.get('DRONE_NAME', 'unknown_drone')
        self.logger = setup_logging(self.drone_name)
        self.swarm = swarm
        
        # Определяем является ли дрон лидером
        self.is_leader = (self.drone_name == LEADER_DRONE)
        
        # Для обработки подтверждений
        self._ack_lock = threading.Lock()
        self._received_acks = set()
        
        # События для получения команд
        self._chess_move_event = threading.Event()
        self._received_chess_move = None
        
        # Для лидера - отслеживание подтверждений от ведомых
        self._expected_followers = set()  # будет заполняться при необходимости
        
        self.logger.info(f"EspController initialized: leader={self.is_leader}, drone={self.drone_name}")

    def _on_custom_message(self, message: str):
        """Обработчик сообщений от лидера и подтверждений"""
        try:
            obj = json.loads(message)
        except Exception:
            return
            
        cmd_type = obj.get('type')

        # --- Ack handling (for leader) ---
        if cmd_type == 'ack':
            if self.is_leader:
                ack_id = obj.get('ack_id')
                with self._ack_lock:
                    self._received_acks.add(ack_id)
            return

        # --- Command handling (for followers) ---
        target = obj.get('to', '*')
        
        # Проверяем что команда для нас (поддерживаем как полные, так и сокращённые имена)
        if target != '*' and target != self.drone_name and target != drone_name_to_short(self.drone_name):
            return
            
        # Отправляем ack, если есть msg_id
        if 'msg_id' in obj and not self.is_leader and self.swarm:
            ack_payload = {
                'type': 'ack',
                'ack_id': obj['msg_id'],
                'from': self.drone_name
            }
            try:
                self.swarm.broadcast_custom_message(json.dumps(ack_payload))
            except Exception as e:
                self.logger.warning(f"Failed to send ack via broadcast: {e}")

        # Обработка шахматных команд
        target_matches = (target == self.drone_name or target == drone_name_to_short(self.drone_name))
        
        # Команда шахматного хода: "move" с форматом "c2->d4"
        if cmd_type == 'move' and target_matches:
            chess_move = obj.get('move')  # формат "c2->d4"
            if chess_move and '->' in chess_move:
                self.logger.info(f"Received CHESS MOVE command: {chess_move}")
                self._received_chess_move = chess_move
                self._chess_move_event.set()
            else:
                self.logger.warning(f"Invalid chess move format: {chess_move}")
                
        # Команда посадки
        elif cmd_type == 'land':
            self.logger.info("Received LAND command from leader")
            # Для шахмат пока не используется, но оставляем для совместимости

    def _broadcast_reliable(self, payload, retries=3, timeout=0.5):
        """Надежная отправка сообщения с ожиданием подтверждения."""
        if not self.swarm:
            self.logger.warning("No swarm link; broadcast skipped")
            return False

        msg_id = uuid.uuid4().hex[:4]
        payload['msg_id'] = msg_id

        # Для команды move делаем больше попыток
        is_move_command = payload.get('type') == 'move'
        max_attempts = retries * 2 if is_move_command else retries

        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            
            # Очищаем старое подтверждение перед отправкой, если оно есть
            with self._ack_lock:
                if msg_id in self._received_acks:
                    self._received_acks.remove(msg_id)

            if is_move_command:
                self.logger.info(f"Broadcasting CHESS MOVE (attempt {attempt}/{max_attempts}): {payload}")
            else:
                self.logger.info(f"Broadcasting (attempt {attempt}/{max_attempts}): {payload}")
            
            try:
                msg = json.dumps(payload)
                self.swarm.broadcast_custom_message(msg)
            except Exception as e:
                self.logger.warning(f"Broadcast failed: {e}")
                time.sleep(timeout)
                continue

            # Ждем подтверждения
            time.sleep(timeout)

            with self._ack_lock:
                if msg_id in self._received_acks:
                    self.logger.info(f"ACK received for msg_id: {msg_id}")
                    return True
            
            if is_move_command:
                self.logger.warning(f"No ACK for CHESS MOVE {msg_id} (attempt {attempt}/{max_attempts})")
            else:
                self.logger.warning(f"No ACK for {msg_id} (attempt {attempt}/{max_attempts})")

        self.logger.error(f"Broadcast failed after {max_attempts} retries for payload: {payload}")
        return False


def create_chess_move_message(to_drone, chess_move):
    """
    Создает сообщение для шахматного хода
    
    Args:
        to_drone: имя дрона-получателя (например, "drone16")
        chess_move: ход в формате "c2->d4"
        
    Returns:
        dict: JSON payload для отправки
    """
    return {
        'type': 'move',
        'to': to_drone,
        'move': chess_move
    }


def parse_chess_move(chess_move_str):
    """
    Парсит строку шахматного хода формата "c2->d4"
    
    Args:
        chess_move_str: строка формата "from_cell->to_cell"
        
    Returns:
        tuple: (from_cell, to_cell) или (None, None) при ошибке
    """
    if not chess_move_str or '->' not in chess_move_str:
        return None, None
        
    try:
        from_cell, to_cell = chess_move_str.split('->', 1)
        from_cell = from_cell.strip()
        to_cell = to_cell.strip()
        
        # Проверяем формат клеток (например, a1, h8)
        if (len(from_cell) == 2 and len(to_cell) == 2 and 
            from_cell[0] in 'abcdefgh' and from_cell[1] in '12345678' and
            to_cell[0] in 'abcdefgh' and to_cell[1] in '12345678'):
            return from_cell, to_cell
    except:
        pass
        
    return None, None