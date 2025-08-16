import json
import time
import uuid
import os
import threading
import socket

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
        
        # Роль может меняться динамически (лидер/ведомый)
        self.is_leader = (self.drone_name == LEADER_DRONE)
        
        # Для обработки подтверждений
        self._ack_lock = threading.Lock()
        self._received_acks = set()
        
        # События для получения команд
        self._chess_move_event = threading.Event()
        self._received_chess_move = None
        
        # Для лидера - отслеживание подтверждений от ведомых
        self._expected_followers = set()  # будет заполняться при необходимости
        
        # Ping/Pong для проверки активности дронов
        self._ping_responses = {}  # {drone_name: timestamp}
        self._ping_lock = threading.Lock()
        
        # Состояние хода лидера
        self._leader_move_state = {
            'is_moving': False,
            'move_start_time': 0,
            'move_timeout': 60.0,  # Таймаут хода в секундах
            'current_move': None
        }
        self._move_state_lock = threading.Lock()
        
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
                # Проверяем что ACK предназначен для нас
                ack_target = obj.get('to', '*')
                if ack_target == '*' or ack_target == self.drone_name or ack_target == drone_name_to_short(self.drone_name):
                    ack_id = obj.get('ack_id')
                    with self._ack_lock:
                        self._received_acks.add(ack_id)
                        self.logger.info(f"Received ACK {ack_id} from {obj.get('from', 'unknown')}")
            return

        # --- Command handling (for followers) ---
        target = obj.get('to', '*')
        
        # Проверяем что команда для нас (поддерживаем как полные, так и сокращённые имена)
        if target != '*' and target != self.drone_name and target != drone_name_to_short(self.drone_name):
            return
            
        # Отправляем ack если команда адресована нам (независимо от роли лидера)
        if 'msg_id' in obj and self.swarm:
            # Получаем отправителя команды
            original_sender = obj.get('from', 'unknown')
            ack_payload = {
                'type': 'ack',
                'ack_id': obj['msg_id'],
                'to': original_sender,  # Отправляем ACK конкретному отправителю
                'from': self.drone_name
            }
            try:
                self.swarm.broadcast_custom_message(json.dumps(ack_payload))
                self.logger.info(f"Sent ACK {obj['msg_id']} to {original_sender}")
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
        
        # Ping запрос - отправляем pong в ответ
        elif cmd_type == 'ping':
            ping_id = obj.get('ping_id')
            from_drone = obj.get('from')
            if ping_id and from_drone:
                pong_payload = {
                    'type': 'pong',
                    'ping_id': ping_id,
                    'to': from_drone,
                    'from': self.drone_name
                }
                try:
                    if self.swarm:
                        self.swarm.broadcast_custom_message(json.dumps(pong_payload))
                except Exception as e:
                    self.logger.warning(f"Failed to send pong: {e}")
        
        # Pong ответ - записываем время ответа
        elif cmd_type == 'pong':
            ping_id = obj.get('ping_id')
            from_drone = obj.get('from')
            if ping_id and from_drone:
                with self._ping_lock:
                    self._ping_responses[from_drone] = time.time()
        
        # Уведомление о начале хода лидера
        elif cmd_type == 'move_start':
            from_drone = obj.get('from')
            move_info = obj.get('move')
            if from_drone:
                with self._move_state_lock:
                    self._leader_move_state['is_moving'] = True
                    self._leader_move_state['move_start_time'] = time.time()
                    self._leader_move_state['current_move'] = move_info
                self.logger.info(f"Leader {from_drone} started move: {move_info}")
        
        # Уведомление о завершении хода лидера
        elif cmd_type == 'move_complete':
            from_drone = obj.get('from')
            move_info = obj.get('move')
            if from_drone:
                with self._move_state_lock:
                    self._leader_move_state['is_moving'] = False
                    self._leader_move_state['move_start_time'] = 0
                    self._leader_move_state['current_move'] = None
                self.logger.info(f"Leader {from_drone} completed move: {move_info}")

    def _broadcast_reliable(self, payload, retries=3, timeout=0.5):
        """Надежная отправка сообщения с ожиданием подтверждения."""
        if not self.swarm:
            self.logger.warning("No swarm link; broadcast skipped")
            return False

        msg_id = uuid.uuid4().hex[:4]
        payload['msg_id'] = msg_id
        # Добавляем отправителя для корректной обработки ACK
        if 'from' not in payload:
            payload['from'] = self.drone_name

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

    def _broadcast_unreliable(self, payload: dict):
        """Отправка сообщения без ожидания подтверждений (для heartbeat)."""
        if not self.swarm:
            return False
        try:
            msg = json.dumps(payload)
            self.swarm.broadcast_custom_message(msg)
            return True
        except Exception as e:
            self.logger.warning(f"Unreliable broadcast failed: {e}")
            return False



    def update_role(self, is_leader: bool):
        if self.is_leader != is_leader:
            self.is_leader = is_leader
            self.logger.info(f"ESP role updated: is_leader={self.is_leader}")
    
    def ping_drone(self, target_drone: str, retries: int = 5, timeout: float = 1.0) -> bool:
        """
        Пингует конкретный дрон и проверяет ответ с ретраями
        
        Args:
            target_drone: Имя целевого дрона
            retries: Количество попыток (по умолчанию 5)
            timeout: Таймаут ожидания ответа в секундах (по умолчанию 1.0)
        """
        if not self.swarm:
            return False
        
        for attempt in range(retries):
            ping_id = uuid.uuid4().hex[:8]
            
            # Очищаем старые ответы от этого дрона
            with self._ping_lock:
                if target_drone in self._ping_responses:
                    del self._ping_responses[target_drone]
            
            # Отправляем ping
            ping_payload = {
                'type': 'ping',
                'ping_id': ping_id,
                'to': target_drone,
                'from': self.drone_name
            }
            
            try:
                self.swarm.broadcast_custom_message(json.dumps(ping_payload))
            except Exception as e:
                self.logger.debug(f"Failed to send ping to {target_drone} (attempt {attempt + 1}/{retries}): {e}")
                continue
            
            # Ждем ответ
            time.sleep(timeout)
            
            # Проверяем ответ
            with self._ping_lock:
                if target_drone in self._ping_responses:
                    if attempt > 0:  # Логируем только если были неудачные попытки
                        self.logger.info(f"Ping to {target_drone} succeeded on attempt {attempt + 1}/{retries}")
                    return True
            
            if attempt < retries - 1:  # Не логируем последнюю неудачную попытку
                self.logger.debug(f"No ping response from {target_drone} (attempt {attempt + 1}/{retries})")
        
        self.logger.warning(f"Drone {target_drone} is not responding to ping after {retries} attempts")
        return False
    
    def ping_all_drones(self, drone_list: list) -> list:
        """Пингует всех дронов и возвращает список живых"""
        alive_drones = []
        
        for drone_name in drone_list:
            if drone_name == self.drone_name:
                # Себя считаем живым
                alive_drones.append(drone_name)
                continue
                
            if self.ping_drone(drone_name):
                alive_drones.append(drone_name)
                self.logger.debug(f"Drone {drone_name} is alive (ping successful)")
            else:
                self.logger.warning(f"Drone {drone_name} is not responding to ping")
        
        return alive_drones
    
    def broadcast_move_start(self, move_info: str):
        """Уведомляет всех дронов о начале хода лидера"""
        if not self.is_leader:
            return False
        
        payload = {
            'type': 'move_start',
            'move': move_info,
            'from': self.drone_name,
            'timestamp': time.time()
        }
        
        self.logger.info(f"Broadcasting move start: {move_info}")
        return self._broadcast_unreliable(payload)
    
    def broadcast_move_complete(self, move_info: str, success: bool = True):
        """Уведомляет всех дронов о завершении хода лидера"""
        if not self.is_leader:
            return False
        
        payload = {
            'type': 'move_complete',
            'move': move_info,
            'success': success,
            'from': self.drone_name,
            'timestamp': time.time()
        }
        
        self.logger.info(f"Broadcasting move complete: {move_info} (success={success})")
        return self._broadcast_unreliable(payload)
    
    def is_leader_moving(self) -> bool:
        """Проверяет, выполняет ли лидер сейчас ход"""
        with self._move_state_lock:
            if not self._leader_move_state['is_moving']:
                return False
            
            # Проверяем таймаут
            elapsed = time.time() - self._leader_move_state['move_start_time']
            if elapsed > self._leader_move_state['move_timeout']:
                self.logger.warning(f"Leader move timeout exceeded ({elapsed:.1f}s > {self._leader_move_state['move_timeout']:.1f}s)")
                # Сбрасываем состояние при таймауте
                self._leader_move_state['is_moving'] = False
                self._leader_move_state['move_start_time'] = 0
                self._leader_move_state['current_move'] = None
                return False
            
            return True
    
    def get_current_move_info(self) -> dict:
        """Возвращает информацию о текущем ходе лидера"""
        with self._move_state_lock:
            return {
                'is_moving': self._leader_move_state['is_moving'],
                'move': self._leader_move_state['current_move'],
                'elapsed_time': time.time() - self._leader_move_state['move_start_time'] if self._leader_move_state['is_moving'] else 0
            }


class WifiEspController:
    def __init__(self, swarm, drone_name=None):
        # Параметры сети
        self.bind_ip = os.environ.get('WIFI_BIND_IP', '0.0.0.0')
        self.port = int(os.environ.get('WIFI_PORT', '47000'))
        self.broadcast_addr = os.environ.get('WIFI_BROADCAST_ADDR', '255.255.255.255')
        self.unicast_targets = [ip.strip() for ip in os.environ.get('WIFI_TARGETS', '').split(',') if ip.strip()]

        self.drone_name = drone_name or os.environ.get('DRONE_NAME', 'unknown_drone')
        self.logger = setup_logging(self.drone_name)
        # Совместимость по сигнатурам: swarm не используется в WiFi-режиме
        self.swarm = None

        # Роль (лидер/ведомый)
        self.is_leader = (self.drone_name == LEADER_DRONE)

        # Ack/Move состояния — совместимы с EspController
        self._ack_lock = threading.Lock()
        self._received_acks = set()
        self._chess_move_event = threading.Event()
        self._received_chess_move = None
        self._expected_followers = set()
        
        # Ping/Pong для проверки активности дронов
        self._ping_responses = {}  # {drone_name: timestamp}
        self._ping_lock = threading.Lock()
        
        # Состояние хода лидера
        self._leader_move_state = {
            'is_moving': False,
            'move_start_time': 0,
            'move_timeout': 60.0,  # Таймаут хода в секундах
            'current_move': None
        }
        self._move_state_lock = threading.Lock()

        # UDP сокет: bind + broadcast
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception:
            pass
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            pass
        self.sock.bind((self.bind_ip, self.port))

        # Поток приёма
        self._stop_event = threading.Event()
        self._rx_thread = threading.Thread(target=self._recv_loop, name=f"wifi-rx-{self.drone_name}", daemon=True)
        self._rx_thread.start()

        self.logger.info(f"WifiEspController initialized on {self.bind_ip}:{self.port}, leader={self.is_leader}, drone={self.drone_name}")

    # Совместимый обработчик входящих сообщений
    def _on_custom_message(self, message: str):
        try:
            obj = json.loads(message)
        except Exception:
            return

        cmd_type = obj.get('type')

        # Ack - проверяем что он для нас перед обработкой
        if cmd_type == 'ack':
            if self.is_leader:
                # Проверяем что ACK предназначен для нас
                ack_target = obj.get('to', '*')
                if ack_target == '*' or ack_target == self.drone_name or ack_target == drone_name_to_short(self.drone_name):
                    ack_id = obj.get('ack_id')
                    with self._ack_lock:
                        self._received_acks.add(ack_id)
                        self.logger.info(f"[WiFi] Received ACK {ack_id} from {obj.get('from', 'unknown')}")
            return

        # Фильтр адресата для остальных команд
        target = obj.get('to', '*')
        if target != '*' and target != self.drone_name and target != drone_name_to_short(self.drone_name):
            return

        # Отправляем ack если команда адресована нам (независимо от роли лидера)
        if 'msg_id' in obj:
            # Получаем отправителя команды из поля 'from' или определяем по контексту
            original_sender = obj.get('from', 'unknown')
            ack_payload = {
                'type': 'ack',
                'ack_id': obj['msg_id'],
                'to': original_sender,  # Отправляем ACK конкретному отправителю
                'from': self.drone_name
            }
            self._send_json(ack_payload)
            self.logger.info(f"[WiFi] Sent ACK {obj['msg_id']} to {original_sender}")

        # Шахматный ход
        target_matches = (target == self.drone_name or target == drone_name_to_short(self.drone_name))
        if cmd_type == 'move' and target_matches:
            chess_move = obj.get('move')
            if chess_move and '->' in chess_move:
                self.logger.info(f"[WiFi] Received CHESS MOVE command: {chess_move}")
                self._received_chess_move = chess_move
                self._chess_move_event.set()
            else:
                self.logger.warning(f"[WiFi] Invalid chess move format: {chess_move}")

        elif cmd_type == 'land':
            self.logger.info("[WiFi] Received LAND command from leader")
        
        # Ping запрос - отправляем pong в ответ
        elif cmd_type == 'ping':
            ping_id = obj.get('ping_id')
            from_drone = obj.get('from')
            if ping_id and from_drone:
                pong_payload = {
                    'type': 'pong',
                    'ping_id': ping_id,
                    'to': from_drone,
                    'from': self.drone_name
                }
                self._send_json(pong_payload)
        
        # Pong ответ - записываем время ответа
        elif cmd_type == 'pong':
            ping_id = obj.get('ping_id')
            from_drone = obj.get('from')
            if ping_id and from_drone:
                with self._ping_lock:
                    self._ping_responses[from_drone] = time.time()
        
        # Уведомление о начале хода лидера
        elif cmd_type == 'move_start':
            from_drone = obj.get('from')
            move_info = obj.get('move')
            if from_drone:
                with self._move_state_lock:
                    self._leader_move_state['is_moving'] = True
                    self._leader_move_state['move_start_time'] = time.time()
                    self._leader_move_state['current_move'] = move_info
                self.logger.info(f"[WiFi] Leader {from_drone} started move: {move_info}")
        
        # Уведомление о завершении хода лидера
        elif cmd_type == 'move_complete':
            from_drone = obj.get('from')
            move_info = obj.get('move')
            if from_drone:
                with self._move_state_lock:
                    self._leader_move_state['is_moving'] = False
                    self._leader_move_state['move_start_time'] = 0
                    self._leader_move_state['current_move'] = None
                self.logger.info(f"[WiFi] Leader {from_drone} completed move: {move_info}")
    
    def _recv_loop(self):
        while not self._stop_event.is_set():
            try:
                data, _addr = self.sock.recvfrom(65535)
                try:
                    text = data.decode('utf-8', errors='ignore')
                except Exception:
                    continue
                self._on_custom_message(text)
            except Exception as e:
                # Не шумим в логи слишком часто
                time.sleep(0.01)

    def _send_json(self, payload: dict):
        # print(f"SENDING JSON: {payload}")
        try:
            msg = json.dumps(payload)
        except Exception as e:
            self.logger.warning(f"[WiFi] encode failed: {e}")
            return False
        data = msg.encode('utf-8')
        sent_any = False
        # Broadcast
        try:
            self.sock.sendto(data, (self.broadcast_addr, self.port))
            sent_any = True
        except Exception as e:
            self.logger.debug(f"[WiFi] broadcast send failed: {e}")
        # Unicast targets (optional)
        for ip in self.unicast_targets:
            try:
                self.sock.sendto(data, (ip, self.port))
                sent_any = True
            except Exception as e:
                self.logger.debug(f"[WiFi] unicast send to {ip} failed: {e}")
        return sent_any

    def _broadcast_reliable(self, payload, retries=3, timeout=0.5):
        print(f"BROADCASTING RELIABLE: {payload}")  
        msg_id = uuid.uuid4().hex[:4]
        payload['msg_id'] = msg_id
        # Добавляем отправителя для корректной обработки ACK
        if 'from' not in payload:
            payload['from'] = self.drone_name
        is_move_command = payload.get('type') == 'move'
        max_attempts = retries * 2 if is_move_command else retries

        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            with self._ack_lock:
                if msg_id in self._received_acks:
                    self._received_acks.remove(msg_id)

            if is_move_command:
                self.logger.info(f"[WiFi] Broadcasting CHESS MOVE (attempt {attempt}/{max_attempts}): {payload}")
            else:
                self.logger.info(f"[WiFi] Broadcasting (attempt {attempt}/{max_attempts}): {payload}")

            ok = self._send_json(payload)
            if not ok:
                time.sleep(timeout)
                continue

            time.sleep(timeout)
            with self._ack_lock:
                if msg_id in self._received_acks:
                    self.logger.info(f"[WiFi] ACK received for msg_id: {msg_id}")
                    return True

            if is_move_command:
                self.logger.warning(f"[WiFi] No ACK for CHESS MOVE {msg_id} (attempt {attempt}/{max_attempts})")
            else:
                self.logger.warning(f"[WiFi] No ACK for {msg_id} (attempt {attempt}/{max_attempts})")

        self.logger.error(f"[WiFi] Broadcast failed after {max_attempts} retries for payload: {payload}")
        return False

    def _broadcast_unreliable(self, payload: dict):
        print(f"BROADCASTING UNRELIABLE: {payload}")
        return self._send_json(payload)



    def update_role(self, is_leader: bool):
        # print(f"UPDATING ROLE", is_leader)
        if self.is_leader != is_leader:
            self.is_leader = is_leader
            self.logger.info(f"[WiFi] Role updated: is_leader={self.is_leader}")
    
    def ping_drone(self, target_drone: str, retries: int = 5, timeout: float = 1.0) -> bool:
        """
        Пингует конкретный дрон и проверяет ответ с ретраями
        
        Args:
            target_drone: Имя целевого дрона
            retries: Количество попыток (по умолчанию 5)
            timeout: Таймаут ожидания ответа в секундах (по умолчанию 1.0)
        """
        for attempt in range(retries):
            ping_id = uuid.uuid4().hex[:8]
            
            # Очищаем старые ответы от этого дрона
            with self._ping_lock:
                if target_drone in self._ping_responses:
                    del self._ping_responses[target_drone]
            
            # Отправляем ping
            ping_payload = {
                'type': 'ping',
                'ping_id': ping_id,
                'to': target_drone,
                'from': self.drone_name
            }
            
            if not self._send_json(ping_payload):
                self.logger.debug(f"[WiFi] Failed to send ping to {target_drone} (attempt {attempt + 1}/{retries})")
                continue
            
            # Ждем ответ
            time.sleep(timeout)
            
            # Проверяем ответ
            with self._ping_lock:
                if target_drone in self._ping_responses:
                    if attempt > 0:  # Логируем только если были неудачные попытки
                        self.logger.info(f"[WiFi] Ping to {target_drone} succeeded on attempt {attempt + 1}/{retries}")
                    return True
            
            if attempt < retries - 1:  # Не логируем последнюю неудачную попытку
                self.logger.debug(f"[WiFi] No ping response from {target_drone} (attempt {attempt + 1}/{retries})")
        
        self.logger.warning(f"[WiFi] Drone {target_drone} is not responding to ping after {retries} attempts")
        return False
    
    def ping_all_drones(self, drone_list: list) -> list:
        """Пингует всех дронов и возвращает список живых"""
        alive_drones = []
        
        for drone_name in drone_list:
            if drone_name == self.drone_name:
                # Себя считаем живым
                alive_drones.append(drone_name)
                continue
                
            if self.ping_drone(drone_name):
                alive_drones.append(drone_name)
                self.logger.debug(f"[WiFi] Drone {drone_name} is alive (ping successful)")
            else:
                self.logger.warning(f"[WiFi] Drone {drone_name} is not responding to ping")
        
        return alive_drones
    
    def broadcast_move_start(self, move_info: str):
        """Уведомляет всех дронов о начале хода лидера"""
        if not self.is_leader:
            return False
        
        payload = {
            'type': 'move_start',
            'move': move_info,
            'from': self.drone_name,
            'timestamp': time.time()
        }
        
        self.logger.info(f"[WiFi] Broadcasting move start: {move_info}")
        return self._broadcast_unreliable(payload)
    
    def broadcast_move_complete(self, move_info: str, success: bool = True):
        """Уведомляет всех дронов о завершении хода лидера"""
        if not self.is_leader:
            return False
        
        payload = {
            'type': 'move_complete',
            'move': move_info,
            'success': success,
            'from': self.drone_name,
            'timestamp': time.time()
        }
        
        self.logger.info(f"[WiFi] Broadcasting move complete: {move_info} (success={success})")
        return self._broadcast_unreliable(payload)
    
    def is_leader_moving(self) -> bool:
        """Проверяет, выполняет ли лидер сейчас ход"""
        with self._move_state_lock:
            if not self._leader_move_state['is_moving']:
                return False
            
            # Проверяем таймаут
            elapsed = time.time() - self._leader_move_state['move_start_time']
            if elapsed > self._leader_move_state['move_timeout']:
                self.logger.warning(f"[WiFi] Leader move timeout exceeded ({elapsed:.1f}s > {self._leader_move_state['move_timeout']:.1f}s)")
                # Сбрасываем состояние при таймауте
                self._leader_move_state['is_moving'] = False
                self._leader_move_state['move_start_time'] = 0
                self._leader_move_state['current_move'] = None
                return False
            
            return True
    
    def get_current_move_info(self) -> dict:
        """Возвращает информацию о текущем ходе лидера"""
        with self._move_state_lock:
            return {
                'is_moving': self._leader_move_state['is_moving'],
                'move': self._leader_move_state['current_move'],
                'elapsed_time': time.time() - self._leader_move_state['move_start_time'] if self._leader_move_state['is_moving'] else 0
            }


def create_comm_controller(swarm, drone_name=None):
    impl = os.environ.get('COMM_IMPL', 'esp').lower()
    print(f"INITIALIZING COMM_IMPL: {impl}")

    if impl == 'wifi':
        return WifiEspController(swarm=swarm, drone_name=drone_name)
    return EspController(swarm=swarm, drone_name=drone_name)


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