import json
import time
import uuid
import os
import threading
import socket

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
        
        # Heartbeat/лидерство
        self._hb_lock = threading.Lock()
        self._last_heartbeat = None  # dict: {leader, term, ts, active}
        self._leader_term = 0
        
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
        
        # Heartbeat от лидера
        elif cmd_type == 'hb':
            leader = obj.get('leader')
            term = int(obj.get('term', 0))
            active = obj.get('active', [])
            with self._hb_lock:
                self._last_heartbeat = {
                    'leader': leader,
                    'term': term,
                    'ts': time.time(),
                    'active': active if isinstance(active, list) else []
                }
                # Сохраняем последний известный term
                self._leader_term = max(self._leader_term, term)
            # Никаких действий напрямую по смене роли здесь не делаем;
            # роль обновляется во внешней логике на основе таймаутов.

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

    def broadcast_heartbeat(self, leader_name: str, term: int, active_drones: list):
        payload = {
            'type': 'hb',
            'leader': leader_name,
            'term': int(term),
            'active': list(active_drones or [])
        }
        self._broadcast_unreliable(payload)

    def get_last_heartbeat(self):
        with self._hb_lock:
            # Возвращаем копию, чтобы снаружи не меняли
            return dict(self._last_heartbeat) if self._last_heartbeat else None

    def get_last_heartbeat_ts(self) -> float:
        with self._hb_lock:
            return self._last_heartbeat.get('ts') if self._last_heartbeat else 0.0

    def get_known_leader_and_term(self):
        with self._hb_lock:
            leader = self._last_heartbeat.get('leader') if self._last_heartbeat else None
            term = self._leader_term
            return leader, term

    def bump_term(self) -> int:
        with self._hb_lock:
            self._leader_term += 1
            return self._leader_term

    def update_role(self, is_leader: bool):
        if self.is_leader != is_leader:
            self.is_leader = is_leader
            self.logger.info(f"ESP role updated: is_leader={self.is_leader}")


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

        # Ack/Move/Heartbeat состояния — совместимы с EspController
        self._ack_lock = threading.Lock()
        self._received_acks = set()
        self._chess_move_event = threading.Event()
        self._received_chess_move = None
        self._expected_followers = set()
        self._hb_lock = threading.Lock()
        self._last_heartbeat = None
        self._leader_term = 0

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
        print(f"RECEIVED MESSAGE: {message}")

        try:
            obj = json.loads(message)
        except Exception:
            return

        cmd_type = obj.get('type')

        # Ack
        if cmd_type == 'ack':
            if self.is_leader:
                ack_id = obj.get('ack_id')
                with self._ack_lock:
                    self._received_acks.add(ack_id)
            return

        # Фильтр адресата
        target = obj.get('to', '*')
        if target != '*' and target != self.drone_name and target != drone_name_to_short(self.drone_name):
            return

        # Отправляем ack (не лидер)
        if 'msg_id' in obj and not self.is_leader:
            ack_payload = {
                'type': 'ack',
                'ack_id': obj['msg_id'],
                'from': self.drone_name
            }
            self._send_json(ack_payload)

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

        elif cmd_type == 'hb':
            leader = obj.get('leader')
            term = int(obj.get('term', 0))
            active = obj.get('active', [])
            with self._hb_lock:
                self._last_heartbeat = {
                    'leader': leader,
                    'term': term,
                    'ts': time.time(),
                    'active': active if isinstance(active, list) else []
                }
                self._leader_term = max(self._leader_term, term)

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
        print(f"SENDING JSON: {payload}")
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

    def broadcast_heartbeat(self, leader_name: str, term: int, active_drones: list):
        print(f"BROADCASTING HEARTBEAT: {leader_name}, {term}, {active_drones}")
        payload = {
            'type': 'hb',
            'leader': leader_name,
            'term': int(term),
            'active': list(active_drones or [])
        }
        self._broadcast_unreliable(payload)

    def get_last_heartbeat(self):
        print(f"GETTING LAST HEARTBEAT", self._last_heartbeat)
        with self._hb_lock:
            return dict(self._last_heartbeat) if self._last_heartbeat else None

    def get_last_heartbeat_ts(self) -> float:
        print(f"GETTING LAST HEARTBEAT TS", self._last_heartbeat.get('ts'))
        with self._hb_lock:
            return self._last_heartbeat.get('ts') if self._last_heartbeat else 0.0

    def get_known_leader_and_term(self):
        print(f"GETTING KNOWN LEADER AND TERM", leader, term)
        with self._hb_lock:
            leader = self._last_heartbeat.get('leader') if self._last_heartbeat else None
            term = self._leader_term
            return leader, term

    def bump_term(self) -> int:
        print(f"BUMPING TERM", self._leader_term)
        with self._hb_lock:
            self._leader_term += 1
            return self._leader_term

    def update_role(self, is_leader: bool):
        print(f"UPDATING ROLE", is_leader)
        if self.is_leader != is_leader:
            self.is_leader = is_leader
            self.logger.info(f"[WiFi] Role updated: is_leader={self.is_leader}")


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