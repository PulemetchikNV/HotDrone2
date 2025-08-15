import os
import time
import json
import math

# rospy fallback
try:
    import rospy
except ImportError:
    class MockRospy:
        def is_shutdown(self): return False
        def init_node(self, name): pass
        def sleep(self, t): time.sleep(t)
    rospy = MockRospy()

from flight import FlightController
from helpers import setup_logging
from alg2_stockfish import get_board_state, get_turn, update_after_execution, AlgTemporaryError, AlgPermanentError
import esp
from const import DRONE_LIST, LEADER_DRONE, rovers
from rover import RoverController
from camera import create_camera_controller

print(f"esp: {esp}")

try:
    from skyros.drone import Drone as SkyrosDrone
except Exception:
    SkyrosDrone = None


FILES = "abcdefgh"
RANKS = "12345678"

def cell_to_xy(cell: str, cell_size: float, origin_x: float, origin_y: float):
    f, r = cell[0], cell[1]
    fi = FILES.index(f)
    ri = RANKS.index(r)
    # a1 в нижнем левом углу; X вправо по файлам, Y вверх по рангам
    x = origin_x + (fi - 3.5) * cell_size
    y = origin_y + (ri - 3.5) * cell_size
    return x, y

def next_cell_simple(current: str) -> str:
    # Простой детерминированный генератор: идём по рангам вверх, затем вправо и снова снизу
    f_idx = FILES.index(current[0])
    r_idx = RANKS.index(current[1])
    if r_idx < 7:
        return f"{current[0]}{RANKS[r_idx+1]}"
    if f_idx < 7:
        return f"{FILES[f_idx+1]}1"
    return "a1"


 


class ChessDroneSingle:
    def __init__(self):
        self.drone_name = os.environ.get("DRONE_NAME", "drone")
        self.is_leader = self.drone_name == LEADER_DRONE
        self.logger = setup_logging(self.drone_name)

        self.swarm = None
        if SkyrosDrone is not None:
            try:
                self.swarm = SkyrosDrone(name=self.drone_name)
            except Exception as e:
                self.logger.warning(f"Skyros init failed: {e}")
                self.swarm = None

        # Не переопределяем FLIGHT_IMPL — используйте main/custom/mock через env
        self.fc = FlightController(drone_name=self.drone_name, logger=self.logger)
        
        # Контроллер связи (ESP по радиоканалу или Wi‑Fi) — выбирается по COMM_IMPL
        try:
            self.esp = esp.create_comm_controller(self.swarm, self.drone_name)
        except Exception as e:
            self.logger.warning(f"Comm controller init failed: {e}")
            self.esp = None
        
        # Регистрируем обработчик сообщений если есть swarm
        if self.swarm and self.esp:
            # Предполагаем что swarm имеет метод set_custom_message_callback
            if hasattr(self.swarm, 'set_custom_message_callback'):
                self.swarm.set_custom_message_callback(self.esp._on_custom_message)
                self.logger.info("ESP message handler registered")
        
        # Список доступных дронов для назначения задач (постоянный порядок)
        self.available_drones = [d.strip() for d in (DRONE_LIST.split(',') if DRONE_LIST else []) if d.strip()]
        self.logger.info(f"Available drones: {self.available_drones}")

        # Параметры лидерства/heartbeat
        self.heartbeat_interval = float(os.getenv("HB_INTERVAL", "0.5"))
        self.heartbeat_timeout = float(os.getenv("HB_TIMEOUT", "1.5"))
        self._last_hb_sent = 0.0
        # Текущий лидер и term
        self.current_leader = LEADER_DRONE if LEADER_DRONE else (self.available_drones[0] if self.available_drones else self.drone_name)
        # Синхронизируемся с ESP при наличии
        if self.esp:
            leader, term = self.esp.get_known_leader_and_term()
            if leader:
                self.current_leader = leader
            self.current_term = int(term)
            self.esp.update_role(self.drone_name == self.current_leader)
        else:
            self.current_term = 0
        # Локальная оценка активных (пока предположение, уточняется heartbeat'ом лидера)
        self.active_drones = list(self.available_drones)

        # Роль текущего дрона (какую фигуру он представляет)
        self.drone_role = os.getenv("DRONE_ROLE", "").lower()
        # Цвет фигур, за которые играет дрон ('white' или 'black')
        self.drone_color = os.getenv("DRONE_COLOR", "white").lower()
        # Начальная буква/клетка для данного дрона
        self.initial_letter = os.getenv("INITIAL_LETTER", "").lower()
        self.logger.info(f"Drone role: {self.drone_role}, color: {self.drone_color}, initial_letter: {self.initial_letter}")
        
        # Маппинг ролей дронов (строится динамически из доступных дронов и их ролей)
        self.drone_role_mapping = self._build_drone_role_mapping()

        # Параметры поля
        self.cell_size = float(os.getenv("CELL_SIZE_M", "0.35"))
        self.origin_x = float(os.getenv("BOARD_ORIGIN_X", "0.0"))  # центр доски = (0,0)
        self.origin_y = float(os.getenv("BOARD_ORIGIN_Y", "0.0"))

        # Параметры полёта
        self.takeoff_z = float(os.getenv("TAKEOFF_Z", "1.0"))
        self.flight_z = float(os.getenv("FLIGHT_Z", "1.0"))
        self.speed = float(os.getenv("SPEED", "0.3"))

        # CameraAdapter removed; use unified camera controller as single source of truth
        self.camera = create_camera_controller(self.logger)
        self.our_team = os.getenv("OUR_TEAM", os.getenv("DRONE_COLOR", "white")).lower()
        self.logger.info(f"Our team: {self.our_team}")

        # Контроллер роверов (пешки)
        self.rover = RoverController(logger=self.logger)
        self.rover_ids = list(rovers.keys()) if isinstance(rovers, dict) else []
        
        # Инициализируем позиции роверов из их начальных клеток
        self._initialize_rover_positions()

        # Загрузка карты ArUco для клеток (по умолчанию aruco_maps/aruco_map2.json)
        default_map_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "aruco_maps",
            "aruco_map2.json",
        )
        self.map_path = os.getenv("ARUCO_MAP_JSON", default_map_path)
        self.cell_markers = {}
        try:
            with open(self.map_path, "r") as f:
                data = json.load(f)
                if isinstance(data, dict):
                    self.cell_markers = data
                else:
                    self.logger.warning(f"Unexpected JSON structure in {self.map_path}")
        except Exception as e:
            self.logger.warning(f"Failed to load ArUco map from {self.map_path}: {e}")

    def get_cell_coordinates(self, cell):
        """Получает координаты клетки из карты или вычисляет их"""
        marker = self.cell_markers.get(cell)
        if isinstance(marker, dict) and "x" in marker and "y" in marker:
            x, y = float(marker["x"]), float(marker["y"])
            self.logger.info(f"Using map coords for {cell}: x={x:.3f}, y={y:.3f}")
            return x, y
        else:
            x, y = cell_to_xy(cell, self.cell_size, self.origin_x, self.origin_y)
            self.logger.warning(f"No map coords for {cell}. Using computed coords: x={x:.3f}, y={y:.3f}")
            return x, y
    
    def _initialize_rover_positions(self):
        """Инициализирует позиции роверов на основе их начальных клеток"""
        for rover_id, rover_config in rovers.items():
            initial_letter = rover_config.get('initial_letter', '')
            if initial_letter:
                # Определяем начальную клетку для белых пешек (2-я линия)
                initial_cell = f"{initial_letter}2"
                try:
                    x, y = self.get_cell_coordinates(initial_cell)
                    # Обновляем позицию ровера в контроллере
                    if hasattr(self.rover, 'set_rover_position'):
                        self.rover.set_rover_position(rover_id, x, y, 0)
                    self.logger.info(f"Rover {rover_id} initialized at {initial_cell} -> ({x:.3f}, {y:.3f})")
                except Exception as e:
                    self.logger.warning(f"Failed to initialize rover {rover_id} position: {e}")
            else:
                self.logger.warning(f"No initial_letter defined for rover {rover_id}")
    
    def _get_rover_current_position(self, rover_id: str) -> tuple:
        """Получает текущую позицию ровера"""
        if hasattr(self.rover, 'rover_positions') and rover_id in self.rover.rover_positions:
            pos = self.rover.rover_positions[rover_id]
            return pos.get('x', 0), pos.get('y', 0), pos.get('yaw', 0)
        return 0, 0, 0
    
    def _update_rover_position(self, rover_id: str, x: float, y: float, yaw: float = 0):
        """Обновляет позицию ровера после движения"""
        if hasattr(self.rover, 'set_rover_position'):
            self.rover.set_rover_position(rover_id, x, y, yaw)
            self.logger.info(f"Updated rover {rover_id} position to ({x:.3f}, {y:.3f}, {yaw:.1f}°)")

    def move_to_xy(self, x: float, y: float, z: float):
        
        # 1. Взлёт на рабочую высоту
        self.fc.takeoff(z=self.takeoff_z, delay=2, speed=0.5)
        time.sleep(3)

        self.fc.navigate_wait(
            x=x,
            y=y,
            z=self.flight_z,
            speed=self.speed,
            auto_arm=True,
        )
        self.fc.wait(0.5)
        
        self.fc.navigate_wait(
            x=x, 
            y=y, 
            z=0.20,
            speed=0.2,
            auto_arm=False,
        )


        self.fc.wait(0.4)

        self.fc.force_disarm()

        # self.fc.land()
        
        # 5. Финальное зависание перед завершением
        # self.fc.wait(0.5)

    def run_once(self, current_turn):
        """Выполняется только на лидере - получает ход и отправляет команду исполнителю"""
        if not self.esp or not self.esp.is_leader:
            self.logger.warning("run_once called on non-leader drone")
            return
            
        # 1) Получаем состояние доски через alg (источник данных — камера внутри alg)
        board = get_board_state()
        print(f"GOT BOARD: {board}")
        
        # Проверяем, чей ход (предпочитаем камеру)
        camera_turn = None
        try:
            if hasattr(self.camera, 'get_board_positions'):
                # обновим внутреннее состояние камеры, чтобы получить turn
                self.camera.get_board_positions()
            if hasattr(self.camera, 'get_turn'):
                camera_turn = self.camera.get_turn()
        except Exception as e:
            self.logger.debug(f"Failed to fetch camera turn: {e}")

        current_player = (camera_turn or getattr(board, 'turn', 'white')).lower()
        if current_player != self.our_team:
            self.logger.info(f"Not our turn. Current turn is {current_player}, we are {self.our_team}. Waiting.")
            return
        else:
            print(f"OUR TURN: {current_turn}")

        # 2) Запрашиваем ход (внутри alg решается логика)
        move = get_turn(board, time_budget_ms=5000)
        print(f"GOT MOVE: {move}")

        from_cell = getattr(move, 'from_cell', 'e2')
        to_cell = getattr(move, 'to_cell', 'e4')
        self.logger.info(f"MOVING {from_cell}->{to_cell}")
        
        # 3) Определяем исполнителя по фигуре на from_cell через камеру
        is_pawn_camera, pawn_id = self._is_pawn_move_by_camera(from_cell)
        
        if is_pawn_camera and self.rover_ids:
            # Пешка -> выполняем ход ровером
            self.logger.info(f"Executing pawn move {from_cell}->{to_cell} with rover")
            self._execute_rover_move(move)
            self.logger.info(f"Leader completed rover move: {from_cell}->{to_cell}")
        else:
            # Не пешка -> выполняем ход дроном
            self.logger.info(f"Executing piece move {from_cell}->{to_cell} with drone")
            self._execute_drone_move(move, current_turn)
            self.logger.info(f"Leader completed drone move: {from_cell}->{to_cell}")
    
    def _build_drone_role_mapping(self):
        """Строит маппинг ролей дронов из переменных окружения"""
        mapping = {}
        
        for drone_name in self.available_drones:
            # Получаем роль и начальную букву/клетку для каждого дрона
            role_var = f"DRONE_ROLE_{drone_name.upper()}"
            initial_var = f"INITIAL_LETTER_{drone_name.upper()}"
            
            role = os.getenv(role_var, "").lower()
            initial = os.getenv(initial_var, "").lower()
            
            if role:
                # Для уникальных фигур (король, ферзь) используем только роль
                if role in ['king', 'queen']:
                    mapping[role] = drone_name
                    self.logger.info(f"Drone {drone_name} mapped to unique role: {role}")
                elif initial:
                    # Для остальных фигур используем формат role_initial
                    full_role = f"{role}_{initial}"
                    mapping[full_role] = drone_name
                    self.logger.info(f"Drone {drone_name} mapped to role: {full_role}")
                else:
                    self.logger.warning(f"Drone {drone_name} has role {role} but no initial letter (env var {initial_var})")
            else:
                self.logger.warning(f"No role defined for drone {drone_name} (env var {role_var})")
        
        return mapping
    
    def _get_piece_on_cell_from_camera(self, cell: str):
        """Получает тип фигуры на указанной клетке из данных камеры"""
        try:
            positions = self.camera.get_board_positions()
            
            for color_data in positions.values():
                for piece_type, piece_pos in color_data.items():
                    if piece_pos.cell == cell:
                        return piece_type.lower()
            
            return None
            
        except Exception as e:
            self.logger.debug(f"Failed to get piece from camera for cell {cell}: {e}")
            return None
    
    def _find_drone_for_piece(self, piece_type: str):
        """Находит дрон, ответственный за данный тип фигуры"""
        if not piece_type:
            return None
        
        piece_type = piece_type.lower()
        
        # Прямое соответствие для уникальных фигур (king, queen)
        if piece_type in self.drone_role_mapping:
            return self.drone_role_mapping[piece_type]
        
        # Для фигур в формате {тип}_{начальная_клетка}
        if '_' in piece_type:
            # Пытаемся найти точное соответствие сначала
            if piece_type in self.drone_role_mapping:
                return self.drone_role_mapping[piece_type]
            
            # Если не найдено, пытаемся извлечь базовый тип и начальную клетку
            parts = piece_type.split('_', 1)
            if len(parts) == 2:
                base_type, initial_cell = parts
                
                # Формируем ключ для поиска в маппинге
                full_role_key = f"{base_type}_{initial_cell}"
                if full_role_key in self.drone_role_mapping:
                    return self.drone_role_mapping[full_role_key]
                
                # Также пробуем с только буквой файла (для совместимости)
                if len(initial_cell) >= 1:
                    file_letter = initial_cell[0]
                    full_role_key_short = f"{base_type}_{file_letter}"
                    if full_role_key_short in self.drone_role_mapping:
                        return self.drone_role_mapping[full_role_key_short]
        
        # Fallback: первый доступный дрон
        self.logger.warning(f"No specific drone found for piece {piece_type}, using fallback")
        return self.available_drones[0] if self.available_drones else None

    def _execute_drone_move(self, move, current_turn):
        """Выполняет ход дрона через ESP или локально"""
        from_cell = getattr(move, 'from_cell', 'e2')
        to_cell = getattr(move, 'to_cell', 'e4')
        
        # Определяем фигуру на исходной клетке через камеру
        piece_type = self._get_piece_on_cell_from_camera(from_cell)
        print(f"PIECE TYPE: {piece_type}")
        if not piece_type:
            self.logger.warning(f"No piece found on {from_cell} by camera, using fallback")
            piece_type = "king"  # fallback
        
        # Находим дрон, ответственный за эту фигуру
        target_drone = self._find_drone_for_piece(piece_type)
        
        if not target_drone:
            self.logger.error(f"No drone found for piece {piece_type} on {from_cell}")
            return
        
        chess_move = f"{from_cell}->{to_cell}"
        self.logger.info(f"Piece {piece_type} on {from_cell} -> drone {target_drone}")
        
        # Если целевой дрон - это мы сами (лидер), выполняем ход локально
        if target_drone == self.drone_name:
            self.logger.info(f"Leader executing own move: {chess_move}")
            try:
                # Получаем координаты целевой клетки
                x, y = self.get_cell_coordinates(to_cell)
                # Выполняем движение локально
                self.move_to_xy(x, y, self.flight_z)
                self.logger.info(f"Leader completed own move to {to_cell}")
            except Exception as e:
                self.logger.error(f"Leader failed to execute own move: {e}")
        else:
            # Отправляем команду другому дрону через ESP
            self.logger.info(f"Leader sending chess move: {chess_move} to drone {target_drone}")
            if self.esp and self.esp.is_leader:
                # Формируем сообщение хода самостоятельно (без esp.create_chess_move_message)
                payload = {
                    'type': 'move',
                    'to': target_drone,
                    'move': chess_move,
                }
                success = self.esp._broadcast_reliable(payload)
                if success:
                    self.logger.info(f"Successfully sent move command to {target_drone}")
                else:
                    self.logger.error(f"Failed to send move command to {target_drone}")
            else:
                self.logger.warning("No ESP controller available for remote drone command")

    def _fen_piece_at(self, fen: str, cell: str):
        try:
            board_part = fen.split()[0]
            ranks = board_part.split('/')
            file_char, rank_char = cell[0], cell[1]
            file_idx = FILES.index(file_char)
            rank_idx = 8 - int(rank_char)
            row = ranks[rank_idx]
            col = 0
            for ch in row:
                if ch.isdigit():
                    col += int(ch)
                else:
                    if col == file_idx:
                        return ch
                    col += 1
            return None
        except Exception:
            return None

    def _is_pawn_move(self, fen: str, from_cell: str) -> bool:
        if not fen or not from_cell or len(from_cell) != 2:
            return False
        piece = self._fen_piece_at(fen, from_cell)
        return piece in ('P', 'p')
    
    def _is_pawn_move_by_camera(self, from_cell: str) -> tuple:
        """
        Определяет, является ли ход пешкой на основе данных камеры.
        
        Returns:
            tuple: (is_pawn, initial_cell) где is_pawn - bool, initial_cell - str или None
        """
        try:
            positions = self.camera.get_board_positions()
            
            for color_data in positions.values():
                for piece_type, piece_pos in color_data.items():
                    if piece_pos.cell == from_cell and piece_type.startswith('pawn_'):
                        # Извлекаем начальную клетку из piece_type (например, "pawn_a2" -> "a2")
                        initial_cell = piece_type.replace('pawn_', '')
                        return True, initial_cell
            
            return False, None
            
        except Exception as e:
            self.logger.debug(f"Camera pawn detection failed: {e}")
            return False, None

    def _pick_rover_id_for_pawn(self, initial_cell: str) -> str:
        """
        Сопоставляет начальную клетку пешки с ID ровера на основе маппинга.
        
        Args:
            initial_cell: Начальная клетка пешки из камеры (например, "a2", "b2", "c7")
            
        Returns:
            str: ID ровера из rovers dict или None
        """
        if not self.rover_ids or not initial_cell:
            return None
        
        # Извлекаем букву файла из клетки (a2 -> a, b7 -> b)
        file_letter = initial_cell[0].lower() if initial_cell else ""
        
        # Ищем ровер с соответствующей initial_letter
        for rover_id, rover_config in rovers.items():
            rover_initial = rover_config.get('initial_letter', '').lower()
            if rover_initial == file_letter:
                return rover_id
        
        # Fallback: если не найден по букве, используем первый доступный
        self.logger.warning(f"No rover found for initial cell {initial_cell}, using fallback")
        return self.rover_ids[0] if self.rover_ids else None

    def _pick_rover_id_for_cell(self, cell: str) -> str:
        """Fallback метод для определения ровера по клетке (старая логика)"""
        if not self.rover_ids:
            return None
        try:
            file_idx = FILES.index(cell[0])
        except Exception:
            file_idx = 0
        # Делим 8 файлов на количество доступных роверов равными сегментами
        n = max(1, len(self.rover_ids))
        seg = max(1, 8 // n)
        bucket = min(n - 1, file_idx // seg)
        return self.rover_ids[bucket]

    def _execute_rover_move(self, move):
        # Сначала пытаемся определить конкретную пешку по камере
        is_pawn_camera, initial_cell = self._is_pawn_move_by_camera(move.from_cell)
        
        if is_pawn_camera and initial_cell:
            # Используем точное сопоставление пешки с ровером по начальной клетке
            rover_id = self._pick_rover_id_for_pawn(initial_cell)
            self.logger.info(f"Camera detected pawn_{initial_cell} -> rover {rover_id}")
        else:
            # Fallback: определяем ровер по клетке (старая логика)
            rover_id = self._pick_rover_id_for_cell(move.from_cell)
            self.logger.info(f"Using fallback rover selection for cell {move.from_cell} -> rover {rover_id}")
        
        if not rover_id:
            self.logger.error("No rover available to execute pawn move")
            return
        
        # Получаем текущую позицию ровера
        current_x, current_y, current_yaw = self._get_rover_current_position(rover_id)
        
        # Получаем координаты целевой клетки (абсолютные в метрах)
        target_x, target_y = self.get_cell_coordinates(move.to_cell)
        
        self.logger.info(f"Rover {rover_id} move: {move.from_cell}->{move.to_cell}")
        self.logger.info(f"  From position: ({current_x:.3f}, {current_y:.3f}, {current_yaw:.1f}°)")
        self.logger.info(f"  To position: ({target_x:.3f}, {target_y:.3f})")
        
        try:
            # Отправляем команду с текущей и целевой позицией
            success = self.rover.navigate(
                rover_id, 
                current_x=current_x, 
                current_y=current_y, 
                current_yaw=current_yaw,
                x=target_x, 
                y=target_y
            )
            
            if success:
                # Обновляем сохранённую позицию ровера
                self._update_rover_position(rover_id, target_x, target_y, 0)
                self.logger.info(f"Rover {rover_id} command sent successfully")
            else:
                self.logger.error(f"Failed to send command to rover {rover_id}")
                
        except Exception as e:
            self.logger.error(f"Failed to send rover navigate: {e}")

    def wait_and_execute_move(self, timeout=0.5):
        """Выполняется на ведомых дронах - ждет команду от лидера и выполняет её"""
        if self.esp and self.esp.is_leader:
            self.logger.warning("wait_and_execute_move called on leader drone")
            return False
            
        if not self.esp:
            self.logger.error("No ESP controller available for follower")
            return False
            
        self.logger.info(f"Follower {self.drone_name} waiting for chess move command...")
        
        # Ждем команду от лидера
        if self.esp._chess_move_event.wait(timeout):
            # Получили команду
            chess_move = self.esp._received_chess_move
            self.esp._chess_move_event.clear()  # Сбрасываем событие
            self.esp._received_chess_move = None
            
            self.logger.info(f"Received chess move command: {chess_move}")
            
            # Парсим команду
            # Парсим ход формата "c2->d4"
            try:
                from_cell, to_cell = chess_move.split('->', 1)
                from_cell = from_cell.strip()
                to_cell = to_cell.strip()
            except Exception:
                from_cell, to_cell = None, None
            if not to_cell:
                self.logger.error(f"Invalid chess move format: {chess_move}")
                return False
                
            # Получаем координаты целевой клетки
            x, y = self.get_cell_coordinates(to_cell)
            
            # Выполняем движение
            try:
                self.logger.info(f"Flying to {to_cell} at coordinates ({x:.3f}, {y:.3f})")
                self.move_to_xy(x, y, self.flight_z)
                self.logger.info(f"Successfully completed move to {to_cell}")
                return True
            except Exception as e:
                self.logger.error(f"Failed to execute move: {e}")
                return False
        else:
            self.logger.warning(f"Timeout waiting for chess move command ({timeout}s)")
            return False

    def _order_index(self, name: str) -> int:
        try:
            return self.available_drones.index(name)
        except ValueError:
            return len(self.available_drones) + 1

    def _next_active_leader(self, current: str, active: list) -> str:
        if not active:
            return None
        # Ищем следующего по постоянному порядку в active
        order = self.available_drones
        if current in order:
            start = (order.index(current) + 1) % len(order)
        else:
            start = 0
        for i in range(len(order)):
            idx = (start + i) % len(order)
            cand = order[idx]
            if cand in active:
                return cand
        return None

    def _estimate_active_drones(self) -> list:
        # Интеграция детектирования активности по камере
        try:
            # Получаем данные с камеры о позициях фигур
            positions = self.camera.get_board_positions()
            
            # Определяем активные дроны на основе видимых фигур
            # Пешки управляются роверами, остальные фигуры — дронами
            active_drones = set()
            
            for color_data in positions.values():
                for piece_type, piece_pos in color_data.items():
                    # Пешки не считаем (они управляются роверами)
                    # Поддерживаем как старый формат 'pawn', так и новый 'pawn_X'
                    if piece_type.lower() == 'pawn' or piece_type.lower().startswith('pawn_'):
                        continue
                    
                    # Для остальных фигур определяем дрон по позиции/типу
                    # Простая логика: разные типы фигур = разные дроны
                    drone_mapping = {
                        'king': 0, 'queen': 1, 'rook': 2, 'knight': 3, 'bishop': 4
                    }
                    
                    drone_idx = drone_mapping.get(piece_type.lower(), 0)
                    if drone_idx < len(self.available_drones):
                        active_drones.add(self.available_drones[drone_idx])
            
            if active_drones:
                # Возвращаем в порядке DRONE_LIST
                ordered = [d for d in self.available_drones if d in active_drones]
                self.logger.debug(f"Camera detected active drones: {ordered}")
                return ordered
                
        except Exception as e:
            self.logger.debug(f"Camera-based activity detection failed: {e}")
        
        # Fallback: используем heartbeat или весь список
        last_hb = self.esp.get_last_heartbeat() if self.esp else None
        if last_hb and isinstance(last_hb.get('active'), list) and last_hb['active']:
            # Гарантируем порядок согласно DRONE_LIST
            ordered = [d for d in self.available_drones if d in last_hb['active']]
            return ordered if ordered else list(self.available_drones)
        return list(self.available_drones)

    def _tick_leader_election(self):
        now = time.time()
        if not self.esp:
            return
        last_hb_ts = self.esp.get_last_heartbeat_ts()
        last_hb = self.esp.get_last_heartbeat()
        known_leader, known_term = self.esp.get_known_leader_and_term()

        # Принятие более свежего лидера
        if last_hb and int(known_term) >= int(getattr(self, 'current_term', 0)):
            self.current_term = int(known_term)
            if known_leader:
                self.current_leader = known_leader
                self.esp.update_role(self.drone_name == self.current_leader)

        # Лидер считается потерянным, если heartbeat старее таймаута
        if (now - last_hb_ts) > self.heartbeat_timeout:
            active = (last_hb.get('active') if last_hb else None) or list(self.available_drones)
            candidate = self._next_active_leader(self.current_leader, active) or (active[0] if active else self.drone_name)
            # Детерминированная задержка по позиции, чтобы снизить коллизии выборов
            delay = 0.05 * self._order_index(self.drone_name)
            if (now - last_hb_ts) > (self.heartbeat_timeout + delay):
                if self.drone_name == candidate:
                    new_term = self.esp.bump_term()
                    self.current_term = int(new_term)
                    self.current_leader = self.drone_name
                    self.esp.update_role(True)
                    self.active_drones = self._estimate_active_drones()
                    self.esp.broadcast_heartbeat(self.drone_name, self.current_term, self.active_drones)
                    self._last_hb_sent = now

    def run(self):
        try:
            rospy.init_node("chess_drone_single")
        except Exception:
            pass

        self.logger.info("Starting chess drone MAIN LOOP (dynamic leadership)...")

        TURN_LIMIT = 5
        turn_count = 0
        last_move_ts = 0.0
        move_interval = 5.0

        while not rospy.is_shutdown():
            try:
                # Обработка лидерства/heartbeat
                self._tick_leader_election()

                now = time.time()
                if self.esp and self.esp.is_leader:
                    # Периодический heartbeat
                    if (now - self._last_hb_sent) >= self.heartbeat_interval:
                        self.active_drones = self._estimate_active_drones()
                        self.esp.broadcast_heartbeat(self.drone_name, self.current_term, self.active_drones)
                        self._last_hb_sent = now

                    # Ходы лидера с паузой между ними
                    if turn_count < TURN_LIMIT and (now - last_move_ts) >= move_interval:
                        self.logger.info(f"=== Leader executing turn {turn_count + 1} ===")
                        # Перед началом ещё раз убедимся, что мы лидер
                        if self.esp.is_leader:
                            self.run_once(turn_count)
                            turn_count += 1
                            last_move_ts = time.time()
                        else:
                            self.logger.info("Leadership lost before executing turn; skipping")
                    elif turn_count >= TURN_LIMIT:
                        # После лимита — просто жить как лидер и рассылать heartbeats
                        pass
                else:
                    # Ведомый: ждём команды небольшими таймаутами, чтобы не блокировать выборы
                    self.wait_and_execute_move(timeout=0.3)

                # Небольшая пауза цикла
                self.fc.wait(0.05)

            except KeyboardInterrupt:
                break
            except Exception as e:
                self.logger.exception("main loop error")
                self.fc.wait(0.3)
        self.logger.info("Main loop stopped.")


if __name__ == "__main__":
    # Для использования с swarm коммуникацией:
    # chess_drone = ChessDroneSingle(swarm=your_swarm_instance)
    # chess_drone.run()
    
    # Для автономного запуска без swarm:
    ChessDroneSingle().run()

"""
Использование:
Логика работы:
   - Лидер получает ходы от алгоритма и отправляет команды ведомым
   - Ведомые ждут команды формата "c2->d4" и летят в указанную клетку
   - Координаты берутся из aruco_map2.json или вычисляются
   - Коммуникация через ESP контроллер с подтверждениями
"""
