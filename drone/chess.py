import os
import time
import json
import math
import requests

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
from utils import AlgTemporaryError, AlgPermanentError
from board_utils import get_board_state
from alg2_stockfish import get_turn as get_turn_sf
from alg_mock2 import get_turn as get_turn_mock
from esp import EspController, create_comm_controller
from const import DRONE_LIST, LEADER_DRONE, rovers, get_current_drone_config, get_drone_config
from rover import RoverController
from camera import create_camera_controller

try:
    from skyros.drone import Drone as SkyrosDrone
except Exception:
    SkyrosDrone = None


FILES = "abcdefgh"
RANKS = "12345678"

# Константы для конвертации координат
CELL_SIZE_METERS = 0.30  # 30 см = 1 клетка
ARUCO_MIN = -1.5  # Минимальная координата ArUco
ARUCO_MAX = 1.5   # Максимальная координата ArUco
ARUCO_RANGE = ARUCO_MAX - ARUCO_MIN  # 3.0 метра = 10 клеток

def aruco_to_rover_meters(aruco_x, aruco_y):
    """
    Конвертирует координаты из ArUco формата (-1.5 до 1.5) в метры для роверов.
    ArUco система: центр (0,0), края ±1.5 метра
    Rover система: начинается с (0,0), в метрах относительно начальной позиции
    
    Args:
        aruco_x, aruco_y: Координаты в ArUco формате
        
    Returns:
        tuple: (rover_x, rover_y) в метрах
    """
    # Сдвигаем начало координат: ArUco (-1.5, -1.5) становится rover (0, 0)
    rover_x = aruco_x - ARUCO_MIN  # -1.5 -> 0, 0 -> 1.5, 1.5 -> 3.0
    rover_y = aruco_y - ARUCO_MIN  # -1.5 -> 0, 0 -> 1.5, 1.5 -> 3.0
    
    return rover_x, rover_y

def rover_meters_to_aruco(rover_x, rover_y):
    """
    Конвертирует координаты из метров роверов обратно в ArUco формат.
    
    Args:
        rover_x, rover_y: Координаты в метрах ровера
        
    Returns:
        tuple: (aruco_x, aruco_y) в ArUco формате
    """
    aruco_x = rover_x + ARUCO_MIN  # 0 -> -1.5, 1.5 -> 0, 3.0 -> 1.5
    aruco_y = rover_y + ARUCO_MIN  # 0 -> -1.5, 1.5 -> 0, 3.0 -> 1.5
    
    return aruco_x, aruco_y


TIME_BUDGET_MS = 15000

def get_turn_final(board, time_budget_ms: int = TIME_BUDGET_MS):
    """
    Универсальная функция выбора алгоритма для получения хода.
    
    Args:
        board: BoardState объект с текущим состоянием доски
        time_budget_ms: время на расчёт хода в миллисекундах
        
    Returns:
        MoveDecision объект с выбранным ходом
    """
    alg_mode = os.getenv("ALG_MODE", "api").lower()
    if alg_mode.startswith("cluster"):
        return get_turn_sf(board, time_budget_ms=time_budget_ms)
    else:
        # api и llm пока одинаково — используем мок-алгоритм
        return get_turn_mock(board, time_budget_ms=time_budget_ms)

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
            self.esp = create_comm_controller(self.swarm, self.drone_name)
        except Exception:
            # Фолбэк на старую реализацию
            self.esp = EspController(swarm=self.swarm, drone_name=self.drone_name) if self.swarm else None
        
        # Регистрируем обработчик сообщений если есть swarm
        if self.swarm and self.esp:
            # Предполагаем что swarm имеет метод set_custom_message_callback
            if hasattr(self.swarm, 'set_custom_message_callback'):
                self.swarm.set_custom_message_callback(self.esp._on_custom_message)
                self.logger.info("ESP message handler registered")
        
        # Список доступных дронов для назначения задач (постоянный порядок)
        self.available_drones = [d.strip() for d in (DRONE_LIST.split(',') if DRONE_LIST else []) if d.strip()]
        self.logger.info(f"Available drones: {self.available_drones}")

        # Текущий лидер (упрощённая логика без heartbeat/term)
        self.current_leader = LEADER_DRONE if LEADER_DRONE else (self.available_drones[0] if self.available_drones else self.drone_name)
        # Устанавливаем роль в ESP контроллере
        if self.esp:
            self.esp.update_role(self.drone_name == self.current_leader)

        # Получаем конфигурацию дрона из централизованного словаря
        try:
            drone_config = get_current_drone_config()
            self.drone_role = drone_config['role'].lower()
            self.drone_color = drone_config['color'].lower()
            self.initial_letter = drone_config['initial_letter'].lower()
            self.drone_team = drone_config['team'].lower()
        except (ValueError, KeyError) as e:
            self.logger.warning(f"Failed to get drone config: {e}. Using defaults.")
            # Фолбэк на старые переменные окружения
            self.drone_role = os.getenv("DRONE_ROLE", "").lower()
            self.drone_color = os.getenv("DRONE_COLOR", "white").lower()
            self.initial_letter = os.getenv("INITIAL_LETTER", "").lower()
            self.drone_team = self.drone_color
        
        self.logger.info(f"Drone role: {self.drone_role}, color: {self.drone_color}, initial_letter: {self.initial_letter}, team: {self.drone_team}")
        
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
        self.our_team = self.drone_team
        self.logger.info(f"Our team: {self.our_team}")

        # Контроллер роверов (пешки)
        self.rover = RoverController(logger=self.logger)
        self.rover_ids = list(rovers.keys()) if isinstance(rovers, dict) else []
        
        # Роверы теперь получают позиции динамически с камеры

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
        
        # Для упрощенной логики выбора лидера
        self._is_our_turn_cache = False
        self._last_turn_check = 0.0

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
    
    def _is_enemy_piece_on_cell(self, cell: str) -> bool:
        """
        Проверяет, есть ли на указанной клетке вражеская фигура.
        
        Args:
            cell: Клетка для проверки (например, "e4")
            
        Returns:
            bool: True если на клетке есть вражеская фигура
        """
        try:
            positions = self.camera.get_board_positions()
            
            # Определяем цвет врага
            enemy_color = 'white' if self.our_team == 'black' else 'black'
            
            # Ищем фигуры противника на целевой клетке
            if enemy_color in positions:
                for piece_type, piece_pos in positions[enemy_color].items():
                    if piece_pos.cell == cell:
                        self.logger.info(f"Enemy {enemy_color} {piece_type} found on {cell} - this is a capture move")
                        return True
            
            self.logger.debug(f"No enemy piece found on {cell}")
            return False
            
        except Exception as e:
            self.logger.debug(f"Failed to check enemy piece on {cell}: {e}")
            return False

    def _get_rover_position_from_camera(self, rover_id: str, from_cell: str) -> tuple:
        """
        Получает текущую позицию ровера с камеры по его ID и начальной клетке.
        
        Args:
            rover_id: ID ровера
            from_cell: Клетка, на которой сейчас находится пешка
            
        Returns:
            tuple: (rover_x, rover_y, rover_yaw) в метрах для rover API
        """
        try:
            # Получаем позиции всех фигур с камеры
            positions = self.camera.get_board_positions()
            
            # Ищем пешку на указанной клетке
            for color_data in positions.values():
                for piece_type, piece_pos in color_data.items():
                    if piece_pos.cell == from_cell and piece_type.startswith('pawn_'):
                        # Получаем ArUco координаты из камеры
                        aruco_x, aruco_y = piece_pos.x, piece_pos.y
                        
                        # Конвертируем в метры для rover API
                        rover_x, rover_y = aruco_to_rover_meters(aruco_x, aruco_y)
                        
                        # Пока что используем yaw = 0 (можно расширить позже)
                        rover_yaw = 0.0
                        
                        self.logger.info(f"Rover {rover_id} at {from_cell}: ArUco({aruco_x:.3f}, {aruco_y:.3f}) -> Rover({rover_x:.3f}, {rover_y:.3f}, {rover_yaw:.1f}°)")
                        return rover_x, rover_y, rover_yaw
            
            # Если не нашли пешку на камере, возвращаем координаты клетки
            self.logger.warning(f"Pawn not found on camera at {from_cell}, using calculated coordinates")
            aruco_x, aruco_y = self.get_cell_coordinates(from_cell)
            rover_x, rover_y = aruco_to_rover_meters(aruco_x, aruco_y)
            return rover_x, rover_y, 0.0
            
        except Exception as e:
            self.logger.error(f"Failed to get rover position from camera: {e}")
            # Fallback: используем рассчитанные координаты клетки
            aruco_x, aruco_y = self.get_cell_coordinates(from_cell)
            rover_x, rover_y = aruco_to_rover_meters(aruco_x, aruco_y)
            return rover_x, rover_y, 0.0

    def move_to_xy(self, x: float, y: float, z: float, is_kill = False):
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
        self.fc.wait(0.5 if is_kill else 4.0)
        
        self.fc.navigate_wait(
            x=x, 
            y=y, 
            z=0.15,
            speed=0.2,
            auto_arm=False,
        )


        self.fc.wait(0.8)

        self.fc.force_disarm()

        # self.fc.land()
        
        # 5. Финальное зависание перед завершением
        # self.fc.wait(0.5)

    def run_once(self):
        """Выполняется только на лидере - получает ход и отправляет команду исполнителю"""
        if not self.esp or not self.esp.is_leader:
            self.logger.warning("run_once called on non-leader drone")
            return
        
        # 0) Проверяем, не стоит ли игра на паузе
        try:
            if self.camera and hasattr(self.camera, 'is_game_paused') and self.camera.is_game_paused():
                self.logger.debug("Game is paused, skipping move")
                return
        except Exception as e:
            self.logger.debug(f"Failed to check pause status: {e}")
            # Продолжаем если не можем проверить паузу
            
        # 1) Получаем состояние доски через alg (источник данных — камера внутри alg)
        board = get_board_state()
        print(f"GOT BOARD: {board.fen}")
        
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
            print(f"OUR TURN: {current_player}")

        # 1.5) Для кластерного режима передаем информацию о живых дронах
        if os.getenv("ALG_MODE", "api").lower() == "cluster":
            alive_drones = self.esp.ping_all_drones(self.available_drones) if self.esp else []
            os.environ["CLUSTER_ALIVE_DRONES"] = ",".join(alive_drones)
            print(f"==== DEBUG chess.py: available_drones={self.available_drones}, alive_drones={alive_drones}")
            print(f"==== DEBUG chess.py: CLUSTER_NP before={os.getenv('CLUSTER_NP', 'not_set')}")
            self.logger.debug(f"Updated cluster alive drones: {alive_drones}")

        # 2) Запрашиваем ход (внутри alg решается логика)
        try:
            move = get_turn_final(board, time_budget_ms=TIME_BUDGET_MS)
            print(f"GOT MOVE: {move}")
        except AlgTemporaryError as e:
            if "restart required" in str(e).lower():
                # Обновляем только stockfish-кластер без убийства главного процесса
                try:
                    from cluster_manager import ClusterManager
                    alive_drones = self.esp.ping_all_drones(self.available_drones) if self.esp else []
                    cluster_mgr = ClusterManager(self.logger, self.esp)
                    if cluster_mgr.restart_stockfish_cluster(alive_drones):
                        self.logger.info("Stockfish cluster was restarted in-place; skipping this turn once")
                        return
                    else:
                        self.logger.warning("Failed to restart stockfish cluster in-place; skipping this turn once")
                        return
                except Exception as e2:
                    self.logger.error(f"Failed to soft-restart stockfish cluster: {e2}")
                    return
            else:
                raise

        from_cell = getattr(move, 'from_cell', 'e2')
        to_cell = getattr(move, 'to_cell', 'e4')
        chess_move = f"{from_cell}->{to_cell}"
        self.logger.info(f"MOVING {chess_move}")
        
        # Уведомляем всех дронов о начале хода
        if self.esp and self.esp.is_leader:
            self.esp.broadcast_move_start(chess_move)
        
        # 3) Определяем исполнителя по фигуре на from_cell через камеру
        is_pawn_camera, pawn_id = self._is_pawn_move_by_camera(from_cell)
        
        move_success = False
        try:
            if is_pawn_camera and self.rover_ids:
                # Пешка -> выполняем ход ровером
                self.logger.info(f"Executing pawn move {from_cell}->{to_cell} with rover")
                self._execute_rover_move(move)
                self.logger.info(f"Leader completed rover move: {from_cell}->{to_cell}")
                move_success = True
            else:
                # Не пешка -> выполняем ход дроном
                self.logger.info(f"Executing piece move {from_cell}->{to_cell} with drone")
                self._execute_drone_move(move)
                self.logger.info(f"Leader completed drone move: {from_cell}->{to_cell}")
                move_success = True
            
            # MOCK CAMERA
            requests.post(
                f"http://192.168.1.119:8001/make_move", 
                json={"to_cell": to_cell, "from_cell": from_cell}
            )
        except Exception as e:
            self.logger.error(f"Failed to execute move {chess_move}: {e}")
            move_success = False
        finally:
            # Всегда уведомляем о завершении хода (успешном или неуспешном)
            if self.esp and self.esp.is_leader:
                self.esp.broadcast_move_complete(chess_move, success=move_success)

    
    def _build_drone_role_mapping(self):
        """Строит маппинг ролей дронов из переменных окружения"""
        mapping = {}
        
        for drone_name in self.available_drones:
            # Получаем роль и начальную букву/клетку для каждого дрона из конфигурации
            try:
                drone_config = get_drone_config(drone_name)
                role = drone_config['role'].lower()
                initial = drone_config['initial_letter'].lower()
            except (ValueError, KeyError):
                # Фолбэк на старые переменные окружения
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

    def _execute_drone_move(self, move):
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
        print(f"TARGET DRONE: {target_drone}")
        
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
                # Проверяем, есть ли на целевой клетке вражеская фигура
                is_kill = self._is_enemy_piece_on_cell(to_cell)
                # Выполняем движение локально
                self.move_to_xy(x, y, self.flight_z, is_kill=is_kill)
                self.logger.info(f"Leader completed own move to {to_cell}")
            except Exception as e:
                self.logger.error(f"Leader failed to execute own move")
        else:
            # Проверяем живость дрона-ведомого перед отправкой команды
            self.logger.info(f"Checking if target drone {target_drone} is alive before sending command")
            
            drone_alive_camera = self._check_drone_alive_by_camera(target_drone)
            # Используем более настойчивые пинги: 3 попытки с таймаутом 0.8 сек
            drone_alive_ping = self._check_drone_alive_by_ping(target_drone, retries=3, timeout=0.8)
            
            self.logger.debug(f"Drone {target_drone} status: camera={drone_alive_camera}, ping={drone_alive_ping}")
            
            # Если дрон не жив по одному из критериев
            if not drone_alive_camera or not drone_alive_ping:
                self.logger.warning(f"Target drone {target_drone} is not alive (camera={drone_alive_camera}, ping={drone_alive_ping}), recalculating move with updated board state")
                return self._handle_dead_drone_and_recalculate(target_drone)
            
            # Дрон жив - отправляем команду
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

    def _handle_dead_drone_and_recalculate(self, dead_drone_name: str):
        """Обрабатывает случай мертвого дрона и пересчитывает ход"""
        try:
            # Получаем текущее состояние доски
            original_board = get_board_state()
            
            # Создаем обновленное состояние доски без мертвого дрона
            updated_board = self._create_board_without_dead_drone(original_board, dead_drone_name)
            
            self.logger.info(f"Recalculating move with updated board state (without drone {dead_drone_name})")
            
            # Пересчитываем ход с обновленным состоянием доски
            new_move = get_turn_final(updated_board, time_budget_ms=TIME_BUDGET_MS)
            
            if new_move:
                new_from_cell = getattr(new_move, 'from_cell', 'e2')
                new_to_cell = getattr(new_move, 'to_cell', 'e4')
                
                self.logger.info(f"Recalculated move: {new_from_cell}->{new_to_cell}")
                
                # Определяем нового исполнителя для пересчитанного хода
                new_piece_type = self._get_piece_on_cell_from_camera(new_from_cell)
                new_target_drone = self._find_drone_for_piece(new_piece_type) if new_piece_type else None
                
                if new_target_drone and new_target_drone != dead_drone_name:
                    # Проверяем, является ли это ходом пешки
                    is_pawn_camera, pawn_id = self._is_pawn_move_by_camera(new_from_cell)
                    
                    if is_pawn_camera and self.rover_ids:
                        # Пешка -> выполняем ход ровером
                        self.logger.info(f"Executing recalculated pawn move {new_from_cell}->{new_to_cell} with rover")
                        self._execute_rover_move(new_move)
                    elif new_target_drone == self.drone_name:
                        # Лидер выполняет ход сам
                        self.logger.info(f"Leader executing recalculated own move: {new_from_cell}->{new_to_cell}")
                        x, y = self.get_cell_coordinates(new_to_cell)
                        # Проверяем, есть ли на целевой клетке вражеская фигура
                        is_kill = self._is_enemy_piece_on_cell(new_to_cell)
                        self.move_to_xy(x, y, self.flight_z, is_kill=is_kill)
                    else:
                        # Отправляем команду другому живому дрону
                        new_chess_move = f"{new_from_cell}->{new_to_cell}"
                        self.logger.info(f"Sending recalculated move to drone {new_target_drone}: {new_chess_move}")
                        
                        if self.esp and self.esp.is_leader:
                            payload = {
                                'type': 'move',
                                'to': new_target_drone,
                                'move': new_chess_move,
                            }
                            success = self.esp._broadcast_reliable(payload)
                            if success:
                                self.logger.info(f"Successfully sent recalculated move command to {new_target_drone}")
                            else:
                                self.logger.error(f"Failed to send recalculated move command to {new_target_drone}")
                    
                    # Обновляем камеру с новым ходом
                    requests.post(
                        f"http://192.168.1.119:8001/make_move", 
                        json={"to_cell": new_to_cell, "from_cell": new_from_cell}
                    )
                else:
                    self.logger.error(f"No valid target drone found for recalculated move or target is same dead drone")
            else:
                self.logger.error("Failed to recalculate move")
                
        except Exception as e:
            self.logger.error(f"Failed to handle dead drone and recalculate: {e}")

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
        
        # Получаем текущую позицию ровера с камеры в метрах
        current_x, current_y, current_yaw = self._get_rover_position_from_camera(rover_id, move.from_cell)
        
        # Получаем координаты целевой клетки и конвертируем в метры для rover API
        target_aruco_x, target_aruco_y = self.get_cell_coordinates(move.to_cell)
        target_x, target_y = aruco_to_rover_meters(target_aruco_x, target_aruco_y)
        
        self.logger.info(f"Rover {rover_id} move: {move.from_cell}->{move.to_cell}")
        self.logger.info(f"  From position (meters): ({current_x:.3f}, {current_y:.3f}, {current_yaw:.1f}°)")
        self.logger.info(f"  To position (meters): ({target_x:.3f}, {target_y:.3f})")
        
        # TODO: Добавить логику is_kill для роверов (проверка вражеской фигуры на целевой клетке)
        # is_kill = self._is_enemy_piece_on_cell(move.to_cell)
        
        try:
            # Отправляем команду с текущей и целевой позицией в метрах
            success = self.rover.navigate(
                rover_id, 
                current_x=current_x, 
                current_y=current_y, 
                current_yaw=current_yaw,
                target_x=target_x, 
                target_y=target_y
            )
            
            if success:
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
            
        self.logger.debug(f"Follower {self.drone_name} waiting for chess move command...")
        
        # Ждем команду от лидера
        if self.esp._chess_move_event.wait(timeout):
            # Получили команду
            chess_move = self.esp._received_chess_move
            self.esp._chess_move_event.clear()  # Сбрасываем событие
            self.esp._received_chess_move = None
            
            self.logger.debug(f"Received chess move command: {chess_move}")
            
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
                # Проверяем, есть ли на целевой клетке вражеская фигура
                is_kill = self._is_enemy_piece_on_cell(to_cell)
                self.move_to_xy(x, y, self.flight_z, is_kill=is_kill)
                self.logger.info(f"Successfully completed move to {to_cell}")
                return True
            except Exception as e:
                self.logger.error(f"Failed to execute move: {e}")
                return False
        else:
            self.logger.warning(f"Timeout waiting for chess move command ({timeout}s)")
            return False



    def _tick_leader_election(self):
        """Упрощенная логика выбора лидера"""
        if not self.esp:
            return
        
        # Проверяем только перед нашим ходом
        if not self._check_is_our_turn():
            return
        
        self.logger.info("Our turn detected - checking leader status")
        
        # Проверяем, выполняет ли лидер сейчас ход
        leader_is_moving = self.esp.is_leader_moving() if self.esp else False
        
        if leader_is_moving:
            move_info = self.esp.get_current_move_info() if self.esp else {}
            self.logger.info(f"Leader {self.current_leader} is executing move: {move_info.get('move', 'unknown')} (elapsed: {move_info.get('elapsed_time', 0):.1f}s)")
            # Лидер выполняет ход - не переизбираем его
            return
        
        # Проверяем живость текущего лидера двумя способами
        leader_alive_camera = self._check_leader_alive_by_camera()
        leader_alive_ping = self._check_leader_alive_by_ping()
        
        self.logger.debug(f"Leader {self.current_leader} status: camera={leader_alive_camera}, ping={leader_alive_ping}")
        
        # Если лидер неактивен по одному из критериев, выбираем нового
        if not leader_alive_camera or not leader_alive_ping:
            self.logger.warning(f"Leader {self.current_leader} is inactive (camera={leader_alive_camera}, ping={leader_alive_ping}), selecting new leader")
            
            new_leader = self._select_new_leader()
            
            if new_leader != self.current_leader:
                # Устанавливаем нового лидера
                self.current_leader = new_leader
                
                # Обновляем свою роль
                is_new_leader = (self.drone_name == new_leader)
                self.esp.update_role(is_new_leader)
                
                if is_new_leader:
                    self.logger.info(f"Became new leader: {self.drone_name}")
                else:
                    self.logger.info(f"New leader selected: {new_leader}")
        else:
            self.logger.info(f"Leader {self.current_leader} is healthy")
    
    def _check_is_our_turn(self) -> bool:
        """Проверяет через камеру, наш ли сейчас ход"""
        now = time.time()
        
        # Кешируем результат на 2 секунды чтобы не спамить камеру
        if (now - self._last_turn_check) < 2.0:
            return self._is_our_turn_cache
        
        try:
            # Получаем информацию о текущем ходе через камеру
            camera_turn = None
            if hasattr(self.camera, 'get_board_positions'):
                self.camera.get_board_positions()
            if hasattr(self.camera, 'get_turn'):
                camera_turn = self.camera.get_turn()
            
            if camera_turn:
                self._is_our_turn_cache = camera_turn.lower() == self.our_team
                self._last_turn_check = now
                return self._is_our_turn_cache
                
        except Exception as e:
            self.logger.debug(f"Failed to check turn via camera: {e}")
        
        # Fallback: проверяем через алгоритм
        try:
            board = get_board_state()
            current_player = getattr(board, 'turn', 'white').lower()
            self._is_our_turn_cache = current_player == self.our_team
            self._last_turn_check = now
            return self._is_our_turn_cache
        except Exception as e:
            self.logger.debug(f"Failed to check turn via board state: {e}")
        
        return False
    
    def _check_leader_alive_by_camera(self) -> bool:
        """Проверяет через камеру, виден ли лидер на доске"""
        if not self.current_leader:
            return False
            
        try:
            positions = self.camera.get_board_positions()
            
            # Ищем фигуры на доске - если видим фигуры, значит дроны живые
            for color_data in positions.values():
                for piece_type, piece_pos in color_data.items():
                    # Пропускаем пешки (они управляются роверами)
                    if piece_type.lower() == 'pawn' or piece_type.lower().startswith('pawn_'):
                        continue
                    
                    # Если видим фигуру, которая соответствует лидеру
                    responsible_drone = self._find_drone_for_piece(piece_type)
                    if responsible_drone == self.current_leader:
                        self.logger.debug(f"Leader {self.current_leader} is alive (visible piece: {piece_type})")
                        return True
            
            self.logger.warning(f"Leader {self.current_leader} not visible on camera")
            return False
            
        except Exception as e:
            self.logger.debug(f"Camera leader check failed: {e}")
            return False
    
    def _check_leader_alive_by_ping(self) -> bool:
        """Проверяет живость лидера через ping с умеренными ретраями"""
        if not self.esp or not self.current_leader:
            return False
        
        try:
            # Для проверки лидера используем меньше попыток: 3 попытки по 0.8 сек
            is_alive = self.esp.ping_drone(self.current_leader, retries=3, timeout=0.8)
            
            if is_alive:
                self.logger.debug(f"Leader {self.current_leader} is alive (ping successful)")
            else:
                self.logger.warning(f"Leader {self.current_leader} did not respond to ping")
            
            return is_alive            
        except Exception as e:
            self.logger.debug(f"Ping leader check failed: {e}")
            return False
    
    def _check_drone_alive_by_camera(self, drone_name: str) -> bool:
        """Проверяет через камеру, виден ли указанный дрон на доске"""
        if not drone_name:
            return False
            
        try:
            positions = self.camera.get_board_positions()
            
            # Ищем фигуры на доске - если видим фигуры, значит дроны живые
            for color_data in positions.values():
                for piece_type, piece_pos in color_data.items():
                    # Пропускаем пешки (они управляются роверами)
                    if piece_type.lower() == 'pawn' or piece_type.lower().startswith('pawn_'):
                        continue
                    
                    # Если видим фигуру, которая соответствует указанному дрону
                    responsible_drone = self._find_drone_for_piece(piece_type)
                    if responsible_drone == drone_name:
                        self.logger.debug(f"Drone {drone_name} is alive (visible piece: {piece_type})")
                        return True
            
            self.logger.warning(f"Drone {drone_name} not visible on camera")
            return False
            
        except Exception as e:
            self.logger.debug(f"Camera drone check failed for {drone_name}: {e}")
            return False
    
    def _check_drone_alive_by_ping(self, drone_name: str, retries: int = 5, timeout: float = 1.0) -> bool:
        """
        Проверяет живость указанного дрона через ping
        
        Args:
            drone_name: Имя проверяемого дрона
            retries: Количество попыток пинга
            timeout: Таймаут ожидания ответа
        """
        if not self.esp or not drone_name:
            return False
        
        try:
            is_alive = self.esp.ping_drone(drone_name, retries=retries, timeout=timeout)
            
            if is_alive:
                self.logger.debug(f"Drone {drone_name} is alive (ping successful)")
            else:
                self.logger.warning(f"Drone {drone_name} did not respond to ping after {retries} attempts")
            
            return is_alive
            
        except Exception as e:
            self.logger.debug(f"Ping drone check failed for {drone_name}: {e}")
            return False
    
    def _create_board_without_dead_drone(self, board, dead_drone_name: str):
        """Создает новое состояние доски без фигур мертвого дрона"""
        try:
            # Получаем текущие позиции с камеры
            positions = self.camera.get_board_positions()
            
            # Создаем новый FEN без фигур мертвого дрона
            board_array = [['.' for _ in range(8)] for _ in range(8)]
            
            # Заполняем доску живыми фигурами
            for color_data in positions.values():
                for piece_type, piece_pos in color_data.items():
                    # Пропускаем пешки (роверы) и фигуры мертвого дрона
                    if piece_type.lower() == 'pawn' or piece_type.lower().startswith('pawn_'):
                        continue
                    
                    responsible_drone = self._find_drone_for_piece(piece_type)
                    if responsible_drone == dead_drone_name:
                        self.logger.info(f"Removing {piece_type} from dead drone {dead_drone_name}")
                        continue
                    
                    # Размещаем живую фигуру на доске
                    try:
                        file_idx = FILES.index(piece_pos.cell[0])
                        rank_idx = 8 - int(piece_pos.cell[1])
                        
                        # Определяем символ фигуры для FEN
                        fen_piece = self._piece_type_to_fen_symbol(piece_type)
                        if fen_piece:
                            board_array[rank_idx][file_idx] = fen_piece
                    except (ValueError, IndexError) as e:
                        self.logger.warning(f"Invalid position {piece_pos.cell} for piece {piece_type}: {e}")
            
            # Конвертируем массив в FEN
            fen_rows = []
            for row in board_array:
                fen_row = ""
                empty_count = 0
                for cell in row:
                    if cell == '.':
                        empty_count += 1
                    else:
                        if empty_count > 0:
                            fen_row += str(empty_count)
                            empty_count = 0
                        fen_row += cell
                if empty_count > 0:
                    fen_row += str(empty_count)
                fen_rows.append(fen_row)
            
            # Создаем новый FEN с текущими метаданными
            board_fen = '/'.join(fen_rows)
            current_turn = getattr(board, 'turn', 'white')
            castling = getattr(board, 'castling_rights', 'KQkq')
            en_passant = getattr(board, 'en_passant', '-')
            halfmove = getattr(board, 'halfmove_clock', 0)
            fullmove = getattr(board, 'fullmove_number', 1)
            
            new_fen = f"{board_fen} {current_turn} {castling} {en_passant} {halfmove} {fullmove}"
            
            # Возвращаем оригинальную доску с логированием об удалении мертвого дрона
            # (Полное обновление FEN требует сложной логики, которую можно добавить позже)
            self.logger.info(f"Created updated board state without drone {dead_drone_name} (simplified)")
            self.logger.info(f"New board FEN would be: {new_fen}")
            
            # Возвращаем оригинальную доску - алгоритм все равно получит обновленные данные с камеры
            return board
                
        except Exception as e:
            self.logger.error(f"Failed to create board without dead drone {dead_drone_name}: {e}")
            return board
    
    def _piece_type_to_fen_symbol(self, piece_type: str) -> str:
        """Конвертирует тип фигуры в символ FEN"""
        piece_type = piece_type.lower()
        
        # Определяем базовый тип фигуры
        if 'king' in piece_type:
            return 'K'  # Предполагаем белые, черные будут другим способом
        elif 'queen' in piece_type:
            return 'Q'
        elif 'rook' in piece_type:
            return 'R'
        elif 'bishop' in piece_type:
            return 'B'
        elif 'knight' in piece_type:
            return 'N'
        elif 'pawn' in piece_type:
            return 'P'
        else:
            return None

    def _select_new_leader(self) -> str:
        """Выбирает нового лидера из живых дронов (первый в DRONE_LIST)"""
        if not self.esp:
            return self.available_drones[0] if self.available_drones else self.drone_name
        
        try:
            # Пингуем всех дронов чтобы найти живых
            alive_drones = self.esp.ping_all_drones(self.available_drones)
            
            if not alive_drones:
                self.logger.warning("No alive drones found, using self as leader")
                return self.drone_name
            
            # Берем первый живой дрон из DRONE_LIST (игнорируем LEADER_DRONE)
            for drone_name in self.available_drones:
                if drone_name in alive_drones:
                    self.logger.info(f"Selected new leader: {drone_name} (first alive in DRONE_LIST)")
                    return drone_name
            
            # Fallback: если что-то пошло не так
            return alive_drones[0]
            
        except Exception as e:
            self.logger.error(f"Leader selection failed: {e}")
            return self.drone_name
    
    def _handle_cluster_restart_request(self):
        """Обрабатывает запрос на перезапуск кластера"""
        try:
            from cluster_manager import ClusterManager
            
            cluster_mgr = ClusterManager(self.logger, self.esp)
            
            # Получаем живых дронов
            alive_drones = self.esp.ping_all_drones(self.available_drones) if self.esp else []
            
            # Инициируем перезапуск кластера
            if cluster_mgr.initiate_cluster_restart(alive_drones, "drone_list_changed"):
                self.logger.info("Cluster restart initiated - exiting process")
                # Используем метод из cluster_manager для изящного завершения
                cluster_mgr.perform_graceful_restart("drone_list_changed")
            else:
                self.logger.error("Failed to initiate cluster restart")
                
        except Exception as e:
            self.logger.error(f"Failed to handle cluster restart: {e}")
    
    def _check_and_handle_restart_completion(self):
        """Проверяет и обрабатывает завершение перезапуска после старта"""
        try:
            from cluster_manager import handle_restart_after_startup
            handle_restart_after_startup(self.logger, self.esp)
        except Exception as e:
            self.logger.debug(f"Failed to handle restart completion: {e}") 

    def run(self):
        try:
            rospy.init_node("chess_drone_single")
        except Exception:
            pass

        self.logger.info("Starting chess drone MAIN LOOP (dynamic leadership)...")
        
        # Проверяем завершение перезапуска кластера
        self._check_and_handle_restart_completion()

        last_move_ts = 0.0
        move_interval = 5.0

        while not rospy.is_shutdown():
            try:
                # Проверка лидерства только перед нашим ходом
                self._tick_leader_election()

                now = time.time()
                if self.esp and self.esp.is_leader:
                    self.logger.info(f"AM I LEADER? {self.esp.is_leader}")
                    # Ходы лидера с паузой между ними
                    if (now - last_move_ts) >= move_interval:
                        self.logger.info(f"=== Leader run_once call ===")
                        # Перед началом ещё раз убедимся, что мы лидер
                        if self.esp.is_leader:
                            self.run_once()
                            last_move_ts = time.time()
                        else:
                            self.logger.info("Leadership lost before executing turn; skipping")
                else:
                    # Ведомый: ждём команды небольшими таймаутами
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
