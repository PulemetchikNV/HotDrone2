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
from alg2_stockfish import get_board_state, get_turn, update_after_execution, AlgTemporaryError, AlgPermanentError, get_stockfish_status, test_stockfish_integration
import esp
from const import DRONE_LIST, LEADER_DRONE, rovers
from rover import RoverController
from camera import create_camera_controller

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


class ChessDroneAlg2:
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

        # Initialize Stockfish integration
        self._initialize_stockfish()

    def _initialize_stockfish(self):
        """Initialize and test Stockfish integration"""
        try:
            self.logger.info("Initializing Stockfish integration...")
            status = get_stockfish_status()
            self.logger.info(f"Stockfish status: {status}")
            
            if status.get("engine_available", False):
                # Test the integration
                test_result = test_stockfish_integration()
                if test_result.get("status") == "success":
                    self.logger.info(f"Stockfish integration test successful: {test_result}")
                else:
                    self.logger.warning(f"Stockfish integration test failed: {test_result}")
            else:
                self.logger.warning(f"Stockfish engine not available: {status}")
                
        except Exception as e:
            self.logger.error(f"Failed to initialize Stockfish: {e}")

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
            z=0.15,
            speed=0.2,
            auto_arm=False,
        )

        self.fc.wait(0.8)

        self.fc.force_disarm()

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
        self.logger.info("Requesting move from Stockfish engine...")
        move = get_turn(board, time_budget_ms=5000)
        print(f"GOT MOVE: {move}")
        
        # Log move details if available
        if hasattr(move, 'meta') and move.meta:
            self.logger.info(f"Move meta: {move.meta}")
        if hasattr(move, 'score_cp') and move.score_cp is not None:
            self.logger.info(f"Move score: {move.score_cp} centipawns")
        if hasattr(move, 'is_mate') and move.is_mate:
            self.logger.info("Move is a mate!")

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
        
        # MOCK CAMERA
        requests.post(
            f"http://localhost:8000/make_move", 
            json={"move": f"{from_cell}->{to_cell}"}
        )

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
                base_type, initial = parts
                # Ищем в формате {base_type}_{initial}
                search_key = f"{base_type}_{initial}"
                if search_key in self.drone_role_mapping:
                    return self.drone_role_mapping[search_key]
        
        return None
    
    def _is_pawn_move_by_camera(self, from_cell: str) -> tuple[bool, str]:
        """Определяет, является ли ход ходом пешки, используя данные камеры"""
        try:
            piece_type = self._get_piece_on_cell_from_camera(from_cell)
            if piece_type and 'pawn' in piece_type.lower():
                # Находим соответствующий rover_id
                for rover_id, rover_config in rovers.items():
                    if isinstance(rover_config, dict):
                        initial_letter = rover_config.get('initial_letter', '').lower()
                        if initial_letter and from_cell.startswith(initial_letter):
                            return True, rover_id
                return True, "unknown_pawn"
            return False, ""
        except Exception as e:
            self.logger.debug(f"Failed to determine if move is pawn move: {e}")
            return False, ""
    
    def _execute_rover_move(self, move):
        """Выполняет ход ровером"""
        from_cell = getattr(move, 'from_cell', 'e2')
        to_cell = getattr(move, 'to_cell', 'e4')
        
        try:
            # Получаем координаты начальной и конечной клеток
            from_x, from_y = self.get_cell_coordinates(from_cell)
            to_x, to_y = self.get_cell_coordinates(to_cell)
            
            # Находим соответствующий rover_id
            rover_id = None
            for rid, rover_config in rovers.items():
                if isinstance(rover_config, dict):
                    initial_letter = rover_config.get('initial_letter', '').lower()
                    if initial_letter and from_cell.startswith(initial_letter):
                        rover_id = rid
                        break
            
            if not rover_id:
                self.logger.error(f"No rover found for pawn move from {from_cell}")
                return
            
            # Выполняем движение ровера
            self.logger.info(f"Moving rover {rover_id} from ({from_x:.3f}, {from_y:.3f}) to ({to_x:.3f}, {to_y:.3f})")
            
            if hasattr(self.rover, 'move_rover_to_position'):
                success = self.rover.move_rover_to_position(rover_id, to_x, to_y, 0)
                if success:
                    self._update_rover_position(rover_id, to_x, to_y, 0)
                    self.logger.info(f"Rover {rover_id} move completed successfully")
                else:
                    self.logger.error(f"Rover {rover_id} move failed")
            else:
                self.logger.warning("Rover controller doesn't support move_rover_to_position")
                # Fallback: just update position
                self._update_rover_position(rover_id, to_x, to_y, 0)
                
        except Exception as e:
            self.logger.error(f"Error executing rover move: {e}")
    
    def _execute_drone_move(self, move, current_turn):
        """Выполняет ход дроном"""
        from_cell = getattr(move, 'from_cell', 'e2')
        to_cell = getattr(move, 'to_cell', 'e4')
        
        try:
            # Получаем тип фигуры на начальной клетке
            piece_type = self._get_piece_on_cell_from_camera(from_cell)
            
            # Находим дрон, ответственный за эту фигуру
            target_drone = self._find_drone_for_piece(piece_type)
            
            if target_drone:
                # Отправляем команду другому дрону
                command = f"{from_cell}->{to_cell}"
                self.logger.info(f"Sending command to {target_drone}: {command}")
                
                if self.esp and hasattr(self.esp, 'send_move_command'):
                    success = self.esp.send_move_command(target_drone, command)
                    if success:
                        self.logger.info(f"Command sent successfully to {target_drone}")
                    else:
                        self.logger.error(f"Failed to send command to {target_drone}")
                else:
                    self.logger.warning("ESP controller doesn't support send_move_command")
            else:
                # Если дрон не найден, выполняем ход сами
                self.logger.info(f"No drone found for piece {piece_type}, executing move ourselves")
                to_x, to_y = self.get_cell_coordinates(to_cell)
                self.move_to_xy(to_x, to_y, self.flight_z)
                
        except Exception as e:
            self.logger.error(f"Error executing drone move: {e}")

    def wait_and_execute_move(self, timeout: float = 0.3):
        """Ведомый дрон: ждёт команду и выполняет ход"""
        if not self.esp:
            self.logger.warning("No ESP controller available for move execution")
            return
        
        try:
            # Получаем команду от лидера
            command = self.esp.get_move_command(timeout=timeout)
            if command:
                self.logger.info(f"Received move command: {command}")
                
                # Парсим команду формата "e2->e4"
                if "->" in command:
                    from_cell, to_cell = command.split("->", 1)
                    from_cell = from_cell.strip()
                    to_cell = to_cell.strip()
                    
                    # Выполняем движение
                    to_x, to_y = self.get_cell_coordinates(to_cell)
                    self.logger.info(f"Executing move to {to_cell} at ({to_x:.3f}, {to_y:.3f})")
                    self.move_to_xy(to_x, to_y, self.flight_z)
                    
                    # Отправляем подтверждение
                    if hasattr(self.esp, 'send_move_confirmation'):
                        self.esp.send_move_confirmation(command)
                        self.logger.info("Move confirmation sent")
                else:
                    self.logger.warning(f"Invalid command format: {command}")
            else:
                self.logger.warning(f"Timeout waiting for chess move command ({timeout}s)")
                
        except Exception as e:
            self.logger.error(f"Error in wait_and_execute_move: {e}")

    def _estimate_active_drones(self) -> list:
        """Оценивает список активных дронов"""
        # Пока просто возвращаем всех доступных дронов
        return list(self.available_drones)

    def _next_active_leader(self, current_leader: str, active_drones: list) -> str:
        """Определяет следующего лидера из списка активных дронов"""
        if not active_drones:
            return None
        
        try:
            current_index = active_drones.index(current_leader)
            next_index = (current_index + 1) % len(active_drones)
            return active_drones[next_index]
        except ValueError:
            # Если текущий лидер не найден в списке активных, возвращаем первого
            return active_drones[0] if active_drones else None

    def _order_index(self, drone_name: str) -> int:
        """Возвращает индекс дрона в списке доступных дронов"""
        try:
            return self.available_drones.index(drone_name)
        except ValueError:
            return 0

    def _tick_leader_election(self):
        """Обработка выборов лидера и heartbeat'ов"""
        if not self.esp:
            return
        
        now = time.time()
        last_hb = self.esp.get_last_heartbeat()
        known_leader, known_term = self.esp.get_known_leader_and_term()

        # Принятие более свежего лидера
        if last_hb and int(known_term) >= int(getattr(self, 'current_term', 0)):
            self.current_term = int(known_term)
            if known_leader:
                self.current_leader = known_leader
                self.esp.update_role(self.drone_name == self.current_leader)

        # MOCK
        self.current_leader = 'drone15' 

    def run(self):
        try:
            rospy.init_node("chess_drone_alg2")
        except Exception:
            pass

        self.logger.info("Starting chess drone MAIN LOOP (dynamic leadership) with Stockfish integration...")

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
                        self.logger.info(f"=== Leader executing turn {turn_count + 1} with Stockfish ===")
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
    # chess_drone = ChessDroneAlg2(swarm=your_swarm_instance)
    # chess_drone.run()
    
    # Для автономного запуска без swarm:
    ChessDroneAlg2().run()

"""
Использование:
Логика работы:
   - Лидер получает ходы от Stockfish алгоритма и отправляет команды ведомым
   - Ведомые ждут команды формата "c2->d4" и летят в указанную клетку
   - Координаты берутся из aruco_map2.json или вычисляются
   - Коммуникация через ESP контроллер с подтверждениями
   - Использует Stockfish engine для интеллектуальной генерации ходов
"""
