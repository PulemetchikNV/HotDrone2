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

try:
    from .flight import FlightController
    from .helpers import setup_logging
    from .alg_mock2 import get_board_state, get_turn, update_after_execution, AlgTemporaryError, AlgPermanentError
    from .esp import EspController, create_chess_move_message, parse_chess_move
    from .const import DRONE_LIST, LEADER_DRONE
except ImportError:
    from flight import FlightController
    from helpers import setup_logging
    from alg_mock2 import get_board_state, get_turn, update_after_execution, AlgTemporaryError, AlgPermanentError
    from esp import EspController, create_chess_move_message, parse_chess_move
    from const import DRONE_LIST, LEADER_DRONE

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


class CameraAdapter:
    """Адаптер к alg.get_board_state()/update_after_execution для совместимости с текущим кодом."""
    def __init__(self, logger):
        self.logger = logger
        self._board = None

    def read_board(self):
        # Получаем снимок через alg API
        self._board = get_board_state()
        return self._board

    def commit_move(self, move, success: bool):
        # Обновляем локальное состояние после исполнения
        if self._board is not None:
            self._board = update_after_execution(self._board, move, success)


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
        
        # ESP контроллер для коммуникации между дронами
        self.esp = EspController(swarm=self.swarm, drone_name=self.drone_name) if self.swarm else None
        
        # Регистрируем обработчик сообщений если есть swarm
        if self.swarm and self.esp:
            # Предполагаем что swarm имеет метод set_custom_message_callback
            if hasattr(self.swarm, 'set_custom_message_callback'):
                self.swarm.set_custom_message_callback(self.esp._on_custom_message)
                self.logger.info("ESP message handler registered")
        
        # Список доступных дронов для назначения задач
        self.available_drones = DRONE_LIST.split(',') if DRONE_LIST else []
        self.logger.info(f"Available drones: {self.available_drones}")

        # Параметры поля
        self.cell_size = float(os.getenv("CELL_SIZE_M", "0.35"))
        self.origin_x = float(os.getenv("BOARD_ORIGIN_X", "0.0"))  # центр доски = (0,0)
        self.origin_y = float(os.getenv("BOARD_ORIGIN_Y", "0.0"))

        # Параметры полёта
        self.takeoff_z = float(os.getenv("TAKEOFF_Z", "1.0"))
        self.flight_z = float(os.getenv("FLIGHT_Z", "1.0"))
        self.speed = float(os.getenv("SPEED", "0.3"))

        self.cam = CameraAdapter(self.logger)

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
        """Выполняется только на лидере - получает ход и отправляет команду дрону"""
        if not self.is_leader:
            self.logger.warning("run_once called on non-leader drone")
            return
            
        # 1) Получаем состояние доски через alg
        board = self.cam.read_board()
        # 2) Запрашиваем ход (внутри alg решается логика)
        move = get_turn(board, time_budget_ms=5000)

        # Временная логика для тестирования - назначаем клетки по очереди
        to_cell = ''
        target_drone = None
        if current_turn == 0:
            to_cell = 'a1'
            target_drone = self.available_drones[0] if self.available_drones else None
        elif current_turn == 1:
            to_cell = 'g1'
            target_drone = self.available_drones[1] if len(self.available_drones) > 1 else self.available_drones[0]
        elif current_turn == 2:
            to_cell = 'e5'
            target_drone = self.available_drones[2] if len(self.available_drones) > 2 else self.available_drones[0]
        elif current_turn == 3:
            to_cell = 'h8'
            target_drone = self.available_drones[0] if self.available_drones else None
        else:
            to_cell = move.to_cell if hasattr(move, 'to_cell') else 'e4'
            target_drone = self.available_drones[0] if self.available_drones else None
            
        chess_move = f"{move.from_cell}->{to_cell}" if hasattr(move, 'from_cell') else f"e2->{to_cell}"
        self.logger.info(f"Leader sending chess move: {chess_move} to drone {target_drone}")

        # 3) Отправляем команду выбранному дрону через ESP
        if target_drone and self.esp:
            payload = create_chess_move_message(target_drone, chess_move)
            success = self.esp._broadcast_reliable(payload)
            if success:
                self.logger.info(f"Successfully sent move command to {target_drone}")
            else:
                self.logger.error(f"Failed to send move command to {target_drone}")
        else:
            self.logger.warning("No ESP controller or target drone available")

        # 4) Сообщаем alg об исполнении
        self.cam.commit_move(move, success=True)
        self.logger.info(f"Leader completed turn {current_turn}: {chess_move}")

    def wait_and_execute_move(self, timeout=30.0):
        """Выполняется на ведомых дронах - ждет команду от лидера и выполняет её"""
        if self.is_leader:
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
            from_cell, to_cell = parse_chess_move(chess_move)
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

    def run(self):
        try:
            rospy.init_node("chess_drone_single")
        except Exception:
            pass

        if self.is_leader:
            self.logger.info("Starting chess drone LEADER...")
            self.run_leader()
        else:
            self.logger.info("Starting chess drone FOLLOWER...")
            self.run_follower()

    def run_leader(self):
        """Основной цикл для дрона-лидера"""
        TURN_LIMIT = 5
        turn_count = 0
        
        while not rospy.is_shutdown():
            try:
                if TURN_LIMIT > turn_count:
                    self.logger.info(f"=== Leader executing turn {turn_count + 1} ===")
                    self.run_once(turn_count)
                    turn_count += 1
                    
                    # Пауза между ходами
                    self.fc.wait(5.0)
                else:
                    self.logger.info("Leader completed all turns, waiting...")
                    self.fc.wait(10.0)

            except KeyboardInterrupt:
                break
            except AlgTemporaryError as e:
                self.logger.warning(f"temporary error in alg: {e}")
                self.fc.wait(1.0)
            except AlgPermanentError as e:
                self.logger.error(f"permanent error in alg: {e}")
                self.fc.wait(3.0)
            except Exception as e:
                self.logger.error(f"leader error: {e}")
                self.fc.wait(3.0)
        self.logger.info("Leader stopped.")

    def run_follower(self):
        """Основной цикл для дрона-ведомого"""
        while not rospy.is_shutdown():
            try:
                # Ждем команду от лидера и выполняем её
                success = self.wait_and_execute_move(timeout=30.0)
                if success:
                    self.logger.info("Move executed successfully, waiting for next command...")
                else:
                    self.logger.warning("Failed to execute move or timeout, continuing...")
                    
                # Короткая пауза перед следующим ожиданием
                self.fc.wait(1.0)

            except KeyboardInterrupt:
                break
            except Exception as e:
                self.logger.error(f"follower error: {e}")
                self.fc.wait(3.0)
        self.logger.info("Follower stopped.")


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