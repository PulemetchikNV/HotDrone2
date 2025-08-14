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
except ImportError:
    from flight import FlightController
    from helpers import setup_logging
    from alg_mock2 import get_board_state, get_turn, update_after_execution, AlgTemporaryError, AlgPermanentError


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
        self.logger = setup_logging(self.drone_name)

        # Не переопределяем FLIGHT_IMPL — используйте main/custom/mock через env
        self.fc = FlightController(drone_name=self.drone_name, logger=self.logger)

        # Параметры поля
        self.aruco_id = int(os.getenv("BOARD_ARUCO_ID", "89"))
        self.cell_size = float(os.getenv("CELL_SIZE_M", "0.35"))
        self.origin_x = float(os.getenv("BOARD_ORIGIN_X", "0.0"))  # центр доски = (0,0)
        self.origin_y = float(os.getenv("BOARD_ORIGIN_Y", "0.0"))

        # Параметры полёта
        self.takeoff_z = float(os.getenv("TAKEOFF_Z", "1.0"))
        self.flight_z = float(os.getenv("FLIGHT_Z", "1.0"))
        self.speed = float(os.getenv("SPEED", "0.3"))
        self.tolerance = float(os.getenv("TOLERANCE", "0.25"))
        self.hover_time = float(os.getenv("HOVER_TIME", "1.0"))

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

    def move_to_xy(self, x: float, y: float, z: float):
        frame_id = f"aruco_{self.aruco_id}"
        
        # 1. Взлёт на рабочую высоту
        self.fc.takeoff(z=self.takeoff_z, delay=2, speed=0.5)
        time.sleep(3)

        self.fc.navigate_wait(
            x=x, y=y, z=self.flight_z,
            speed=self.speed,
            frame_id=frame_id,
            auto_arm=True,
            arrival_tolerance=self.tolerance,
            hover_time=self.hover_time
        )
        
        # 4. Снижение до z=0.5
        self.fc.navigate_wait(
            x=x, y=y, z=0.25,
            speed=0.3,
            frame_id=frame_id,
            auto_arm=False,
            arrival_tolerance=0.1,
            hover_time=0.5
        )

        self.fc.land()
        
        # 5. Финальное зависание перед завершением
        # self.fc.wait(0.5)

    def run_once(self):
        # 1) Получаем состояние доски через alg
        board = self.cam.read_board()
        # 2) Запрашиваем ход (внутри alg решается логика)
        move = get_turn(board, time_budget_ms=5000)
        to_cell = 'f6' # move.to_cell
        self.logger.info(f"Move: {move.from_cell} -> {to_cell} (uci={move.uci})")

        # 3) Берём координаты клетки из JSON-карты и летим (фолбэк на формулу при отсутствии)
        marker = self.cell_markers.get(to_cell)
        if isinstance(marker, dict) and "x" in marker and "y" in marker:
            x, y = float(marker["x"]), float(marker["y"])
            self.logger.info(f"Using map coords for {to_cell}: x={x:.3f}, y={y:.3f}")
        else:
            x, y = cell_to_xy(move.to_cell, self.cell_size, self.origin_x, self.origin_y)
            self.logger.warning(
                f"No map coords for {move.to_cell}. Using computed coords: x={x:.3f}, y={y:.3f}"
            )
        self.move_to_xy(x, y, self.flight_z)

        # 4) Сообщаем alg об исполнении
        self.cam.commit_move(move, success=True)
        self.logger.info(f"Done: now at {to_cell}")

    def run(self):
        try:
            rospy.init_node("chess_drone_single")
        except Exception:
            pass

        self.logger.info("Starting autonomous chess drone (single piece)…")

        TURN_LIMIT = 1
        turn_count = 0
        while not rospy.is_shutdown():
            try:
                if(TURN_LIMIT > turn_count):
                    self.run_once()
                    turn_count += 1

                self.fc.wait(2.0)
            except KeyboardInterrupt:
                break
            except AlgTemporaryError as e:
                self.logger.warning(f"temporary error in alg: {e}")
                self.fc.wait(1.0)
            except AlgPermanentError as e:
                self.logger.error(f"permanent error in alg: {e}")
                self.fc.wait(3.0)
            except Exception as e:
                self.logger.error(f"loop error: {e}")
                self.fc.wait(3.0)
        self.logger.info("Stopped.")


if __name__ == "__main__":
    ChessDroneSingle().run()