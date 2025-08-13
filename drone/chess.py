import math
import logging
import os
import sys
import time
import requests

# rospy fallback for local testing
try:
    import rospy
except ImportError:
    # Mock rospy for local testing
    class MockRospy:
        def is_shutdown(self):
            return False
        def init_node(self, name):
            pass
        def sleep(self, duration):
            import time
            time.sleep(duration)
    rospy = MockRospy()

try:
    from .flight import FlightController
    from .helpers import setup_logging
except ImportError:
    from flight import FlightController
    from helpers import setup_logging


class ChessCoords:
    """Object to hold chess move coordinates with from/to positions"""
    def __init__(self, from_x=0.0, from_y=0.0, to_x=0.0, to_y=0.0, z=1.2, aruco_id=89):
        self.from_x = from_x
        self.from_y = from_y
        self.to_x = to_x
        self.to_y = to_y
        self.z = z
        self.aruco_id = aruco_id
    
    def __str__(self):
        return f"ChessCoords(from=({self.from_x}, {self.from_y}) to=({self.to_x}, {self.to_y}), z={self.z}, aruco_id={self.aruco_id})"


class ChessAPI:
    """API для взаимодействия с шахматным сервером"""
    def __init__(self, drone_name="not_known", api_url='http://192.168.2.95:8000'):
        self.api_url = api_url
        self.logger = setup_logging(drone_name)
    
    def get_game_state(self):
        """Получить текущее состояние шахматной игры"""
        try:
            response = requests.get(f"{self.api_url}/chess/game-state")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error getting chess game state: {e}")
            return None
    
    def complete_move(self, drone_name):
        """Сообщить серверу о завершении хода"""
        try:
            response = requests.post(f"{self.api_url}/chess/complete-move", 
                                   json={"drone_name": drone_name})
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error completing chess move: {e}")
            return None
    
    def get_board_status(self):
        """Получить статус доски"""
        try:
            response = requests.get(f"{self.api_url}/chess/board")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error getting board status: {e}")
            return None


def wait_for_chess_move(drone_name, chess_api=None, check_interval=1.0):
    """Ожидает ход для конкретного дрона в шахматной игре"""
    logger = setup_logging(drone_name)
    if chess_api is None:
        chess_api = ChessAPI(drone_name)
    
    logger.info("♟️  Waiting for chess move...")
    
    while not rospy.is_shutdown():
        game_state = chess_api.get_game_state()
        
        if not game_state or game_state.get('status') != 'active':
            logger.info("Game not active, waiting...")
            time.sleep(check_interval)
            continue
        
        # Проверяем, наш ли ход
        current_turn = game_state.get('current_turn')
        if current_turn != drone_name:
            logger.debug(f"Waiting for turn... Current: {current_turn}")
            time.sleep(check_interval)
            continue
        
        # Получаем данные хода
        move_data = game_state.get('move')
        if not move_data:
            logger.warning("No move data available")
            time.sleep(check_interval)
            continue
        
        # Парсим координаты хода
        from_pos = move_data.get('from', [0, 0])
        to_pos = move_data.get('to', [0, 0])
        move_z = move_data.get('z', 1.2)
        target_aruco = move_data.get('aruco_id', 89)
        
        logger.info(f"♟️  Chess move! From ({from_pos[0]:.3f}, {from_pos[1]:.3f}) to ({to_pos[0]:.3f}, {to_pos[1]:.3f}) z={move_z:.3f} (ArUco: {target_aruco})")
        
        return ChessCoords(
            from_x=from_pos[0], from_y=from_pos[1],
            to_x=to_pos[0], to_y=to_pos[1],
            z=move_z, aruco_id=target_aruco
        )
    
    return None


def complete_chess_move(drone_name, chess_api=None):
    """Сообщает серверу о завершении хода"""
    logger = setup_logging(drone_name)
    if chess_api is None:
        chess_api = ChessAPI(drone_name)
    
    result = chess_api.complete_move(drone_name)
    if result:
        logger.info("✅ Chess move completed successfully")
        return True
    else:
        logger.error("❌ Failed to complete chess move")
        return False


class ChessDrone:
    """Шахматный дрон с использованием FlightControllerMain"""
    
    def __init__(self):
        # Получаем имя дрона из переменной окружения
        self.drone_name = os.environ.get('DRONE_NAME', 'unknown_drone')
        print(f"Chess drone running on: {self.drone_name}")
        
        # Инициализируем логгер
        self.logger = setup_logging(self.drone_name)
        
        # Инициализируем FlightController (Main implementation)
        os.environ['FLIGHT_IMPL'] = 'main'  # Используем FlightControllerMain
        self.fc = FlightController(drone_name=self.drone_name, logger=self.logger)
        
        # API для взаимодействия с сервером
        self.chess_api = ChessAPI(self.drone_name)
        
        # Параметры полета
        self.flight_params = {
            'takeoff_z': 1.5,
            'speed': 0.3,
            'tolerance': 0.1,
            'hover_time': 1.0,
            'bias_x': 0.0,
            'bias_y': 0.0
        }
    
    def execute_chess_move(self, chess_coords):
        """Выполнить шахматный ход: дрон перемещается в новую позицию"""
        
        self.logger.info(f"♟️  Chess move: ({chess_coords.from_x:.2f},{chess_coords.from_y:.2f}) → ({chess_coords.to_x:.2f},{chess_coords.to_y:.2f})")
        
        try:
            # Простое перемещение дрона-фигуры в новую позицию
            self.fc.navigate_and_land(
                x=chess_coords.to_x + self.flight_params.get('bias_x', 0),
                y=chess_coords.to_y + self.flight_params.get('bias_y', 0),
                z=chess_coords.z,
                takeoff_z=self.flight_params['takeoff_z'],
                speed=self.flight_params['speed'],
                frame_id=f"aruco_{chess_coords.aruco_id}",
                tolerance=self.flight_params['tolerance'],
                hover_time=self.flight_params['hover_time']
            )
            
            self.logger.info("✅ Chess move completed!")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Chess move failed: {e}")
            return False
    
    def run(self):
        """Основной цикл работы шахматного дрона"""
        self.logger.info("♟️  Starting chess drone...")
        
        # Проверяем телеметрию
        try:
            telem = self.fc.get_telemetry(frame_id="aruco_map")
            self.logger.info(f"Initial telemetry - Mode: {telem.mode}, Armed: {telem.armed}")
        except Exception as e:
            self.logger.warning(f"Could not get initial telemetry: {e}")
        
        # Основной игровой цикл
        while not rospy.is_shutdown():
            try:
                # Ожидаем наш ход
                chess_move = wait_for_chess_move(self.drone_name, self.chess_api)
                
                if chess_move is None:
                    self.logger.warning("No chess move received, continuing...")
                    continue
                
                # Выполняем ход
                success = self.execute_chess_move(chess_move)
                
                if success:
                    # Сообщаем серверу о завершении хода
                    complete_chess_move(self.drone_name, self.chess_api)
                else:
                    self.logger.error("Failed to execute chess move")
                
                # Пауза перед следующим ходом
                self.fc.wait(2.0)
                
            except KeyboardInterrupt:
                self.logger.info("Chess drone interrupted by user")
                break
            except Exception as e:
                self.logger.error(f"Error in chess drone main loop: {e}")
                self.fc.wait(5.0)  # Пауза при ошибке
        
        self.logger.info("♟️  Chess drone stopped")


if __name__ == "__main__":
    # Инициализируем ROS node если доступен
    try:
        rospy.init_node('chess_drone')
    except:
        pass
    
    # Создаем и запускаем шахматного дрона
    chess_drone = ChessDrone()
    chess_drone.run()
