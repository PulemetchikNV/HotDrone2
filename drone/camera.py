import os
import time
import json
import requests
from typing import Dict, Any, Optional
from dataclasses import dataclass

try:
    from .helpers import setup_logging
except ImportError:
    from helpers import setup_logging


@dataclass
class PiecePosition:
    """Позиция фигуры на доске"""
    x: float
    y: float
    cell: str
    piece_type: str
    color: str

    def get(self, key: str, default: Any = None) -> Any:
        return getattr(self, key, default)


class CameraError(Exception):
    """Базовая ошибка камеры"""
    pass


class CameraTemporaryError(CameraError):
    """Временная ошибка камеры (плохая связь, таймаут)"""
    pass


class CameraPermanentError(CameraError):
    """Постоянная ошибка камеры (неверный URL, формат данных)"""
    pass


class CameraController:
    """Контроллер для получения данных с камеры"""
    
    def __init__(self, logger=None):
        self.logger = logger or setup_logging("camera")
        self.api_url = os.getenv("CAMERA_API_URL", "")
        self.timeout = float(os.getenv("CAMERA_TIMEOUT", "5.0"))
        self.retry_count = int(os.getenv("CAMERA_RETRIES", "3"))
        
        if not self.api_url:
            self.logger.warning("CAMERA_API_URL not set, camera disabled")
        else:
            self.logger.info(f"Camera controller initialized: URL={self.api_url}")
    
    def get_board_positions(self) -> Dict[str, Dict[str, PiecePosition]]:
        """
        Получает позиции фигур с камеры
        
        Returns:
            Dict с структурой: {'black': {'king': PiecePosition(...)}, 'white': {...}}
        """
        if not self.api_url:
            raise CameraPermanentError("Camera API URL not configured")
        
        for attempt in range(self.retry_count):
            try:
                self.logger.debug(f"Requesting camera data (attempt {attempt + 1}/{self.retry_count})")
                response = requests.get(self.api_url, timeout=self.timeout)
                response.raise_for_status()
                
                # Парсим JSON ответ
                raw_data = response.json()
                return self._parse_camera_data(raw_data)
                
            except requests.exceptions.Timeout:
                self.logger.warning(f"Camera request timeout (attempt {attempt + 1})")
                if attempt == self.retry_count - 1:
                    raise CameraTemporaryError("Camera request timeout after retries")
                time.sleep(0.5)
                
            except requests.exceptions.ConnectionError:
                self.logger.warning(f"Camera connection error (attempt {attempt + 1})")
                if attempt == self.retry_count - 1:
                    raise CameraTemporaryError("Camera connection failed after retries")
                time.sleep(0.5)
                
            except requests.exceptions.HTTPError as e:
                self.logger.error(f"Camera HTTP error: {e}")
                raise CameraPermanentError(f"Camera HTTP error: {e}")
                
            except json.JSONDecodeError as e:
                self.logger.error(f"Camera JSON decode error: {e}")
                raise CameraPermanentError(f"Invalid JSON response from camera: {e}")
                
            except Exception as e:
                self.logger.error(f"Unexpected camera error: {e}")
                if attempt == self.retry_count - 1:
                    raise CameraTemporaryError(f"Unexpected camera error: {e}")
                time.sleep(0.5)
        
        raise CameraTemporaryError("Camera requests failed after all retries")
    
    def _parse_camera_data(self, raw_data: Dict) -> Dict[str, Dict[str, PiecePosition]]:
        """
        Парсит сырые данные камеры в структуру позиций
        
        Ожидаемый формат:
        {
            'black': {'king': {'x': 1.0, 'y': 1.0, 'cell': 'a1'}},
            'white': {
                'queen': {'x': 0.5, 'y': -0.5, 'cell': 'g6'},
                'pawn_a2': {'x': -3.5, 'y': -2.5, 'cell': 'a2'},
                'pawn_b2': {'x': -2.5, 'y': -2.5, 'cell': 'b2'},
                'rook_a1': {'x': -3.5, 'y': -3.5, 'cell': 'a1'},
                'knight_b1': {'x': -2.5, 'y': -3.5, 'cell': 'b1'}
            }
        }
        """
        result = {}
        
        for color in ['black', 'white']:
            if color not in raw_data:
                continue
                
            result[color] = {}
            color_data = raw_data[color]
            
            if not isinstance(color_data, dict):
                continue
                
            for piece_type, piece_data in color_data.items():
                try:
                    if not isinstance(piece_data, dict):
                        continue
                    
                    x = float(piece_data.get('x', 0))
                    y = float(piece_data.get('y', 0))
                    cell = str(piece_data.get('cell', ''))
                    
                    # Валидация клетки
                    if not self._is_valid_cell(cell):
                        self.logger.warning(f"Invalid cell '{cell}' for {color} {piece_type}")
                        continue
                    
                    position = PiecePosition(
                        x=x,
                        y=y,
                        cell=cell,
                        piece_type=piece_type,
                        color=color
                    )
                    
                    result[color][piece_type] = position
                    
                except (ValueError, TypeError) as e:
                    self.logger.warning(f"Invalid piece data for {color} {piece_type}: {e}")
                    continue
        
        return result
    
    def _is_valid_cell(self, cell: str) -> bool:
        """Проверяет валидность шахматной клетки"""
        if not isinstance(cell, str) or len(cell) != 2:
            return False
        
        file_char, rank_char = cell[0].lower(), cell[1]
        return (file_char in 'abcdefgh' and rank_char in '12345678')
    
    def get_pieces_on_cell(self, cell: str) -> list:
        """Получает все фигуры на указанной клетке"""
        try:
            positions = self.get_board_positions()
            pieces = []
            
            for color_data in positions.values():
                for piece in color_data.values():
                    if piece.cell == cell:
                        pieces.append(piece)
            
            return pieces
            
        except CameraError:
            # При ошибках камеры возвращаем пустой список
            return []
    
    def get_all_pawns(self) -> Dict[str, list]:
        """
        Получает все пешки с их ID, сгруппированные по цвету
        
        Returns:
            Dict: {'white': [PiecePosition, ...], 'black': [PiecePosition, ...]}
        """
        try:
            positions = self.get_board_positions()
            pawns = {'white': [], 'black': []}
            
            for color, color_data in positions.items():
                for piece_type, piece_pos in color_data.items():
                    if piece_type.lower() == 'pawn' or piece_type.lower().startswith('pawn_'):
                        pawns[color].append(piece_pos)
            
            return pawns
            
        except CameraError:
            return {'white': [], 'black': []}
    
    def get_pawn_by_initial_cell(self, initial_cell: str) -> Optional[PiecePosition]:
        """
        Находит пешку по её начальной клетке
        
        Args:
            initial_cell: Начальная клетка пешки (например, "a2", "b7")
            
        Returns:
            PiecePosition или None если не найдена
        """
        try:
            positions = self.get_board_positions()
            target_piece_type = f"pawn_{initial_cell.lower()}"
            
            for color_data in positions.values():
                for piece_type, piece_pos in color_data.items():
                    if piece_type.lower() == target_piece_type.lower():
                        return piece_pos
            
            return None
            
        except CameraError:
            return None
    
    def get_piece_by_initial_cell(self, piece_base_type: str, initial_cell: str) -> Optional[PiecePosition]:
        """
        Находит фигуру по типу и начальной клетке
        
        Args:
            piece_base_type: Базовый тип фигуры (например, "rook", "knight")
            initial_cell: Начальная клетка (например, "a1", "b1")
            
        Returns:
            PiecePosition или None если не найдена
        """
        try:
            positions = self.get_board_positions()
            
            # Для уникальных фигур (король, ферзь) ищем без суффикса
            if piece_base_type.lower() in ['king', 'queen']:
                target_piece_type = piece_base_type.lower()
            else:
                # Для остальных фигур используем формат {тип}_{начальная_клетка}
                target_piece_type = f"{piece_base_type.lower()}_{initial_cell.lower()}"
            
            for color_data in positions.values():
                for piece_type, piece_pos in color_data.items():
                    if piece_type.lower() == target_piece_type.lower():
                        return piece_pos
            
            return None
            
        except CameraError:
            return None
    
    def is_game_paused(self) -> bool:
        """
        Проверяет, стоит ли игра на паузе
        
        Returns:
            bool: True если игра на паузе, False если игра продолжается
        """
        try:
            # Используем тот же URL что и для позиций, но другой endpoint
            base_url = self.api_url.replace('/api/positions', '')
            pause_url = f"{base_url}/api/pause_status"
            
            response = requests.get(pause_url, timeout=self.timeout)
            response.raise_for_status()
            
            data = response.json()
            return data.get('is_paused', False)
            
        except Exception as e:
            self.logger.debug(f"Failed to check pause status: {e}")
            # При ошибке считаем что игра НЕ на паузе (безопасный fallback)
            return False


class MockCameraController(CameraController):
    """Мок-контроллер камеры, привязанный к camera_mock API"""
    
    def __init__(self, logger=None):
        super().__init__(logger)
        # Если URL не задан, используем дефолт локального мок-сервера
        self.api_url = os.getenv("CAMERA_API_URL", "http://127.0.0.1:8001/api/positions")
        self.logger.info(f"Mock camera controller bound to {self.api_url}")
        self._last_turn: Optional[str] = None
    
    def get_board_positions(self) -> Dict[str, Dict[str, PiecePosition]]:
        """Получает позиции фигур из camera_mock"""
        if not self.api_url:
            raise CameraPermanentError("Camera API URL not configured for mock")
        
        for attempt in range(self.retry_count):
            try:
                self.logger.debug(f"Requesting mock camera data (attempt {attempt + 1}/{self.retry_count})")
                response = requests.get(self.api_url, timeout=self.timeout)
                response.raise_for_status()
                raw_data = response.json()
                # Сохраняем чей ход, если пришло
                turn = raw_data.get('turn')
                if isinstance(turn, str):
                    self._last_turn = turn.lower()
                # Парсим фигуры тем же парсером
                return self._parse_camera_data(raw_data)
            except requests.exceptions.Timeout:
                self.logger.warning(f"Mock camera request timeout (attempt {attempt + 1})")
                if attempt == self.retry_count - 1:
                    raise CameraTemporaryError("Mock camera request timeout after retries")
                time.sleep(0.3)
            except requests.exceptions.ConnectionError:
                self.logger.warning(f"Mock camera connection error (attempt {attempt + 1})")
                if attempt == self.retry_count - 1:
                    raise CameraTemporaryError("Mock camera connection failed after retries")
                time.sleep(0.3)
            except requests.exceptions.HTTPError as e:
                self.logger.error(f"Mock camera HTTP error: {e}")
                raise CameraPermanentError(f"Mock camera HTTP error: {e}")
            except Exception as e:
                self.logger.error(f"Unexpected mock camera error: {e}")
                if attempt == self.retry_count - 1:
                    raise CameraTemporaryError(f"Unexpected mock camera error: {e}")
                time.sleep(0.3)
        raise CameraTemporaryError("Mock camera requests failed after all retries")

    def get_turn(self) -> Optional[str]:
        """Возвращает чей сейчас ход, если известен (white/black)"""
        return self._last_turn
    
    def is_game_paused(self) -> bool:
        """
        Проверяет, стоит ли игра на паузе (переопределяем для mock контроллера)
        
        Returns:
            bool: True если игра на паузе, False если игра продолжается
        """
        try:
            # Используем base URL из api_url
            base_url = self.api_url.replace('/api/positions', '')
            pause_url = f"{base_url}/api/pause_status"
            
            response = requests.get(pause_url, timeout=self.timeout)
            response.raise_for_status()
            
            data = response.json()
            is_paused = data.get('is_paused', False)
            
            if is_paused:
                self.logger.info("Game is paused")
            else:
                self.logger.debug("Game is running")
                
            return is_paused
            
        except Exception as e:
            self.logger.debug(f"Failed to check pause status: {e}")
            # При ошибке считаем что игра НЕ на паузе (безопасный fallback)
            return False


def positions_to_fen(positions: dict, turn: str) -> str:
    FILES = "abcdefgh"
    RANKS = "12345678"
    # Построим 8x8 сетку пустых клеток
    board = [[None for _ in range(8)] for _ in range(8)]

    def base_to_symbol(base: str, color: str) -> str:
        m = {
            'king': 'k', 'queen': 'q', 'rook': 'r', 'knight': 'n', 'bishop': 'b', 'pawn': 'p'
        }
        s = m.get(base, 'p')
        return s.upper() if color == 'white' else s

    def put(color: str, piece_key: str, cell: str):
        if not (isinstance(cell, str) and len(cell) == 2 and cell[0] in FILES and cell[1] in RANKS):
            return
        file_idx = FILES.index(cell[0])
        rank_idx = RANKS.index(cell[1])
        row = 8 - int(cell[1])  # 0..7, где 0 = 8-я горизонталь
        col = file_idx          # 0..7, где 0 = столбец 'a'
        base = piece_key.split('_')[0]
        board[row][col] = base_to_symbol(base, color)

    for color, color_data in positions.items():
        for piece_key, info in color_data.items():
            put(color, piece_key, info.get('cell', ''))

    # Конвертируем в FEN piece placement
    ranks = []
    for row in range(8):
        empty = 0
        fen_row = ''
        for col in range(8):
            sym = board[row][col]
            if sym is None:
                empty += 1
            else:
                if empty > 0:
                    fen_row += str(empty)
                    empty = 0
                fen_row += sym
        if empty > 0:
            fen_row += str(empty)
        ranks.append(fen_row)

    placement = '/'.join(ranks)
    active = 'w' if turn == 'white' else 'b'
    # Убираем рокировки, en passant, счетчики — простая позиция без возможности рокировки
    return f"{placement} {active} - - 0 1"


def get_board_state_from_camera(camera_controller) -> dict:
    """
    Получает состояние доски с камеры и формирует структуру для alg
    
    Args:
        camera_controller: Экземпляр CameraController/MockCameraController
        
    Returns:
        Dict с информацией о доске
    """
    positions = camera_controller.get_board_positions()
    
    # Определяем текущую клетку (первая найденная фигура)
    current_cell = os.getenv("START_CELL", "e2")
    for color in ("white", "black"):
        if color in positions and isinstance(positions[color], dict):
            for piece in positions[color].values():
                if hasattr(piece, 'cell'):
                    current_cell = piece.cell
                    break
            if current_cell != os.getenv("START_CELL", "e2"):
                break
    
    # Чей ход
    turn = 'white'
    if hasattr(camera_controller, 'get_turn'):
        camera_turn = camera_controller.get_turn()
        if camera_turn:
            turn = camera_turn.lower()

    print(f"GOT TURN: {turn}")
    
    # Генерируем FEN
    fen = positions_to_fen(positions, turn)

    print(f"GOT FEN: {fen}")
    
    return {
        'positions': positions,
        'current_cell': current_cell,
        'turn': 'w' if turn.startswith('w') else 'b',
        'fen': fen,
        'timestamp': time.time()
    }


def create_camera_controller(logger=None) -> CameraController:
    """
    Фабрика для создания контроллера камеры
    
    Выбирает реализацию на основе переменной окружения CAMERA_IMPL:
    - 'mock': MockCameraController
    - 'real' или любое другое: CameraController
    """
    impl = os.getenv('CAMERA_IMPL', 'real').lower()
    
    if impl == 'mock':
        return MockCameraController(logger=logger)
    else:
        return CameraController(logger=logger)
