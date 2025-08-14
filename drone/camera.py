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


class MockCameraController(CameraController):
    """Мок-контроллер камеры для тестирования"""
    
    def __init__(self, logger=None):
        super().__init__(logger)
        self.logger.info("Mock camera controller initialized")
        
        # Мок данные по умолчанию в новом формате {тип}_{начальная_клетка}
        self.mock_data = {
            'white': {
                'king': {'x': 0.0, 'y': -3.5, 'cell': 'e1'},
                'queen': {'x': -0.5, 'y': -3.5, 'cell': 'd1'},
                'rook_a1': {'x': -3.5, 'y': -3.5, 'cell': 'a1'},
                'rook_h1': {'x': 3.5, 'y': -3.5, 'cell': 'h1'},
                'knight_b1': {'x': -2.5, 'y': -3.5, 'cell': 'b1'},
                'knight_g1': {'x': 2.5, 'y': -3.5, 'cell': 'g1'},
                'bishop_c1': {'x': -1.5, 'y': -3.5, 'cell': 'c1'},
                'bishop_f1': {'x': 1.5, 'y': -3.5, 'cell': 'f1'},
                'pawn_a2': {'x': -3.5, 'y': -2.5, 'cell': 'a2'},
                'pawn_b2': {'x': -2.5, 'y': -2.5, 'cell': 'b2'},
                'pawn_c2': {'x': -1.5, 'y': -2.5, 'cell': 'c2'},
                'pawn_d2': {'x': -0.5, 'y': -2.5, 'cell': 'd2'}
            },
            'black': {
                'king': {'x': 0.0, 'y': 3.5, 'cell': 'e8'},
                'queen': {'x': -0.5, 'y': 3.5, 'cell': 'd8'},
                'rook_a8': {'x': -3.5, 'y': 3.5, 'cell': 'a8'},
                'rook_h8': {'x': 3.5, 'y': 3.5, 'cell': 'h8'},
                'knight_b8': {'x': -2.5, 'y': 3.5, 'cell': 'b8'},
                'knight_g8': {'x': 2.5, 'y': 3.5, 'cell': 'g8'},
                'bishop_c8': {'x': -1.5, 'y': 3.5, 'cell': 'c8'},
                'bishop_f8': {'x': 1.5, 'y': 3.5, 'cell': 'f8'},
                'pawn_a7': {'x': -3.5, 'y': 2.5, 'cell': 'a7'},
                'pawn_b7': {'x': -2.5, 'y': 2.5, 'cell': 'b7'},
                'pawn_c7': {'x': -1.5, 'y': 2.5, 'cell': 'c7'},
                'pawn_d7': {'x': -0.5, 'y': 2.5, 'cell': 'd7'}
            }
        }
    
    def get_board_positions(self) -> Dict[str, Dict[str, PiecePosition]]:
        """Возвращает мок-данные позиций"""
        # Имитируем небольшую задержку сети
        time.sleep(0.1)
        
        # Парсим мок данные через тот же парсер
        return self._parse_camera_data(self.mock_data)
    
    def set_mock_data(self, mock_data: Dict):
        """Устанавливает новые мок-данные"""
        self.mock_data = mock_data
        self.logger.debug("Mock camera data updated")


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
