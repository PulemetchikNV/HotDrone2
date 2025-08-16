"""
Общие функции для работы с доской (get_board_state и связанные).
"""
import os
import time
from utils import BoardState, AlgTemporaryError, AlgPermanentError
from camera import create_camera_controller, CameraTemporaryError, CameraPermanentError

FILES = "abcdefgh"
RANKS = "12345678"

_camera_controller = None


def _get_camera_controller():
    global _camera_controller
    if _camera_controller is None:
        _camera_controller = create_camera_controller()
    return _camera_controller


def _normalize_cell(cell: str) -> str:
    if not isinstance(cell, str) or len(cell) != 2:
        raise AlgPermanentError("cell must be like 'e2'")
    f, r = cell[0].lower(), cell[1]
    if f not in FILES or r not in RANKS:
        raise AlgPermanentError(f"invalid cell: {cell}")
    return f + r


def _camera_read_board():
    """Считывает данные с камеры и возвращает информацию о доске."""
    try:
        camera = _get_camera_controller()
        positions = camera.get_board_positions()
        board_info = {
            'positions': positions,
            'timestamp': time.time(),
        }
        # current_cell: выбираем первую фигуру для детерминизма
        current_cell = os.getenv("START_CELL", "e2")
        for color in ("white", "black"):
            if color in positions and isinstance(positions[color], dict):
                for piece in positions[color].values():
                    current_cell = piece.cell
                    break
                if current_cell != os.getenv("START_CELL", "e2"):
                    break
        board_info['current_cell'] = _normalize_cell(current_cell)
        
        # чей ход — если контроллер умеет, заберём; иначе white
        turn = getattr(camera, 'get_turn', lambda: None)() or 'white'
        board_info['turn'] = 'w' if turn.lower().startswith('w') else 'b'
        
        # Получаем FEN: используем функцию конвертации из camera.py
        from camera import positions_to_fen
        turn_word = 'white' if board_info['turn'] == 'w' else 'black'
        fen = positions_to_fen(positions, turn_word)
        
        board_info['fen'] = fen
        return board_info
    except CameraTemporaryError as e:
        raise AlgTemporaryError(f"Camera temporary error: {e}")
    except CameraPermanentError as e:
        raise AlgPermanentError(f"Camera permanent error: {e}")
    except Exception as e:
        raise AlgTemporaryError(f"Unexpected camera error: {e}")


def get_board_state() -> BoardState:
    """
    Универсальная функция получения состояния доски.
    Используется всеми алгоритмами для получения текущего состояния.
    """
    board_info = _camera_read_board()
    
    meta = {
        "current_cell": board_info['current_cell'],
        "positions": board_info['positions'],
        "camera_timestamp": board_info['timestamp']
    }
    
    return BoardState(
        fen=board_info['fen'],
        turn=board_info['turn'],
        move_number=1,
        timestamp=time.time(),
        meta=meta,
    )


def get_board_state_from_camera(camera):
    """
    Совместимость с alg_mock2.py - получение состояния с переданной камеры.
    """
    try:
        positions = camera.get_board_positions()
        board_info = {
            'positions': positions,
            'timestamp': time.time(),
        }
        
        # current_cell: выбираем первую фигуру для детерминизма
        current_cell = os.getenv("START_CELL", "e2")
        for color in ("white", "black"):
            if color in positions and isinstance(positions[color], dict):
                for piece in positions[color].values():
                    current_cell = piece.cell
                    break
                if current_cell != os.getenv("START_CELL", "e2"):
                    break
        board_info['current_cell'] = _normalize_cell(current_cell)
        
        # чей ход — если контроллер умеет, заберём; иначе white
        turn = getattr(camera, 'get_turn', lambda: None)() or 'white'
        board_info['turn'] = 'w' if turn.lower().startswith('w') else 'b'
        
        # Получаем FEN: используем функцию конвертации из camera.py
        from camera import positions_to_fen
        turn_word = 'white' if board_info['turn'] == 'w' else 'black'
        fen = positions_to_fen(positions, turn_word)
        
        board_info['fen'] = fen
        return board_info
    except CameraTemporaryError as e:
        raise AlgTemporaryError(f"Camera temporary error: {e}")
    except CameraPermanentError as e:
        raise AlgPermanentError(f"Camera permanent error: {e}")
    except Exception as e:
        raise AlgTemporaryError(f"Unexpected camera error: {e}")
