import os
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Union

# Для Python 3.7 совместимости
try:
    from typing import Literal
except ImportError:
    # Fallback для Python < 3.8
    try:
        from typing_extensions import Literal
    except ImportError:
        # Если typing_extensions тоже нет, создаем заглушку
        def Literal(*args):
            return str

from camera import create_camera_controller, CameraTemporaryError, CameraPermanentError


# -----------------------------
# Исключения уровня алгоритма
# -----------------------------
class AlgTemporaryError(Exception):
    """Временная ошибка (плохой кадр/низкая уверенность). Можно повторить позже."""


class AlgPermanentError(Exception):
    """Постоянная ошибка (некорректная позиция или входные данные)."""


# -----------------------------
# Модели данных контракта
# -----------------------------
@dataclass(frozen=True)
class BoardState:
    fen: str
    turn: Literal["w", "b"]
    move_number: int
    timestamp: float
    meta: Union[Dict[str, Any], None] = None


@dataclass(frozen=True)
class MoveDecision:
    uci: str
    from_cell: str
    to_cell: str
    score_cp: Optional[int] = None
    is_mate: bool = False
    reason: Optional[str] = None
    meta: Union[Dict[str, Any], None] = None

# -----------------------------
# Источник данных камеры
# -----------------------------
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


def _camera_read_board() -> Dict[str, Any]:
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
        return board_info
    except CameraTemporaryError as e:
        raise AlgTemporaryError(f"Camera temporary error: {e}")
    except CameraPermanentError as e:
        raise AlgPermanentError(f"Camera permanent error: {e}")
    except Exception as e:
        raise AlgTemporaryError(f"Unexpected camera error: {e}")


def get_board_state() -> BoardState:
    """Считывает текущее состояние с камеры и возвращает BoardState."""
    board_info = _camera_read_board()
    # Пока fen упрощённый — берём из позиций через мок-сервер, либо дефолт
    fen = os.getenv("DEFAULT_FEN", "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w - - 0 1")
    meta = {
        "current_cell": board_info['current_cell'],
        "positions": board_info['positions'],
        "camera_timestamp": board_info['timestamp']
    }
    return BoardState(
        fen=fen,
        turn=board_info['turn'],
        move_number=1,
        timestamp=time.time(),
        meta=meta,
    )


def get_turn(board: BoardState, time_budget_ms: int = 5000, seed: Optional[int] = None) -> MoveDecision:
    """Возвращает следующий ход (мок-логика детерминированна)."""
    if board is None or not isinstance(board, BoardState):
        raise AlgPermanentError("board must be BoardState")
    meta = board.meta or {}
    cur = meta.get("current_cell")
    if not cur:
        raise AlgPermanentError("board.meta.current_cell is required")
    cur = _normalize_cell(cur)
    # Простая последовательность
    f_idx = FILES.index(cur[0])
    r_idx = RANKS.index(cur[1])
    if r_idx < 7:
        to_cell = f"{cur[0]}{RANKS[r_idx+1]}"
    elif f_idx < 7:
        to_cell = f"{FILES[f_idx+1]}1"
    else:
        to_cell = "a1"
    uci = f"{cur}{to_cell}"
    return MoveDecision(
        uci=uci,
        from_cell=cur,
        to_cell=to_cell,
        score_cp=0,
        is_mate=False,
        reason="mock-next-cell",
        meta={"policy": "simple_sequence"},
    )


def update_after_execution(prev: BoardState, move: MoveDecision, success: bool) -> BoardState:
    """Обновляет предсказанное состояние после попытки исполнения хода."""
    if prev is None or move is None:
        raise AlgPermanentError("prev and move are required")
    if not success:
        return BoardState(
            fen=prev.fen,
            turn=prev.turn,
            move_number=prev.move_number,
            timestamp=time.time(),
            meta=prev.meta,
        )
    meta = dict(prev.meta or {})
    meta["current_cell"] = _normalize_cell(move.to_cell)
    next_turn: Literal["w", "b"] = "b" if prev.turn == "w" else "w"
    return BoardState(
        fen=prev.fen,
        turn=next_turn,
        move_number=max(1, prev.move_number) + 1,
        timestamp=time.time(),
        meta=meta,
    )
