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
# Внутренний мок-источник данных камеры
# -----------------------------
FILES = "abcdefgh"
RANKS = "12345678"


def _normalize_cell(cell: str) -> str:
    if not isinstance(cell, str) or len(cell) != 2:
        raise AlgPermanentError("cell must be like 'e2'")
    f, r = cell[0].lower(), cell[1]
    if f not in FILES or r not in RANKS:
        raise AlgPermanentError(f"invalid cell: {cell}")
    return f + r


def _camera_mock_read_cell() -> str:
    # В реальном коде здесь будет чтение с камеры + распознавание.
    # Пока используем стабильный источник из переменной окружения.
    cell = os.getenv("START_CELL", "e2")
    return _normalize_cell(cell)


def _next_cell_simple(current: str) -> str:
    # Идём по рангам вверх, затем вправо и снова снизу
    current = _normalize_cell(current)
    f_idx = FILES.index(current[0])
    r_idx = RANKS.index(current[1])
    if r_idx < 7:
        return f"{current[0]}{RANKS[r_idx+1]}"
    if f_idx < 7:
        return f"{FILES[f_idx+1]}1"
    return "a1"


# -----------------------------
# Публичный контракт API
# -----------------------------
def get_board_state() -> BoardState:
    """Считывает текущее состояние с камеры и возвращает BoardState.

    Сейчас реализован мок: берём текущую клетку из START_CELL.
    """
    try:
        current_cell = _camera_mock_read_cell()
    except AlgPermanentError:
        # Прокидываем дальше как фатальную ошибку
        raise
    except Exception as e:
        # Любые иные сбои считаем временными
        raise AlgTemporaryError(str(e))

    # Валидный FEN по умолчанию (стартовая позиция)
    default_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
    ts = time.time()
    return BoardState(
        fen=default_fen,
        turn="w",
        move_number=1,
        timestamp=ts,
        meta={"current_cell": current_cell},
    )


def get_turn(board: BoardState, time_budget_ms: int = 5000, seed: Optional[int] = None) -> MoveDecision:
    """Вычисляет следующий ход для данной позиции.

    В мок-реализации: двигаемся в "следующую" клетку по детерминированному правилу.
    """
    if board is None or not isinstance(board, BoardState):
        raise AlgPermanentError("board must be BoardState")

    meta = board.meta or {}
    cur = meta.get("current_cell")
    if not cur:
        raise AlgPermanentError("board.meta.current_cell is required")

    cur = _normalize_cell(cur)
    to_cell = _next_cell_simple(cur)
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
        # Ничего не меняем, только отметим время
        return BoardState(
            fen=prev.fen,
            turn=prev.turn,
            move_number=prev.move_number,
            timestamp=time.time(),
            meta=prev.meta,
        )

    # Успешный ход: обновим текущую клетку и счётчики
    meta = dict(prev.meta or {})
    meta["current_cell"] = _normalize_cell(move.to_cell)

    # Переключим очередь хода (для валидности модели данных)
    next_turn: Literal["w", "b"] = "b" if prev.turn == "w" else "w"

    return BoardState(
        fen=prev.fen,  # В мок-версии fen не пересчитываем
        turn=next_turn,
        move_number=max(1, prev.move_number) + 1,
        timestamp=time.time(),
        meta=meta,
    )


