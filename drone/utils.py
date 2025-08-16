"""
Общие типы данных и исключения для шахматных алгоритмов.
"""
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Union

# Для Python 3.7 совместимости
try:
    from typing import Literal
except ImportError:
    try:
        from typing_extensions import Literal
    except ImportError:
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
