import os
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Union
import httpx

from const import OUR_TEAM, get_current_drone_config

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

# Импорт общих типов и исключений
from utils import BoardState, MoveDecision, AlgTemporaryError, AlgPermanentError
from board_utils import get_board_state, get_board_state_from_camera

# -----------------------------
# Источник данных камеры
# -----------------------------
FILES = "abcdefgh"
RANKS = "12345678"

# Убрали _camera_controller - используем board_utils


def _normalize_cell(cell: str) -> str:
    if not isinstance(cell, str) or len(cell) != 2:
        raise AlgPermanentError("cell must be like 'e2'")
    f, r = cell[0].lower(), cell[1]
    if f not in FILES or r not in RANKS:
        raise AlgPermanentError(f"invalid cell: {cell}")
    return f + r


# get_board_state теперь импортируется из board_utils


def get_turn(board: BoardState, time_budget_ms: int = 5000, seed: Optional[int] = None) -> MoveDecision:
    """Возвращает следующий ход, предпочитая Stockfish API; при сбое — простой фолбэк."""
    if board is None or not isinstance(board, BoardState):
        raise AlgPermanentError("board must be BoardState")

    fen = board.fen
    print(f"SENDING FEN TO SF: {fen}")
    # Попытка получить ход от chess-api.com (Stockfish)
    try:
        timeout_s = max(1.0, float(time_budget_ms) / 1000.0)
        with httpx.Client(timeout=timeout_s) as client:
            resp = client.post("https://chess-api.com/v1", json={"fen": fen})
            resp.raise_for_status()
            data = resp.json()


        print(f"==== RAW STOCKFISH DATA: {data}")
        # Извлекаем ход в UCI, поддерживаем промоции
        uci_move = (
            (data.get("move") or data.get("lan") or "").strip()
        )
        # Получаем цвет команды из конфигурации дрона
        try:
            drone_config = get_current_drone_config()
            our_color = drone_config['team']
        except (ValueError, KeyError):
            # Фолбэк на константу
            our_color = OUR_TEAM
        stockfish_color = data.get("color", "w")
        if stockfish_color.lower() != our_color[0].lower():
            print(f"NOT OUR TURN: {stockfish_color} != {our_color[0]}")
            return MoveDecision(
                uci="",
                from_cell="",
                to_cell="",
            )

        from_cell = data.get("from")
        to_cell = data.get("to")

        if (not from_cell or not to_cell) and len(uci_move) in (4, 5):
            from_cell = uci_move[0:2]
            to_cell = uci_move[2:4]

        if not (from_cell and to_cell):
            raise AlgTemporaryError("Stockfish API returned no parsable move")

        # Сборка итоговой структуры
        uci = uci_move if len(uci_move) >= 4 else f"{from_cell}{to_cell}"
        cp = None
        try:
            cp = int(str(data.get("centipawns", "")).strip()) if data.get("centipawns") else None
        except Exception:
            cp = None

        return MoveDecision(
            uci=uci,
            from_cell=_normalize_cell(from_cell),
            to_cell=_normalize_cell(to_cell),
            score_cp=cp,
            is_mate=bool(data.get("mate")),
            reason="stockfish-api",
            meta={
                "depth": data.get("depth"),
                "eval": data.get("eval"),
                "text": data.get("text"),
                "flags": data.get("flags"),
            },
        )
    except Exception:
        # Фолбэк: простой детерминированный ход на основе current_cell
        meta = board.meta or {}
        cur = meta.get("current_cell")
        if not cur:
            raise AlgPermanentError("board.meta.current_cell is required")
        cur = _normalize_cell(cur)
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
            reason="fallback-simple-sequence",
            meta={"policy": "simple_sequence"},
        )
