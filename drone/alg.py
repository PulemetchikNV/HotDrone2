from __future__ import annotations

import os
import subprocess
import time
from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any


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
    meta: Dict[str, Any] | None = None


@dataclass(frozen=True)
class MoveDecision:
    uci: str
    from_cell: str
    to_cell: str
    score_cp: Optional[int] = None
    is_mate: bool = False
    reason: Optional[str] = None
    meta: Dict[str, Any] | None = None


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
# Интеграция со Stockfish
# -----------------------------
# Используем Stockfish из окружения Nix, а не жестко заданный путь
STOCKFISH_PATH = "stockfish"


def _get_stockfish_move(fen: str, time_budget_ms: int) -> str:
    """
    Запускает Stockfish, чтобы получить лучший ход для данной FEN-позиции.
    """
    try:
        process = subprocess.Popen(
            [STOCKFISH_PATH],
            universal_newlines=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=1,  # Line-buffered
        )
    except FileNotFoundError:
        raise AlgPermanentError(
            f"Stockfish not found. Make sure the '{STOCKFISH_PATH}' executable is in your PATH."
        )

    def read_line():
        line = process.stdout.readline().strip()
        return line

    def write_line(command):
        process.stdin.write(command + "\n")
        process.stdin.flush()

    try:
        # UCI Handshake
        write_line("uci")
        while read_line() != "uciok":
            pass
        
        # Set skill level to maximum
        write_line("setoption name Skill Level value 20")

        write_line("isready")
        while read_line() != "readyok":
            pass

        # Send position and command
        write_line(f"position fen {fen}")
        write_line(f"go movetime {time_budget_ms}")

        # Wait for bestmove
        best_move = ""
        while True:
            line = read_line()
            if line.startswith("bestmove"):
                best_move = line.split(" ")[1]
                break
            if not line and process.poll() is not None:
                raise AlgTemporaryError("Stockfish process exited unexpectedly.")

        write_line("quit")
        return best_move

    except Exception as e:
        process.kill()
        stderr_output = process.stderr.read()
        raise AlgTemporaryError(f"Stockfish error: {e}. Stderr: {stderr_output}")

    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


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
    """Вычисляет следующий ход для данной позиции, используя Stockfish."""
    if board is None or not isinstance(board, BoardState):
        raise AlgPermanentError("board must be BoardState")

    # Получаем лучший ход от Stockfish
    uci_move = _get_stockfish_move(board.fen, time_budget_ms)

    # Парсим uci-строку
    if len(uci_move) < 4:
        raise AlgPermanentError(f"Invalid uci move from stockfish: {uci_move}")

    from_cell = uci_move[:2]
    to_cell = uci_move[2:4]

    # Возвращаем решение
    return MoveDecision(
        uci=uci_move,
        from_cell=from_cell,
        to_cell=to_cell,
        reason="stockfish",
        meta={"engine": "stockfish", "time_budget_ms": time_budget_ms},
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
