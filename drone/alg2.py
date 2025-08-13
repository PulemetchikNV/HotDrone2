from __future__ import annotations

import os
import asyncio
import httpx
import chess
import subprocess
import time
from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any, List

# -----------------------------
# 1. DATA CLASSES AND EXCEPTIONS (from alg.py)
# -----------------------------
class AlgTemporaryError(Exception):
    """Временная ошибка (плохой кадр/низкая уверенность). Можно повторить позже."""

class AlgPermanentError(Exception):
    """Постоянная ошибка (некорректная позиция или входные данные)."""

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
# 2. STOCKFISH INTEGRATION
# -----------------------------
STOCKFISH_PATH = "stockfish"

def _get_stockfish_evaluation_local(fen: str, movetime: int) -> int:
    """
    Локальная версия для мастера. Не должна конфликтовать с воркерами.
    """
    # В реальной системе здесь могла бы быть другая, более легкая логика.
    # Для теста мы просто используем тот же движок, но это показывает разделение.
    return _get_stockfish_evaluation(fen, movetime)

def _get_stockfish_evaluation(fen: str, movetime: int) -> int:
    """
    Запускает Stockfish для оценки позиции и возвращает счет в сантипешках.
    """
    try:
        process = subprocess.Popen(
            [STOCKFISH_PATH],
            universal_newlines=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=1,
        )
    except FileNotFoundError:
        raise AlgPermanentError(f"Stockfish not found at '{STOCKFISH_PATH}'")

    def read_line():
        return process.stdout.readline().strip()

    def write_line(command):
        process.stdin.write(command + "\n")
        process.stdin.flush()

    try:
        write_line("uci")
        while "uciok" not in read_line(): pass
        
        write_line("setoption name Skill Level value 20")
        
        write_line("isready")
        while "readyok" not in read_line(): pass

        write_line(f"position fen {fen}")
        
        # For very short movetimes, 'go movetime' can be unreliable and hang.
        # In such cases, we switch to searching a fixed number of nodes,
        # which is faster and guarantees a result.
        if movetime < 100:
            write_line("go nodes 5000")
        else:
            write_line(f"go movetime {movetime}")

        score = 0
        while True:
            line = read_line()
            if line.startswith("bestmove"):
                break
            if "score cp" in line:
                parts = line.split(" ")
                try:
                    cp_index = parts.index("cp")
                    score = int(parts[cp_index + 1])
                except (ValueError, IndexError):
                    continue
        
        write_line("quit")
        return score

    finally:
        if process.poll() is None:
            process.kill()
            process.wait()

# -----------------------------
# 3. MASTER LOGIC
# -----------------------------
# 3. MASTER LOGIC
# -----------------------------
async def get_turn_distributed(board: BoardState, time_budget_ms: int) -> MoveDecision:
    """
    Вычисляет лучший ход, используя двухэтапный распределенный поиск.
    Этап 1: Локальный быстрый поиск для отсеивания плохих ходов.
    Этап 2: Глубокий анализ нескольких лучших ходов на удаленных воркерах.
    """
    # --- НАСТРОЙКА ---
    worker_urls_str = os.getenv("WORKER_DRONES")
    if not worker_urls_str:
        raise AlgPermanentError("Переменная окружения WORKER_DRONES не установлена.")
    
    worker_urls = [url.strip() for url in worker_urls_str.split(',')]
    chess_board = chess.Board(board.fen)
    legal_moves = list(chess_board.legal_moves)
    
    if not legal_moves:
        raise AlgPermanentError("Нет доступных ходов из данной позиции.")

    # --- ЭТАП 1: ЛОКАЛЬНЫЙ ШИРОКИЙ, НО НЕГЛУБОКИЙ ПОИСК ---
    print("Этап 1: Локальный поиск перспективных ходов для определения 'ветвей' дерева...")
    promising_moves_with_scores = []
    local_eval_time_ms = 50  # Очень малое время на каждый ход
    
    for move in legal_moves:
        temp_board = chess_board.copy()
        temp_board.push(move)
        # Используем локальный Stockfish для быстрой оценки
        score = _get_stockfish_evaluation_local(temp_board.fen(), local_eval_time_ms)
        promising_moves_with_scores.append((move, score))

    # Сортируем ходы, чтобы найти самые перспективные
    is_white_turn = chess_board.turn == chess.WHITE
    promising_moves_with_scores.sort(key=lambda item: item[1], reverse=is_white_turn)
    
    # --- ВЫБОР ВЕТВЕЙ ДЛЯ ГЛУБОКОГО АНАЛИЗА ---
    # Анализируем столько ходов, сколько у нас есть воркеров, чтобы максимизировать параллелизм.
    # Ограничиваем сверху, чтобы не тратить время на заведомо плохие ходы.
    num_workers = len(worker_urls)
    moves_for_deep_analysis = min(max(num_workers, 2), 8) 
    top_moves_to_evaluate = [move for move, score in promising_moves_with_scores[:moves_for_deep_analysis]]
    
    if not top_moves_to_evaluate:
        raise AlgTemporaryError("Не удалось определить перспективные ходы локально.")

    print(f"Этап 2: Распределение {len(top_moves_to_evaluate)} лучших ходов (ветвей) для глубокого анализа на {num_workers} воркерах...")

    # --- ЭТАП 2: РАСПРЕДЕЛЕННЫЙ ГЛУБОКИЙ АНАЛИЗ ---
    # Оставшееся время распределяем на глубокий анализ
    time_for_workers_ms = time_budget_ms - (len(legal_moves) * local_eval_time_ms)
    movetime_per_worker = max(100, time_for_workers_ms // len(top_moves_to_evaluate))
    
    # Таймаут HTTP запроса должен быть чуть больше времени работы воркера
    request_timeout_seconds = (movetime_per_worker / 1000) + 15.0

    async with httpx.AsyncClient(timeout=request_timeout_seconds) as client:
        tasks = []
        for i, move in enumerate(top_moves_to_evaluate):
            # Для каждой "ветви" создаем новую позицию
            temp_board = chess_board.copy()
            temp_board.push(move)
            
            # Циклически распределяем задачи по воркерам
            worker_url = worker_urls[i % num_workers].strip('/')
            endpoint = f"{worker_url}/evaluate"
            
            # Формируем тело запроса
            payload = {"fen": temp_board.fen(), "movetime": movetime_per_worker}
            
            print(f"  - Ветвь '{move.uci()}' -> на воркер {worker_url}")
            tasks.append(client.post(endpoint, json=payload))

        # Асинхронно выполняем все запросы
        results = await asyncio.gather(*tasks, return_exceptions=True)

    # --- ЭТАП 3: АГРЕГАЦИЯ РЕЗУЛЬТАТОВ И ПРИНЯТИЕ РЕШЕНИЯ ---
    best_move = None
    best_score = -float('inf') if is_white_turn else float('inf')
    print("\nЭтап 3: Агрегация результатов от воркеров...")

    for i, result in enumerate(results):
        move = top_moves_to_evaluate[i]
        if isinstance(result, httpx.Response) and result.status_code == 200:
            score = result.json()["score_cp"]
            print(f"  - Результат для ветви '{move.uci()}': {score} cp")
            if is_white_turn:
                if score > best_score:
                    best_score, best_move = score, move
            else:
                if score < best_score:
                    best_score, best_move = score, move
        else:
            # Обработка ошибок сети или ответа сервера
            error_type = type(result).__name__
            print(f"  - ⚠️  Ошибка для ветви '{move.uci()}': {error_type} - {result}")

    # Если все воркеры не ответили, выбираем лучший ход из локальной оценки
    if best_move is None:
        print("⚠️  Все воркеры не ответили. Возвращаемся к результатам локальной оценки.")
        best_move = top_moves_to_evaluate[0]
        best_score = promising_moves_with_scores[0][1]

    print(f"\nИтоговое решение: ход {best_move.uci()} с оценкой {best_score} cp.")
    return MoveDecision(
        uci=best_move.uci(),
        from_cell=chess.square_name(best_move.from_square),
        to_cell=chess.square_name(best_move.to_square),
        score_cp=best_score,
        reason="distributed_2_stage"
    )

# -----------------------------
# 6. PUBLIC API FUNCTIONS
# -----------------------------
def get_board_state() -> BoardState:
    # This function's logic remains the same as in alg.py
    # For now, returning a default start position.
    return BoardState(
        fen="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
        turn="w",
        move_number=1,
        timestamp=time.time(),
    )

def get_turn_centralized(board: BoardState, time_budget_ms: int) -> MoveDecision:
    """
    Вычисляет лучший ход, используя только локальный движок Stockfish.
    """
    print("Централизованный поиск лучшего хода...")
    chess_board = chess.Board(board.fen)
    
    if not chess_board.legal_moves:
        raise AlgPermanentError("No legal moves available from this position.")

    # В этом режиме мы просто даем Stockfish весь бюджет времени
    # Это эквивалентно одному большому "глубокому" анализу
    movetime = time_budget_ms - 500 # Небольшой запас
    
    try:
        process = subprocess.Popen(
            [STOCKFISH_PATH],
            universal_newlines=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=1,
        )
    except FileNotFoundError:
        raise AlgPermanentError(f"Stockfish not found at '{STOCKFISH_PATH}'")

    def read_line():
        return process.stdout.readline().strip()

    def write_line(command):
        process.stdin.write(command + "\n")
        process.stdin.flush()

    try:
        write_line("uci")
        while "uciok" not in read_line(): pass
        
        write_line("setoption name Skill Level value 20")
        
        write_line("isready")
        while "readyok" not in read_line(): pass

        write_line(f"position fen {board.fen}")
        write_line(f"go movetime {movetime}")

        best_move_uci = None
        score_cp = 0
        
        while True:
            line = read_line()
            if line.startswith("bestmove"):
                best_move_uci = line.split(" ")[1]
                break
            if "score cp" in line:
                parts = line.split(" ")
                try:
                    cp_index = parts.index("cp")
                    score_cp = int(parts[cp_index + 1])
                except (ValueError, IndexError):
                    continue
        
        write_line("quit")
        
        if best_move_uci is None:
            raise AlgTemporaryError("Stockfish did not return a best move in time.")
            
        move = chess.Move.from_uci(best_move_uci)
        return MoveDecision(
            uci=move.uci(),
            from_cell=chess.square_name(move.from_square),
            to_cell=chess.square_name(move.to_square),
            score_cp=score_cp,
            reason="centralized_stockfish"
        )

    finally:
        if process.poll() is None:
            process.kill()
            process.wait()

def get_turn(board: BoardState, time_budget_ms: int = 5000) -> MoveDecision:
    """
    Выбирает режим работы (распределенный или централизованный) на основе
    переменной окружения DRONE_MODE.
    """
    mode = os.getenv("DRONE_MODE", "distributed") # По умолчанию распределенный
    
    if os.getenv("DRONE_ROLE") != "master":
        raise RuntimeError("get_turn can only be called on the master drone.")
    
    if mode == "centralized":
        return get_turn_centralized(board, time_budget_ms)
    else:
        return asyncio.run(get_turn_distributed(board, time_budget_ms))

def update_after_execution(prev: BoardState, move: MoveDecision, success: bool) -> BoardState:
    # Logic remains the same
    if not success:
        return prev
    
    board = chess.Board(prev.fen)
    board.push(chess.Move.from_uci(move.uci))
    
    return BoardState(
        fen=board.fen(),
        turn="b" if prev.turn == "w" else "w",
        move_number=prev.move_number + 1,
        timestamp=time.time(),
        meta=prev.meta,
    )

# -----------------------------
# 7. MAIN EXECUTION BLOCK
# -----------------------------
# The main execution block has been moved to drone/run_worker.py for a cleaner separation
# of concerns. This file is now a pure library.
if __name__ == "__main__":
    print("This file is a library and is not meant to be run directly.")
    print("To start a worker, run: python -m drone.run_worker --port <port>")
    print("To use the master, import and call get_turn() from your main script.")
