import os
import asyncio
import httpx
import chess
import subprocess
from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any

# --- Модели данных ---
@dataclass(frozen=True)
class BoardState:
    fen: str

@dataclass(frozen=True)
class MoveDecision:
    uci: str
    from_cell: str
    to_cell: str
    score_cp: Optional[int] = None
    reason: Optional[str] = None

# --- Локальный Stockfish ---
STOCKFISH_PATH = "stockfish"

def _get_stockfish_evaluation_local(fen: str, movetime: int) -> int:
    try:
        process = subprocess.Popen(
            [STOCKFISH_PATH], universal_newlines=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1
        )
    except FileNotFoundError:
        raise RuntimeError(f"Stockfish not found at '{STOCKFISH_PATH}'")

    def read_line(): return process.stdout.readline().strip()
    def write_line(cmd): process.stdin.write(cmd + "\n"); process.stdin.flush()

    try:
        write_line("uci")
        while "uciok" not in read_line(): pass
        write_line("isready")
        while "readyok" not in read_line(): pass
        write_line(f"position fen {fen}")
        write_line(f"go movetime {movetime}")
        score = 0
        while True:
            line = read_line()
            if line.startswith("bestmove"): break
            if "score cp" in line:
                parts = line.split()
                try: score = int(parts[parts.index("cp") + 1]); break
                except (ValueError, IndexError): continue
        write_line("quit")
        return score
    finally:
        if process.poll() is None: process.kill(); process.wait()

# --- Логика распараллеливания ---
async def get_distributed_move(board: BoardState, time_budget_ms: int) -> MoveDecision:
    worker_urls_str = os.getenv("WORKER_DRONES")
    if not worker_urls_str:
        raise ValueError("Переменная окружения WORKER_DRONES не установлена.")
    
    worker_urls = [url.strip() for url in worker_urls_str.split(',')]
    chess_board = chess.Board(board.fen)
    legal_moves = list(chess_board.legal_moves)
    
    if not legal_moves:
        raise ValueError("Нет доступных ходов.")

    # Этап 1: Локальный быстрый поиск
    print("Этап 1: Локальный быстрый поиск для выбора веток...")
    promising_moves = []
    for move in legal_moves:
        temp_board = chess_board.copy()
        temp_board.push(move)
        score = _get_stockfish_evaluation_local(temp_board.fen(), 50)
        promising_moves.append((move, score))

    is_white_turn = chess_board.turn == chess.WHITE
    promising_moves.sort(key=lambda item: item[1], reverse=is_white_turn)
    
    # Этап 2: Распределенный глубокий анализ
    num_workers = len(worker_urls)
    top_moves = [move for move, score in promising_moves[:num_workers]]
    
    if not top_moves:
        raise ValueError("Не удалось определить перспективные ходы.")

    print(f"Этап 2: Отправка {len(top_moves)} лучших ходов на {num_workers} воркеров...")
    movetime_per_worker = max(100, (time_budget_ms - len(legal_moves) * 50) // len(top_moves))
    
    async with httpx.AsyncClient(timeout=movetime_per_worker / 1000 + 15.0) as client:
        tasks = []
        for i, move in enumerate(top_moves):
            temp_board = chess_board.copy()
            temp_board.push(move)
            worker_url = worker_urls[i % num_workers]
            endpoint = f"{worker_url.strip('/')}/evaluate"
            payload = {"fen": temp_board.fen(), "movetime": movetime_per_worker}
            print(f"  - Ход '{move.uci()}' -> на воркер {worker_url}")
            tasks.append(client.post(endpoint, json=payload))
        results = await asyncio.gather(*tasks, return_exceptions=True)

    # Этап 3: Агрегация результатов
    best_move = None
    best_score = -float('inf') if is_white_turn else float('inf')
    print("\nЭтап 3: Агрегация результатов...")

    for i, result in enumerate(results):
        move = top_moves[i]
        if isinstance(result, httpx.Response) and result.status_code == 200:
            score = result.json()["score_cp"]
            print(f"  - Результат для '{move.uci()}': {score} cp")
            if (is_white_turn and score > best_score) or (not is_white_turn and score < best_score):
                best_score, best_move = score, move
        else:
            print(f"  - ⚠️ Ошибка для хода '{move.uci()}': {type(result).__name__}")

    if best_move is None:
        print("⚠️ Воркеры не ответили. Выбираем лучший ход из локальной оценки.")
        best_move = top_moves[0]
        best_score = promising_moves[0][1]

    print(f"\nИтоговое решение: ход {best_move.uci()} с оценкой {best_score} cp.")
    return MoveDecision(
        uci=best_move.uci(),
        from_cell=chess.square_name(best_move.from_square),
        to_cell=chess.square_name(best_move.to_square),
        score_cp=best_score,
        reason="distributed_search"
    )
