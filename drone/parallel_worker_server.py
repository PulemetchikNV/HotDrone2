import uvicorn
import subprocess
from fastapi import FastAPI
from pydantic import BaseModel

# --- Модели данных API ---
class EvaluationRequest(BaseModel):
    fen: str
    movetime: int

class EvaluationResponse(BaseModel):
    score_cp: int

import os

# --- Stockfish ---
# Вычисляем абсолютный путь к Stockfish
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    STOCKFISH_PATH = os.path.join(script_dir, 'chess', 'stockfish', 'stockfish-ubuntu-x86-64-avx2')
    STOCKFISH_PATH = os.path.normpath(STOCKFISH_PATH)
except NameError:
    STOCKFISH_PATH = "drone/chess/stockfish/stockfish-ubuntu-x86-64-avx2"


def get_stockfish_evaluation(fen: str, movetime: int) -> int:
    if not os.path.exists(STOCKFISH_PATH):
        raise RuntimeError(f"Stockfish binary not found at the specified path: {STOCKFISH_PATH}")
    if not os.access(STOCKFISH_PATH, os.X_OK):
        raise RuntimeError(f"Stockfish binary is not executable. Please run 'chmod +x {STOCKFISH_PATH}'")

    # ВАЖНОЕ ПРЕДУПРЕЖДЕНИЕ:
    # Имя файла 'stockfish-ubuntu-x86-64-avx2' говорит о том, что он скомпилирован для
    # 64-битных процессоров Intel/AMD. Он НЕ БУДЕТ работать на Raspberry Pi (ARM-архитектура).
    # Если вы получите ошибку 'cannot execute binary file', вам нужно найти или скомпилировать
    # Stockfish для ARM.

    try:
        process = subprocess.Popen(
            [STOCKFISH_PATH], universal_newlines=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1
        )
    except (FileNotFoundError, PermissionError):
        raise RuntimeError(f"Failed to execute Stockfish. Check path and permissions for: {STOCKFISH_PATH}")

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

# --- FastAPI приложение ---
app = FastAPI(title="Parallel Chess Worker")

@app.get("/health", summary="Проверка состояния")
async def health_check():
    return {"status": "ok"}

@app.post("/evaluate", response_model=EvaluationResponse, summary="Оценка позиции")
async def evaluate_position(request: EvaluationRequest):
    score = get_stockfish_evaluation(request.fen, request.movetime)
    return EvaluationResponse(score_cp=score)

# --- Запуск сервера ---
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info")
