from __future__ import annotations

import subprocess
import os
import logging
from fastapi import FastAPI
from pydantic import BaseModel
from drone.log_handler import UDPLogHandler

# -----------------------------
# 1. LOGGING SETUP
# -----------------------------
# Получаем имя дрона из переменной окружения, чтобы идентифицировать логи
DRONE_NAME = os.getenv("DRONE_NAME", "unknown_worker")
LOG_SERVER_IP = os.getenv("LOG_SERVER_IP", "192.168.2.124") # IP машины, где запущен log_server

# Настраиваем логгер
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
# Убираем стандартные обработчики, чтобы избежать вывода в консоль воркера
if logger.hasHandlers():
    logger.handlers.clear()
# Добавляем наш UDP обработчик
logger.addHandler(UDPLogHandler(server_ip=LOG_SERVER_IP, drone_name=DRONE_NAME))

# -----------------------------
# 2. PYDANTIC MODELS FOR API
# -----------------------------
class EvaluationRequest(BaseModel):
    """Запрос на оценку позиции."""
    fen: str
    movetime: int

class EvaluationResponse(BaseModel):
    """Ответ с оценкой позиции."""
    score_cp: int

# -----------------------------
# 2. STOCKFISH INTEGRATION
# -----------------------------
STOCKFISH_PATH = "stockfish"

class StockfishError(Exception):
    """Общая ошибка при работе со Stockfish."""

def get_stockfish_evaluation(fen: str, movetime: int) -> int:
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
        raise StockfishError(f"Stockfish not found at '{STOCKFISH_PATH}'")

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
        
        # Для коротких вычислений используем поиск по узлам, для длинных - по времени
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
# 4. FASTAPI APPLICATION
# -----------------------------
app = FastAPI(
    title="Chess Drone Worker API",
    description="API для децентрализованного анализа шахматных позиций.",
    version="1.0.0"
)

@app.get("/health", summary="Проверка состояния")
async def health_check():
    """
    Простой эндпоинт для проверки, что воркер запущен и отвечает.
    """
    logger.info("Health check requested.")
    return {"status": "ok"}

@app.post("/evaluate", response_model=EvaluationResponse, summary="Оценка позиции")
async def evaluate_position(request: EvaluationRequest):
    """
    Принимает шахматную позицию (FEN) и время на обдумывание,
    возвращает оценку позиции в сантипешках от движка Stockfish.
    
    Этот роут является основной точкой для распределенного решения.
    """
    logger.info(f"Received evaluation request for FEN: {request.fen} with movetime: {request.movetime}ms")
    try:
        score = get_stockfish_evaluation(request.fen, request.movetime)
        logger.info(f"Evaluation successful. Score: {score} cp")
        return EvaluationResponse(score_cp=score)
    except StockfishError as e:
        logger.error(f"Stockfish error during evaluation: {e}")
        # В реальном приложении можно было бы вернуть HTTP 500
        raise
    except Exception as e:
        logger.critical(f"Unexpected error during evaluation: {e}")
        raise

@app.post("/submit_result", summary="Отправка решенной ветки")
async def submit_result(request: EvaluationResponse):
    """
    Принимает результат анализа ветки от воркера.
    В реальной системе здесь была бы логика агрегации результатов.
    """
    print(f"Получен результат от воркера: {request.score_cp}")
    return {"status": "result received"}

# Чтобы можно было запустить этот файл напрямую для отладки:
# uvicorn drone.fast_api_priem:app --reload
