import os
import asyncio
from drone.parallel_logic import get_distributed_move, BoardState

def main():
    """
    Основная функция для главного дрона, которая инициирует
    распределенный анализ шахматной позиции.
    """
    print("--- Запуск главного дрона для параллельных вычислений ---")

    # Проверяем необходимые переменные окружения
    if "WORKER_DRONES" not in os.environ:
        print("Ошибка: Установите переменную окружения WORKER_DRONES.")
        print('Пример: export WORKER_DRONES="http://192.168.1.71:3000"')
        return

    print(f"Главный дрон: {os.getenv('DRONE_NAME', 'drone18')}")
    print(f"Воркеры для вычислений: {os.getenv('WORKER_DRONES')}")

    try:
        # Начальная позиция для теста
        initial_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        board_state = BoardState(fen=initial_fen)
        print(f"\nНачальная позиция FEN: {board_state.fen}")

        # Бюджет времени на принятие решения в миллисекундах
        time_budget_ms = 10000

        # Запускаем распределенный поиск
        move_decision = asyncio.run(get_distributed_move(board_state, time_budget_ms))

        print("\n--- РЕШЕНИЕ ПОЛУЧЕНО ---")
        print(f"Лучший ход (UCI): {move_decision.uci}")
        print(f"Оценка позиции: {move_decision.score_cp} сантипешек")

    except Exception as e:
        print(f"\nПроизошла ошибка: {e}")

if __name__ == "__main__":
    main()
