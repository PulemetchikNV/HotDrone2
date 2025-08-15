import os
import asyncio
from drone.alg2 import get_board_state, get_turn, AlgPermanentError, AlgTemporaryError

# --- Конфигурация ---
# Имена дронов и роли должны быть установлены через переменные окружения
# export DRONE_NAME=drone18
# export LEADER_DRONE=drone18
# export WORKER_DRONES="http://192.168.1.71:3000" # IP дрона drone3

def main():
    """
    Основная функция для главного дрона, которая инициирует
    распределенный анализ шахматной позиции.
    """
    print("--- Запуск главного дрона для параллельных вычислений ---")

    drone_name = os.getenv("DRONE_NAME")
    leader_drone = os.getenv("LEADER_DRONE")
    worker_drones = os.getenv("WORKER_DRONES")

    if not all([drone_name, leader_drone, worker_drones]):
        print("Ошибка: Не установлены необходимые переменные окружения.")
        print("Пожалуйста, установите DRONE_NAME, LEADER_DRONE и WORKER_DRONES.")
        return

    if drone_name != leader_drone:
        print(f"Этот скрипт предназначен для запуска только на главном дроне (лидере).")
        print(f"Текущий дрон: {drone_name}, ожидался: {leader_drone}.")
        return

    print(f"Главный дрон: {drone_name}")
    print(f"Воркеры для вычислений: {worker_drones}")

    try:
        # 1. Получаем текущее состояние доски (в реальности это будет от камеры)
        board_state = get_board_state()
        print(f"\nПолучено состояние доски. FEN: {board_state.fen}")

        # 2. Запускаем распределенный поиск лучшего хода
        print("Запрос на вычисление лучшего хода с распараллеливанием...")
        
        # Устанавливаем DRONE_ROLE в 'master', так как get_turn это проверяет
        os.environ["DRONE_ROLE"] = "master"
        
        # Бюджет времени на принятие решения в миллисекундах
        time_budget_ms = 10000

        # Запускаем асинхронную функцию get_turn
        move_decision = get_turn(board_state, time_budget_ms)

        print("\n--- РЕШЕНИЕ ПОЛУЧЕНО ---")
        print(f"Лучший ход (UCI): {move_decision.uci}")
        print(f"Из клетки: {move_decision.from_cell} в клетку: {move_decision.to_cell}")
        print(f"Оценка позиции: {move_decision.score_cp} сантипешек")
        print(f"Причина выбора: {move_decision.reason}")

    except (AlgPermanentError, AlgTemporaryError) as e:
        print(f"\nОшибка во время вычислений: {e}")
    except Exception as e:
        print(f"\nПроизошла непредвиденная ошибка: {e}")

if __name__ == "__main__":
    main()
