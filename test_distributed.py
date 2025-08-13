import os
import sys
import time
import httpx
import argparse

def run_test(mode: str):
    """
    Запускает и координирует тест с реальными дронами в сети.
    """
    print("=" * 50)
    print("Запуск теста с РЕАЛЬНЫМИ дронами")
    print("=" * 50)

    os.environ["DRONE_MODE"] = mode
    
    if mode == "distributed":
        print("=" * 50)
        print("Запуск теста в РАСПРЕДЕЛЕННОМ режиме")
        print("=" * 50)
        # В распределенном режиме нам нужны URL воркеров
        worker_addresses = [
            "http://drone15.local:8080/",
            "http://drone18.local:8080/",
        ]
        os.environ["WORKER_DRONES"] = ",".join(worker_addresses)
    else:
        print("=" * 50)
        print("Запуск теста в ЦЕНТРАЛИЗОВАННОМ режиме")
        print("=" * 50)
        # В централизованном режиме воркеры не нужны
        worker_addresses = []

    try:
        if mode == "distributed":
            # --- 1. Health Check для всех дронов ---
            print("\nОжидание готовности всех дронов...")
            max_wait_time = 20
            start_time = time.time()
            
            for address in worker_addresses:
                health_url = f"{address.strip('/')}/health"
                worker_ready = False
                print(f"  - Проверка {health_url}...", end="")
                while time.time() - start_time < max_wait_time:
                    try:
                        response = httpx.get(health_url, timeout=5)
                        if response.status_code == 200:
                            print(" готово!")
                            worker_ready = True
                            break
                        else:
                            print(f" (статус {response.status_code})", end="")
                    except httpx.RequestError as e:
                        print(f" (ошибка: {type(e).__name__})", end="")
                    time.sleep(1)
                    print(".", end="", flush=True)
                
                if not worker_ready:
                    raise RuntimeError(f"Дрон {address} не ответил вовремя.")

        # --- 2. Запуск логики ---
        print("\nНастройка и запуск основной логики...")
        os.environ["DRONE_ROLE"] = "master" # В обоих режимах кто-то должен быть мастером
        sys.path.insert(0, os.getcwd())
        
        # Импортируем alg2 ПОСЛЕ установки переменных окружения
        from drone.alg2 import get_turn, get_board_state, AlgPermanentError

        initial_board_state = get_board_state()
        print(f"Начальная позиция (FEN): {initial_board_state.fen}")
        
        print("\nМастер вычисляет ход...")
        decision = get_turn(initial_board_state, time_budget_ms=20000) # Увеличим бюджет для реальных дронов
        
        print("\n--- РЕЗУЛЬТАТ ---")
        print(f"Лучший ход по мнению системы: {decision.uci}")
        print(f"Оценка позиции: {decision.score_cp} сантипешек")
        print(f"Причина: {decision.reason}")
        print("=" * 50)

    except Exception as e:
        print(f"\nОШИБКА во время теста: {e}")
    finally:
        print("\nТест завершен.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Запуск теста шахматного движка.")
    parser.add_argument(
        "--mode", 
        type=str, 
        choices=["distributed", "centralized"], 
        default="distributed",
        help="Режим работы: 'distributed' для распределенных вычислений, 'centralized' для локальных."
    )
    args = parser.parse_args()
    
    run_test(mode=args.mode)
