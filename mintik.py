import httpx
import asyncio
import os

# --- Конфигурация ---
# IP-адрес дрона-воркера (drone3). В реальной системе это может браться из файла или переменных окружения.
DRONE3_IP = "192.168.1.71" 
WORKER_PORT = 3000

# URL эндпоинта для оценки позиции
EVALUATE_URL = f"http://{DRONE3_IP}:{WORKER_PORT}/evaluate"

# Тестовая FEN-позиция (начальная позиция в шахматах)
TEST_FEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
# Время на оценку в миллисекундах
MOVETIME_MS = 2000

async def run_test():
    """
    Асинхронная функция для отправки тестового запроса на дрон-воркер.
    """
    print("--- Запуск теста FastAPI взаимодействия ---")
    print(f"Главный дрон (этот скрипт) будет общаться с дроном-воркером (drone3) по адресу: {EVALUATE_URL}")

    # Устанавливаем переменные окружения, как будто мы - главный дрон
    os.environ["DRONE_NAME"] = "drone18"
    os.environ["LEADER_DRONE"] = "drone18"
    print(f"Имитируем запуск от имени: DRONE_NAME={os.getenv('DRONE_NAME')}, LEADER_DRONE={os.getenv('LEADER_DRONE')}")
    
    # Тело запроса
    payload = {
        "fen": TEST_FEN,
        "movetime": MOVETIME_MS
    }

    print(f"\nОтправка запроса на оценку позиции:")
    print(f"  - FEN: {payload['fen']}")
    print(f"  - Movetime: {payload['movetime']} ms")

    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(EVALUATE_URL, json=payload)

            # Проверяем статус ответа
            response.raise_for_status() 

            # Выводим результат
            result = response.json()
            print("\n✅ Успешный ответ от drone3!")
            print(f"  - Оценка позиции (score_cp): {result.get('score_cp')}")

    except httpx.ConnectError as e:
        print(f"\n❌ Ошибка подключения: Не удалось соединиться с drone3 по адресу {EVALUATE_URL}.")
        print("  - Убедитесь, что на drone3 запущен воркер (./scripts/start_workers.sh).")
        print(f"  - Детали ошибки: {e}")
    except httpx.HTTPStatusError as e:
        print(f"\n❌ Ошибка HTTP: Дрон-воркер ответил с ошибкой (статус {e.response.status_code}).")
        print(f"  - Тело ответа: {e.response.text}")
    except Exception as e:
        print(f"\n❌ Произошла непредвиденная ошибка: {e}")

if __name__ == "__main__":
    print("Для запуска этого теста, сначала убедитесь, что на дроне drone3 запущен FastAPI воркер.")
    print("Вы можете запустить его командой: ./scripts/start_workers.sh")
    print("После этого, на главном дроне (drone18) запустите этот скрипт: python3 mintik.py\n")
    
    asyncio.run(run_test())
