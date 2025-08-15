import httpx
import asyncio
import os

# --- Конфигурация ---
# IP-адрес дрона-воркера (drone3).
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
    print(f"Имитируем запуск от имени: DRONE_NAME={os.getenv('DRONE_NAME')}")
    
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
            # Сначала проверим /health эндпоинт
            print("\nШаг 1: Проверка состояния воркера (/health)...")
            health_response = await client.get(f"http://{DRONE3_IP}:{WORKER_PORT}/health")
            health_response.raise_for_status()
            print("✅ Воркер доступен и отвечает.")

            # Затем отправим основной запрос
            print("\nШаг 2: Отправка запроса на оценку (/evaluate)...")
            response = await client.post(EVALUATE_URL, json=payload)
            response.raise_for_status() 

            result = response.json()
            print("\n✅ Успешный ответ от drone3!")
            print(f"  - Оценка позиции (score_cp): {result.get('score_cp')}")

    except httpx.ConnectError as e:
        print(f"\n❌ Ошибка подключения: Не удалось соединиться с drone3 по адресу {EVALUATE_URL}.")
        print("  - Убедитесь, что на drone3 запущен воркер, и что нет фаервола.")
        print(f"  - Детали ошибки: {e}")
    except httpx.HTTPStatusError as e:
        print(f"\n❌ Ошибка HTTP: Дрон-воркер ответил с ошибкой (статус {e.response.status_code}).")
        print(f"  - Тело ответа: {e.response.text}")
    except Exception as e:
        print(f"\n❌ Произошла непредвиденная ошибка: {e}")

if __name__ == "__main__":
    print("--- ИНСТРУКЦИЯ ПО ЗАПУСКУ ТЕСТА ---")
    print("\n1. На дроне-воркере (drone3):")
    print("   - Зайдите в директорию проекта: cd ~/HotDrone2")
    print("   - Активируйте виртуальное окружение: source myvenv/bin/activate")
    print("   - Установите переменные окружения: export DRONE_NAME=drone3")
    print("   - Запустите воркер напрямую. Вы должны увидеть вывод uvicorn:")
    print("     python3 -m drone.run_worker --host 0.0.0.0 --port 3000")
    print("   - Если при запуске возникнут ошибки, пришлите их мне.")
    print("\n2. На главном дроне (drone18):")
    print("   - Убедитесь, что воркер на drone3 запущен и работает.")
    print("   - Запустите этот скрипт:")
    print("     python3 mintik.py\n")
    
    asyncio.run(run_test())
