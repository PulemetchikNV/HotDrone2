#!/bin/bash

# Скрипт для запуска LLM Chess системы
# Автор: AI Assistant
# Дата: $(date)

echo "=== Запуск LLM Chess системы ==="

# Проверяем что мы в правильной директории
if [ ! -f "drone/chess.py" ]; then
    echo "Ошибка: Запустите скрипт из корневой директории проекта HotDrone2"
    exit 1
fi

# Устанавливаем режим LLM chess
echo "Настройка режима алгоритма..."
sed -i "s/ALG_MODE = '.*'/ALG_MODE = 'llm_chess'/" drone/const.py
echo "✓ ALG_MODE установлен в 'llm_chess'"

# Проверяем что LLM сервер не запущен
echo "Проверка LLM сервера..."
if pgrep -f "python3.*server.py" > /dev/null; then
    echo "Остановка существующего LLM сервера..."
    pkill -f "python3.*server.py"
    sleep 2
fi

# Запускаем LLM сервер
echo "Запуск LLM сервера..."
cd llm_server
python3 server.py &
LLM_SERVER_PID=$!
cd ..

echo "✓ LLM сервер запущен (PID: $LLM_SERVER_PID)"

# Ждем запуска сервера
echo "Ожидание запуска сервера..."
sleep 5

# Проверяем что сервер отвечает
if curl -s http://localhost:8080 > /dev/null; then
    echo "✓ LLM сервер доступен на http://localhost:8080"
else
    echo "⚠ Предупреждение: LLM сервер может быть не готов"
fi

echo ""
echo "=== Система готова к работе ==="
echo ""
echo "Доступные команды:"
echo "  • Веб-интерфейс: http://localhost:8080"
echo "  • Тест алгоритма: python3 test_llm_chess.py"
echo "  • Остановка сервера: pkill -f 'python3.*server.py'"
echo ""
echo "Для интеграции с дронами используйте:"
echo "  from drone.chess import get_turn_final"
echo "  move = get_turn_final(board_state)"
echo ""
echo "LLM сервер будет работать в фоне."
echo "Для остановки нажмите Ctrl+C или выполните: pkill -f 'python3.*server.py'"
