from flask import Flask, send_from_directory, Response, jsonify
from flask_cors import CORS
import subprocess
import os
import json
import time
from datetime import datetime

app = Flask(__name__)
CORS(app)  # Разрешаем CORS для всех маршрутов

LOG_FILE_PATH = os.path.join(os.path.dirname(__file__), '..', 'moves.txt')
MOVES_LOG_PATH = os.path.join(os.path.dirname(__file__), 'moves_log.json')

def log_move(move_data):
    """Логирует ход в JSON файл для polling."""
    try:
        # Читаем существующие ходы
        moves = []
        if os.path.exists(MOVES_LOG_PATH):
            with open(MOVES_LOG_PATH, 'r', encoding='utf-8') as f:
                try:
                    moves = json.load(f)
                except json.JSONDecodeError:
                    moves = []
        
        # Добавляем новый ход
        move_entry = {
            'timestamp': datetime.now().isoformat(),
            'move': move_data.get('move', ''),
            'fen': move_data.get('fen', ''),
            'from_cell': move_data.get('from_cell', ''),
            'to_cell': move_data.get('to_cell', ''),
            'reason': move_data.get('reason', ''),
            'engine': move_data.get('engine', 'unknown')
        }
        
        moves.append(move_entry)
        
        # Оставляем только последние 50 ходов
        moves = moves[-50:]
        
        # Сохраняем в файл
        with open(MOVES_LOG_PATH, 'w', encoding='utf-8') as f:
            json.dump(moves, f, ensure_ascii=False, indent=2)
            
    except Exception as e:
        print(f"Error logging move: {e}")

def get_recent_moves(since_timestamp=None):
    """Возвращает последние ходы, опционально с указанного времени."""
    try:
        if not os.path.exists(MOVES_LOG_PATH):
            return []
            
        with open(MOVES_LOG_PATH, 'r', encoding='utf-8') as f:
            moves = json.load(f)
            
        if since_timestamp:
            # Фильтруем ходы после указанного времени
            filtered_moves = []
            for move in moves:
                if move.get('timestamp', '') > since_timestamp:
                    filtered_moves.append(move)
            return filtered_moves
        
        return moves[-10:]  # Возвращаем последние 10 ходов
        
    except Exception as e:
        print(f"Error reading moves: {e}")
        return []

@app.route('/')
def index():
    # Redirect to the new interface by default
    return send_from_directory('.', 'index_v2.html')

@app.route('/v2')
def index_v2():
    return send_from_directory('.', 'index_v2.html')

@app.route('/<path:path>')
def static_files(path):
    return send_from_directory('.', path)

@app.route('/log', methods=['GET'])
def get_log():
    try:
        with open(LOG_FILE_PATH, 'r', encoding='utf-8') as f:
            return Response(f.read(), mimetype='text/plain')
    except FileNotFoundError:
        return Response("Лог-файл 'moves.txt' еще не создан.", mimetype='text/plain', status=404)
    except Exception as e:
        return Response(f"Ошибка чтения файла: {e}", mimetype='text/plain', status=500)

@app.route('/poll', methods=['GET'])
def poll_moves():
    """Endpoint для polling новых ходов с телефона."""
    from flask import request
    
    since = request.args.get('since', None)
    moves = get_recent_moves(since)
    
    response = jsonify({
        'status': 'success',
        'moves': moves,
        'timestamp': datetime.now().isoformat(),
        'count': len(moves)
    })
    
    # Добавляем CORS заголовки вручную для совместимости
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    
    return response

@app.route('/log_move', methods=['POST'])
def log_move_endpoint():
    """Endpoint для логирования ходов от дронов."""
    from flask import request
    
    try:
        move_data = request.get_json()
        if not move_data:
            response = jsonify({'status': 'error', 'message': 'No JSON data provided'})
            response.status_code = 400
        else:
            log_move(move_data)
            response = jsonify({'status': 'success', 'message': 'Move logged successfully'})
            
    except Exception as e:
        response = jsonify({'status': 'error', 'message': str(e)})
        response.status_code = 500
    
    # Добавляем CORS заголовки
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    
    return response

@app.route('/poll', methods=['OPTIONS'])
@app.route('/log_move', methods=['OPTIONS'])
def handle_options():
    """Обрабатываем preflight OPTIONS запросы для CORS."""
    response = Response()
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    response.headers.add('Access-Control-Max-Age', '86400')
    return response

@app.route('/run', methods=['POST'])
def run_script():
    def generate():
        try:
            script_path = os.path.join(os.path.dirname(__file__), 'alg_llm.py')
            project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            
            # Получаем FEN из запроса, если есть
            from flask import request
            fen = None
            try:
                if request.is_json:
                    fen = request.json.get('fen')
            except:
                pass  # Игнорируем ошибки контекста
            
            # Формируем команду
            cmd = ['python3', '-u', script_path]
            if fen:
                cmd.append(fen)
            
            # Use Popen for real-time streaming
            process = subprocess.Popen(
                cmd, # -u for unbuffered output
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding='utf-8',
                cwd=project_root
            )

            # Stream the output line by line
            for line in iter(process.stdout.readline, ''):
                yield f"data: {line}\n\n"
            
            process.stdout.close()
            return_code = process.wait()
            if return_code:
                yield f"data: SCRIPT_ERROR: Process returned exit code {return_code}\n\n"

        except Exception as e:
            yield f"data: SERVER_ERROR: {str(e)}\n\n"

    # Use a streaming response
    return Response(generate(), mimetype='text/event-stream')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)
