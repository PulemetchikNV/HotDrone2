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
DIALOG_LOG_PATH = os.path.join(os.path.dirname(__file__), 'dialog_log.json')

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

def log_dialog(dialog_data):
    """Логирует диалог дронов в JSON файл для polling."""
    try:
        # Читаем существующие диалоги
        dialogs = []
        if os.path.exists(DIALOG_LOG_PATH):
            with open(DIALOG_LOG_PATH, 'r', encoding='utf-8') as f:
                try:
                    dialogs = json.load(f)
                except json.JSONDecodeError:
                    dialogs = []
        
        # Добавляем новый диалог
        dialog_entry = {
            'timestamp': datetime.now().isoformat(),
            'session_id': dialog_data.get('session_id', 'unknown'),
            'messages': dialog_data.get('messages', []),
            'final_move': dialog_data.get('final_move', ''),
            'fen': dialog_data.get('fen', '')
        }
        
        dialogs.append(dialog_entry)
        
        # Оставляем только последние 20 диалогов
        dialogs = dialogs[-20:]
        
        # Сохраняем в файл
        with open(DIALOG_LOG_PATH, 'w', encoding='utf-8') as f:
            json.dump(dialogs, f, ensure_ascii=False, indent=2)
            
    except Exception as e:
        print(f"Error logging dialog: {e}")

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

def get_recent_dialogs(since_timestamp=None):
    """Возвращает последние диалоги, опционально с указанного времени."""
    try:
        if not os.path.exists(DIALOG_LOG_PATH):
            return []
            
        with open(DIALOG_LOG_PATH, 'r', encoding='utf-8') as f:
            dialogs = json.load(f)
            
        if since_timestamp:
            # Фильтруем диалоги после указанного времени
            filtered_dialogs = []
            for dialog in dialogs:
                if dialog.get('timestamp', '') > since_timestamp:
                    filtered_dialogs.append(dialog)
            return filtered_dialogs
        
        return dialogs[-5:]  # Возвращаем последние 5 диалогов
        
    except Exception as e:
        print(f"Error reading dialogs: {e}")
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
    """Endpoint для polling новых диалогов и ходов с телефона."""
    from flask import request
    
    since = request.args.get('since', None)
    dialogs = get_recent_dialogs(since)
    moves = get_recent_moves(since)
    
    response = jsonify({
        'status': 'success',
        'dialogs': dialogs,
        'moves': moves,
        'timestamp': datetime.now().isoformat(),
        'dialogs_count': len(dialogs),
        'moves_count': len(moves)
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

@app.route('/log_dialog', methods=['POST'])
def log_dialog_endpoint():
    """Endpoint для логирования диалогов дронов."""
    from flask import request
    
    try:
        dialog_data = request.get_json()
        if not dialog_data:
            response = jsonify({'status': 'error', 'message': 'No JSON data provided'})
            response.status_code = 400
        else:
            log_dialog(dialog_data)
            response = jsonify({'status': 'success', 'message': 'Dialog logged successfully'})
            
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
@app.route('/log_dialog', methods=['OPTIONS'])
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

            # Собираем диалог для логирования
            dialog_messages = []
            final_move = None
            session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Stream the output line by line
            for line in iter(process.stdout.readline, ''):
                line_clean = line.strip()
                yield f"data: {line}\n\n"
                
                # Парсим и сохраняем диалог
                if line_clean:
                    if line_clean.startswith('FIGURE:'):
                        parts = line_clean.split(':', 3)
                        if len(parts) >= 4:
                            dialog_messages.append({
                                'type': 'figure',
                                'figure': parts[1],
                                'role': parts[2], 
                                'content': parts[3]
                            })
                    elif line_clean.startswith('MASTER:'):
                        content = line_clean[7:] if len(line_clean) > 7 else ''
                        dialog_messages.append({
                            'type': 'master',
                            'content': content
                        })
                    elif line_clean.startswith('FINAL_MOVE:'):
                        final_move = line_clean[11:].strip()
                        dialog_messages.append({
                            'type': 'final_move',
                            'content': final_move
                        })
                    elif line_clean.startswith('SYSTEM:'):
                        content = line_clean[7:] if len(line_clean) > 7 else ''
                        dialog_messages.append({
                            'type': 'system',
                            'content': content
                        })
            
            process.stdout.close()
            return_code = process.wait()
            
            # Логируем диалог, если есть сообщения
            if dialog_messages:
                try:
                    log_dialog({
                        'session_id': session_id,
                        'messages': dialog_messages,
                        'final_move': final_move or '',
                        'fen': fen or ''
                    })
                except Exception as log_e:
                    print(f"Error logging dialog: {log_e}")
            
            if return_code:
                yield f"data: SCRIPT_ERROR: Process returned exit code {return_code}\n\n"

        except Exception as e:
            yield f"data: SERVER_ERROR: {str(e)}\n\n"

    # Use a streaming response
    return Response(generate(), mimetype='text/event-stream')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)
