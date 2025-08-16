from flask import Flask, send_from_directory, Response
import subprocess
import os

app = Flask(__name__)

LOG_FILE_PATH = os.path.join(os.path.dirname(__file__), '..', 'moves.txt')

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

@app.route('/run', methods=['POST'])
def run_script():
    def generate():
        try:
            script_path = os.path.join(os.path.dirname(__file__), 'alg_llm.py')
            project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            
            # Use Popen for real-time streaming
            process = subprocess.Popen(
                ['python3', '-u', script_path], # -u for unbuffered output
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
