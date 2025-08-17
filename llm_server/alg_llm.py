import os
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any
import json
import subprocess
import re

# Add the parent directory to the Python path
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from drone.const import DRONES_CONFIG, LEADER_DRONE, DRONE_LIST

# --- Data Classes ---
@dataclass
class BoardState:
    fen: str
    turn: str = "w"
    move_number: int = 1
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            import time
            object.__setattr__(self, 'timestamp', time.time())

# --- Constants ---
FILES = "abcdefgh"
RANKS = "12345678"
API_KEY = os.environ.get("GOOGLE_API_KEY", "AIzaSyAy817y7DirgpWnMPS6t5ps-6Ui2pFAMys")
LLM_URL = "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent"

# --- Translations and Personas (Expanded) ---
CHARACTER_MAP = {
    "king": "Очень осторожный и стратегический игрок. Предпочитает защитные ходы и контроль центра доски.",
    "queen": "Агрессивный игрок. Всегда ищет возможности для атаки на короля противника.",
    "bishop": "Расчетливый игрок, предпочитающий дальнобойные атаки и контроль диагоналей.",
    "knight": "Хитрый и непредсказуемый игрок, использующий необычные маневры.",
    "rook": "Прямолинейный и сильный игрок, нацеленный на контроль открытых линий.",
    "pawn": "Надежный командный игрок, сосредоточенный на построении прочной структуры.",
}
RUSSIAN_ROLES = {
    "king": "Король", "queen": "Королева", "bishop": "Слон", "knight": "Конь", "rook": "Ладья", "pawn": "Пешка",
    "master": "Мастер", "slave": "Ведомый"
}
UNICODE_PIECES = {
    "king": "♔", "queen": "♕", "bishop": "♗", "knight": "♘", "rook": "♖", "pawn": "♙",
}

# --- Helper Functions ---
def call_llm(prompt):
    """Calls the LLM with a given prompt using curl and a SOCKS5 proxy."""
    proxy = "socks5h://danterobot:zxckaliparrot@83.151.2.50:8543"
    command = [
        'curl', '--silent', '--max-time', '60',
        '--proxy', proxy,
        LLM_URL,
        '-H', 'Content-Type: application/json',
        '-H', f'X-goog-api-key: {API_KEY}',
        '-X', 'POST',
        '-d', json.dumps({"contents": [{"parts": [{"text": prompt}]}]})
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True, encoding='utf-8')
        response_json = json.loads(result.stdout)
        # Basic validation of the response structure
        if 'candidates' in response_json and len(response_json['candidates']) > 0:
            content = response_json['candidates'][0].get('content', {})
            if 'parts' in content and len(content['parts']) > 0:
                return content['parts'][0].get('text', '')
        print(f"LLM Error: Unexpected response format: {result.stdout}", flush=True)
        return None
    except subprocess.CalledProcessError as e:
        print(f"LLM Error: curl command failed with exit code {e.returncode}. Stderr: {e.stderr}", flush=True)
        return None
    except json.JSONDecodeError:
        print(f"LLM Error: Failed to decode JSON from response: {result.stdout}", flush=True)
        return None
    except Exception as e:
        print(f"LLM Error: An unexpected error occurred: {e}", flush=True)
        return None

# --- Agent Class ---
class Agent:
    def __init__(self, drone_info):
        self.id = drone_info['id']
        self.role = drone_info['role']
        self.character = drone_info['character']
        self.figure = drone_info['figure']

    def think(self, prompt):
        full_prompt = f"Вы — шахматный ассистент. Ваш характер: '{self.character}'.\n\n{prompt}"
        return call_llm(full_prompt)

# --- Main Logic (Rewritten with Simulated Discussion) ---
def get_turn(board: BoardState):
    """Simulates a multi-agent discussion in a single LLM call to determine the best move."""
    print("SYSTEM: --- Запуск обсуждения дронами ---", flush=True)
    time.sleep(1)

    # --- Agent Setup ---
    all_drone_ids = [d.strip() for d in DRONE_LIST.split(',') if d.strip()]
    default_figures = ['rook', 'knight', 'bishop', 'pawn', 'pawn', 'pawn']
    agents = []
    for drone_id in all_drone_ids:
        config = DRONES_CONFIG.get(drone_id, {})
        role_in_llm = 'master' if drone_id == LEADER_DRONE else 'slave'
        figure_role = config.get('role') or (default_figures.pop(0) if default_figures else 'pawn')
        agents.append(Agent({
            "id": drone_id, "role": role_in_llm, "figure": figure_role,
            "character": CHARACTER_MAP.get(figure_role, CHARACTER_MAP["pawn"])
        }))

    # --- Create the single, comprehensive prompt ---
    agent_descriptions = []
    for agent in agents:
        role_str = "Мастер" if agent.role == 'master' else "Ведомый"
        # Pass the english figure name for the parser
        agent_descriptions.append(f"- ({agent.figure}) {RUSSIAN_ROLES.get(agent.figure)} ({role_str}): Характер - {agent.character}")
    
    agents_prompt_part = "\n".join(agent_descriptions)

    main_prompt = f"""
Вы — симулятор обсуждения шахматной партии между несколькими ИИ-агентами (дронами).
Ваша задача — сгенерировать полный лог их обсуждения и принять итоговое решение.

Текущая позиция на доске (FEN): {board.fen}
Ход белых.

Участники обсуждения:
{agents_prompt_part}

СИМУЛЯЦИЯ ОБСУЖДЕНИЯ:

ШАГ 1: ПЕРВОНАЧАЛЬНЫЕ МНЕНИЯ
Каждый агент должен предложить свой ход в формате UCI и дать краткое обоснование (1-2 предложения), исходя из своего характера.

ШАГ 2: ПРЕДЛОЖЕНИЕ МАСТЕРА
Мастер (Король) анализирует все предложения от ведомых. Он должен выбрать лучший, по его мнению, ход, и написать развернутое обоснование, почему этот ход лучше остальных, пытаясь убедить ведомых.

ШАГ 3: ОБРАТНАЯ СВЯЗЬ ОТ ВЕДОМЫХ
Каждый ведомый агент отвечает на предложение Мастера. Он должен указать, согласен он или нет, и почему.

ШАГ 4: ИТОГОВОЕ РЕШЕНИЕ (ГОЛОСОВАНИЕ)
Проанализируйте все ходы, предложенные в ходе обсуждения. Подсчитайте, какой ход упоминался чаще всего. Этот ход становится победителем. В случае ничьей, решение Мастера является окончательным.

ФОРМАТ ВЫВОДА:
Сгенерируйте ВЕСЬ диалог, используя СТРОГО следующие префиксы в начале каждой строки:
- Для мнения фигуры: `FIGURE:<figure_en>:<role_ru>:<текст мнения>` (например, `FIGURE:queen:Ведомый:Предлагаю e2e4...`)
- Для решения Мастера: `MASTER:<текст решения>`
- Для итогового хода в самом конце: `FINAL_MOVE:<uci_ход>`

Начинайте симуляцию. Весь диалог должен быть на русском языке.
"""

    # --- Execute LLM Call and Parse Response ---
    print("SYSTEM: --- Генерация полного обсуждения... (это может занять до 60 секунд) ---", flush=True)
    full_discussion = call_llm(main_prompt)

    if not full_discussion:
        print("ERROR: Не удалось получить ответ от LLM для симуляции обсуждения.", flush=True)
        return

    # --- Parse and stream the response ---
    print("SYSTEM: --- Воспроизведение обсуждения ---", flush=True)
    lines = full_discussion.strip().split('\n')
    final_move_line = None
    
    for line in lines:
        if line.strip(): # Process only non-empty lines
            if line.startswith("FINAL_MOVE:"):
                final_move_line = line # Save for the end
            else:
                print(line, flush=True)
                time.sleep(2) # Simulate thinking time

    # Print the final move last
    if final_move_line:
        print(final_move_line, flush=True)
    else:
        # Fallback if FINAL_MOVE is not in the response
        print("ERROR: LLM не предоставил итоговый ход в нужном формате.", flush=True)
        def extract_move(text):
            if not text: return None
            match = re.search(r'\b[a-h][1-8][a-h][1-8][qrbn]?\b', text.lower())
            return match.group(0) if match else None
        
        final_move = extract_move(full_discussion)
        if final_move:
            print(f"FINAL_MOVE:{final_move}", flush=True)

if __name__ == '__main__':
    # This block is for direct testing of the script
    import sys
    
    # Если передан FEN как аргумент, используем его
    if len(sys.argv) > 1:
        fen = sys.argv[1]
        start_board = BoardState(fen=fen)
    else:
        # Иначе используем начальную позицию
        start_board = BoardState(fen="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")
    
    get_turn(start_board)
