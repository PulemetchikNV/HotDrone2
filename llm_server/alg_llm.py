import json
import subprocess
import os
import time
import sys

# Add the parent directory to the Python path to allow sibling imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from drone.const import DRONES_CONFIG, LEADER_DRONE


API_KEY = os.environ.get("GOOGLE_API_KEY", "AIzaSyAy817y7DirgpWnMPS6t5ps-6Ui2pFAMys")
LLM_URL = "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent"
PROXY = "socks5h://danterobot:zxckaliparrot@83.151.2.50:8543"
SHARED_FILE = "moves.txt"
MAX_ITERATIONS = 3 # The 'n' from the description

# Map drone roles from const.py to descriptive characters for the LLM
CHARACTER_MAP = {
    "king": "Очень осторожный и стратегический игрок. Предпочитает защитные ходы и контроль центра доски. Действует как лидер, обобщая идеи.",
    "queen": "Агрессивный игрок. Всегда ищет возможности для атаки на короля противника, даже если это сопряжено с риском.",
    "default": "Позиционный игрок. Сосредоточен на долгосрочных преимуществах, пешечной структуре и координации фигур. Очень логичен."
}

# Dictionary for translating roles to Russian
RUSSIAN_ROLES = {
    "king": "Король",
    "queen": "Королева",
    "bishop": "Слон",
    "pawn": "Пешка",
    "master": "Мастер",
    "slave": "Ведомый"
}

def call_llm(prompt):
    """
    Calls the Gemini LLM with a given prompt.
    """
    headers = {
        'Content-Type': 'application/json',
        'X-goog-api-key': API_KEY,
    }
    data = {
        "contents": [
            {
                "parts": [
                    {
                        "text": prompt
                    }
                ]
            }
        ]
    }
    # Using a list of arguments for better security and handling of special characters
    command = [
        'curl',
        # '--proxy', PROXY, # Temporarily disabled due to connection errors
        LLM_URL,
        '-H', f"Content-Type: {headers['Content-Type']}",
        '-H', f"X-goog-api-key: {headers['X-goog-api-key']}",
        '-X', 'POST',
        '-d', json.dumps(data)
    ]

    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True, timeout=60)
        response_json = json.loads(result.stdout)
        # Extracting the text from the response
        return response_json['candidates'][0]['content']['parts'][0]['text']
    except subprocess.CalledProcessError as e:
        print(f"Error calling LLM: {e}")
        print(f"Stderr: {e.stderr}")
        return f"Error: Could not get a response from LLM. Details: {e.stderr}"
    except (KeyError, IndexError, json.JSONDecodeError) as e:
        print(f"Error parsing LLM response: {e}")
        print(f"Raw Response: {result.stdout}")
        return f"Error: Could not parse LLM response. Raw text: {result.stdout}"
    except subprocess.TimeoutExpired:
        print("Error: LLM call timed out.")
        return "Error: LLM call timed out."


class Agent:
    def __init__(self, drone_info):
        self.id = drone_info['id']
        self.role = drone_info['role'] # 'master' or 'slave'
        self.character = drone_info['character']
        self.figure = drone_info['figure'] # e.g., 'king', 'queen'

    def think(self, prompt):
        """Generic thinking method for an agent."""
        # Prepending character to every prompt to guide the LLM
        full_prompt = f"Вы — шахматный ассистент. Ваш характер: '{self.character}'.\n\n{prompt}"
        response = call_llm(full_prompt)
        return response

class MultiAgentSystem:
    def __init__(self, drones_json, initial_game_state):
        self.agents = [Agent(drone_info) for drone_info in drones_json]
        self.master = next((agent for agent in self.agents if agent.role == 'master'), None)
        self.slaves = [agent for agent in self.agents if agent.role == 'slave']
        self.game_state = initial_game_state
        self.chat_history = f"Initial board state (FEN): {initial_game_state}\n\n"

    def run_discussion(self):
        if not self.master:
            print("Ошибка: не найден главный агент.", flush=True)
            return None

        for i in range(MAX_ITERATIONS):
            print(f"--- Итерация {i+1} ---", flush=True)
            
            # 1. All drones suggest a move
            current_suggestions = ""
            print("Агенты обдумывают свои ходы...", flush=True)
            for agent in self.agents:
                prompt = f"""
                Текущее состояние шахматной партии (FEN): {self.game_state}
                История обсуждения на данный момент:
                {self.chat_history}

                Основываясь на вашем характере и состоянии игры, какой ход является лучшим в формате UCI (например, e2e4)?
                Ответьте кратко, в паре предложений. Затем укажите ход на новой строке в формате: MOVE: <ваш_ход>
                """
                thought = agent.think(prompt)
                figure_ru = RUSSIAN_ROLES.get(agent.figure, agent.figure.capitalize())
                role_ru = RUSSIAN_ROLES.get(agent.role, agent.role)
                print(f"Агент {figure_ru} ({role_ru}):\n{thought}\n", flush=True)
                suggestion_text = f"Agent {agent.id} ({agent.role}):\n{thought}\n\n"
                current_suggestions += suggestion_text
            
            self.chat_history += f"--- Итерация {i+1} Предложения ---\n{current_suggestions}"
            with open(SHARED_FILE, "w") as f:
                f.write(self.chat_history)
            print("Все агенты сделали свои предложения.", flush=True)

            # 2. Master reads and decides
            master_decision_prompt = f"""
            Вы — главный шахматный агент. Вам нужно выбрать лучший ход на основе предложений от ваших ведомых агентов.
            Текущее состояние игры (FEN): {self.game_state}
            Вот полная история обсуждения:
            {self.chat_history}

            Проанализируйте эти предложения и выберите единственно лучший ход в формате UCI.
            Кроме того, для каждого ведомого агента приведите персональный аргумент, почему выбранный вами ход лучше его предложения.
            Отформатируйте ваш ответ в виде чистого JSON-объекта без дополнительного текста или markdown-разметки.
            JSON должен выглядеть так:
            {{
                "best_move": "e2e4",
                "reasoning": "Этот ход открывает линии для ферзя и слона...",
                "feedback_to_slaves": {{
                    "drone_slave_1": "Ваше предложение g1f3 было хорошим, но e2e4 сильнее, потому что...",
                    "drone_slave_2": "Хотя d2d4 — это надежный выбор, он не..."
                }}
            }}
            """
            print("Мастер принимает решение...", flush=True)
            master_response_str = self.master.think(master_decision_prompt)
            
            try:
                # Clean up the response string to ensure it's valid JSON
                json_str = master_response_str[master_response_str.find('{'):master_response_str.rfind('}')+1]
                master_decision = json.loads(json_str)
                best_move = master_decision['best_move']
                master_reasoning = master_decision['reasoning']
                
                master_summary = f"--- Итерация {i+1} Решение Мастера ---\n"
                master_summary += f"Главный агент ({self.master.id}) выбрал ход: {best_move}\n"
                master_summary += f"Обоснование: {master_reasoning}\n\n"
                self.chat_history += master_summary
                print(f"Мастер выбрал ход: {best_move}", flush=True)

            except (json.JSONDecodeError, KeyError) as e:
                print(f"Ошибка разбора решения мастера: {e}", flush=True)
                print(f"Необработанный ответ мастера: {master_response_str}", flush=True)
                # If master fails to decide, break the loop and try to make a final decision based on history
                break

            # 3. Slaves react to master's decision
            slave_feedback_text = f"--- Итерация {i+1} Обратная связь от ведомых ---\n"
            print("Ведомые предоставляют обратную связь...", flush=True)
            for slave in self.slaves:
                feedback_for_slave = master_decision.get('feedback_to_slaves', {}).get(slave.id, "No specific feedback for you.")
                slave_reaction_prompt = f"""
                Главный агент решил сделать ход: {best_move}.
                Обоснование мастера: {master_reasoning}
                Отзыв мастера конкретно для вас: {feedback_for_slave}

                Согласны ли вы с решением мастера? Объясните свою точку зрения кратко, в паре предложений.
                """
                reaction = slave.think(slave_reaction_prompt)
                figure_ru = RUSSIAN_ROLES.get(slave.figure, slave.figure.capitalize())
                role_ru = RUSSIAN_ROLES.get(slave.role, slave.role)
                print(f"Агент {figure_ru} ({role_ru}):\n{reaction}\n", flush=True)
                slave_feedback_text += f"Agent {slave.id} ({slave.role}):\n{reaction}\n\n"
            
            self.chat_history += slave_feedback_text
            with open(SHARED_FILE, "w") as f:
                f.write(self.chat_history)
            print("Ведомые предоставили свою обратную связь.", flush=True)

        # Final decision after all iterations
        print("Подготовка финального решения...", flush=True)
        final_decision_prompt = f"""
        После {MAX_ITERATIONS} итераций обсуждения, итоговая история чата такова:
        {self.chat_history}
        Основываясь на всем этом обсуждении, какой итоговый, согласованный ход в формате UCI?
        Выведите только один лучший ход и ничего больше (например, e2e4).
        """
        final_move = self.master.think(final_decision_prompt).strip()
        # Clean up the final move response
        final_move = final_move.split('\n')[-1]

        final_summary = f"\n--- ФИНАЛЬНОЕ РЕШЕНИЕ ---\nИтоговый согласованный ход: {final_move}\n"
        self.chat_history += final_summary
        with open(SHARED_FILE, "w") as f:
            f.write(self.chat_history)

        print(final_summary, flush=True)
        return final_move


def main():
    # Game state in Forsyth-Edwards Notation (FEN)
    # This is the starting position of a chess game.
    game_state_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"

    # Dynamically build the drones_json from the imported DRONES_CONFIG
    drones_json = []
    for drone_id, config in DRONES_CONFIG.items():
        role_in_llm = 'master' if drone_id == LEADER_DRONE else 'slave'
        figure_role = config.get('role', 'pawn') # Default to 'pawn' if not specified
        character = CHARACTER_MAP.get(figure_role, CHARACTER_MAP['default'])
        
        drones_json.append({
            "id": drone_id,
            "role": role_in_llm,
            "character": character,
            "figure": figure_role
        })

    multi_agent_system = MultiAgentSystem(drones_json, game_state_fen)
    optimal_move = multi_agent_system.run_discussion()

    if optimal_move:
        print(f"\nОптимальный ход для отправки главному дрону: {optimal_move}", flush=True)
    else:
        print("\nНе удалось определить оптимальный ход.", flush=True)


if __name__ == "__main__":
    main()
