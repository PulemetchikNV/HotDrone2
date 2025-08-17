from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import os
import copy
import json
import threading

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

def load_aruco_map():
    """Загружает координаты из aruco_map1.json"""
    try:
        # Путь к карте ArUco относительно camera_mock
        map_path = os.path.join(os.path.dirname(__file__), '..', 'aruco_maps', 'aruco_map1.json')
        with open(map_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Warning: Failed to load aruco_map2.json: {e}")
        return {}
ui_directory = os.path.dirname(__file__)

FILES = "abcdefgh"
RANKS = "12345678"
CELL_SIZE = 1.0
ORIGIN_X = 0.0
ORIGIN_Y = 0.0


def cell_to_xy(cell: str):
    f, r = cell[0], cell[1]
    fi = FILES.index(f)
    ri = RANKS.index(r)
    x = ORIGIN_X + (fi - 3.5) * CELL_SIZE
    y = ORIGIN_Y + (ri - 3.5) * CELL_SIZE
    return x, y


def make_initial_positions():
    """Создает начальные позиции фигур, используя координаты из aruco_map2.json"""
    aruco_map = load_aruco_map()
    
    def get_aruco_coords(cell):
        """Получает координаты клетки из ArUco карты"""
        if cell in aruco_map:
            return aruco_map[cell]['x'], aruco_map[cell]['y']
        else:
            print(f"Warning: No ArUco coordinates for {cell}, using fallback")
            # Fallback к старой системе (если нет в карте)
            file_idx = FILES.index(cell[0])
            rank_idx = RANKS.index(cell[1])
            x = (file_idx - 3.5) * 0.436  # ~40см между клетками
            y = (rank_idx - 3.5) * 0.436
            return x, y
    
    # Используем реальные координаты из ArUco карты
    white = {}
    white_pieces = {
        'king': 'h3', 
        'queen': 'd5',
        'knight_b4': 'b4',  
        'pawn_a2': 'a2',
        'pawn_g3': 'g3', 
        'pawn_h2': 'h2', 
    }
    
    for piece_name, cell in white_pieces.items():
        x, y = get_aruco_coords(cell)
        white[piece_name] = {'x': x, 'y': y, 'cell': cell}
    
    # Черные (только те, что активны в тесте)
    black = {}
    black_pieces = {
        'king': 'h8',
        'queen': 'f6',
        'bishop_b2': 'b2',
        'pawn_f5': 'f5',
        'pawn_g5': 'g5',
        'pawn_h7': 'h7',
    }
    
    for piece_name, cell in black_pieces.items():
        x, y = get_aruco_coords(cell)
        black[piece_name] = {'x': x, 'y': y, 'cell': cell}
    
    print(f"ArUco coordinates loaded for mock camera:")
    print(f"  b7: {black.get('pawn_b7', {})}")
    print(f"  b6: {get_aruco_coords('b6')}")
    
    return {'white': white, 'black': black}


# Глобальное состояние игры (максимально простое)
POSITIONS = make_initial_positions()
TURN = 'white'  # white начинает
IS_PAUSED = False  # Состояние паузы

# Рентерабельная блокировка для защиты состояния игры от гонок
GAME_LOCK = threading.RLock()

# История состояний для отката ходов
GAME_HISTORY = []  # Хранит состояния (positions, turn) перед каждым ходом


def positions_to_fen(positions: dict, turn: str) -> str:
    # Построим 8x8 сетку пустых клеток
    board = [[None for _ in range(8)] for _ in range(8)]

    def base_to_symbol(base: str, color: str) -> str:
        m = {
            'king': 'k', 'queen': 'q', 'rook': 'r', 'knight': 'n', 'bishop': 'b', 'pawn': 'p'
        }
        s = m.get(base, 'p')
        return s.upper() if color == 'white' else s

    def put(color: str, piece_key: str, cell: str):
        if not (isinstance(cell, str) and len(cell) == 2 and cell[0] in FILES and cell[1] in RANKS):
            return
        file_idx = FILES.index(cell[0])
        rank_idx = RANKS.index(cell[1])
        row = 8 - int(cell[1])  # 0..7, где 0 = 8-я горизонталь
        col = file_idx          # 0..7, где 0 = столбец 'a'
        base = piece_key.split('_')[0]
        board[row][col] = base_to_symbol(base, color)

    for color, color_data in positions.items():
        for piece_key, info in color_data.items():
            put(color, piece_key, info.get('cell', ''))

    # Конвертируем в FEN piece placement
    ranks = []
    for row in range(8):
        empty = 0
        fen_row = ''
        for col in range(8):
            sym = board[row][col]
            if sym is None:
                empty += 1
            else:
                if empty > 0:
                    fen_row += str(empty)
                    empty = 0
                fen_row += sym
        if empty > 0:
            fen_row += str(empty)
        ranks.append(fen_row)

    placement = '/'.join(ranks)
    active = 'w' if turn == 'white' else 'b'
    # Без рокировок, en passant, счетчиков — чтобы chessboard.js просто показал позицию
    return f"{placement} {active} - - 0 1"


def find_piece_at_cell(cell: str):
    for color in ['white', 'black']:
        for piece_key, info in POSITIONS[color].items():
            if info.get('cell') == cell:
                return color, piece_key
    return None, None


def save_current_state():
    """Сохраняет текущее состояние игры в историю"""
    global GAME_HISTORY
    state = {
        'positions': copy.deepcopy(POSITIONS),
        'turn': TURN
    }
    GAME_HISTORY.append(state)
    # Ограничиваем размер истории (например, последние 50 ходов)
    if len(GAME_HISTORY) > 50:
        GAME_HISTORY.pop(0)


class Move(BaseModel):
    move: str | None = None  # формат 'e2e4'
    from_cell: str | None = None
    to_cell: str | None = None


@app.get("/")
def read_root():
    return FileResponse(os.path.join(ui_directory, "index.html"))


@app.get("/board_state")
def get_board_state():
    with GAME_LOCK:
        fen = positions_to_fen(POSITIONS, TURN)
        return {"board_fen": fen, "turn": TURN}


@app.get("/api/positions")
def api_positions():
    # Совместимо с ожидаемым форматом в drone/camera.py
    with GAME_LOCK:
        payload = copy.deepcopy(POSITIONS)
        payload['turn'] = TURN
        fen = positions_to_fen(POSITIONS, TURN)
        payload['fen'] = fen
        return payload


@app.post("/make_move")
def make_move(move: Move):
    global TURN
    with GAME_LOCK:
        # Сохраняем текущее состояние перед выполнением хода
        save_current_state()

        # Распарсим ходы
        if move.move:
            if len(move.move) != 4:
                raise HTTPException(status_code=400, detail="Invalid move format; expected UCI like e2e4")
            from_cell = move.move[:2]
            to_cell = move.move[2:]
        else:
            from_cell = (move.from_cell or '').lower()
            to_cell = (move.to_cell or '').lower()
            if len(from_cell) != 2 or len(to_cell) != 2:
                raise HTTPException(status_code=400, detail="Invalid from/to cell")

        # Найти фигуру на исходной клетке
        color, piece_key = find_piece_at_cell(from_cell)
        print(f"{color} {piece_key} Making move {from_cell}->{to_cell}")
        if not piece_key:
            raise HTTPException(status_code=400, detail=f"No piece at {from_cell}")

        # Ходить можно только стороной, у которой сейчас ход
        if color != TURN:
            raise HTTPException(status_code=400, detail=f"It is {TURN}'s turn")

        # Съесть, если на целевой клетке есть фигура любой стороны
        cap_color, cap_piece = find_piece_at_cell(to_cell)
        if cap_piece:
            del POSITIONS[cap_color][cap_piece]

        # Переместить фигуру (без правил, просто перенос)
        x, y = cell_to_xy(to_cell)
        POSITIONS[color][piece_key]['cell'] = to_cell
        POSITIONS[color][piece_key]['x'] = x
        POSITIONS[color][piece_key]['y'] = y

        # Сменить ход
        TURN = 'black' if TURN == 'white' else 'white'

        return get_board_state()


@app.post("/undo_move")
def undo_move():
    """Откатывает последний ход"""
    global POSITIONS, TURN, GAME_HISTORY
    with GAME_LOCK:
        if not GAME_HISTORY:
            raise HTTPException(status_code=400, detail="No moves to undo")
        # Восстанавливаем последнее сохраненное состояние
        last_state = GAME_HISTORY.pop()
        POSITIONS = last_state['positions']
        TURN = last_state['turn']
        return get_board_state()


@app.post("/reset_board")
def reset_board():
    global POSITIONS, TURN, GAME_HISTORY
    with GAME_LOCK:
        POSITIONS = make_initial_positions()
        TURN = 'white'
        GAME_HISTORY = []  # Очищаем историю при сбросе
        return get_board_state()


@app.get("/api/pause_status")
def get_pause_status():
    """Возвращает текущее состояние паузы"""
    return {"is_paused": IS_PAUSED}


@app.post("/api/toggle_pause")
def toggle_pause():
    """Переключает состояние паузы"""
    global IS_PAUSED
    IS_PAUSED = not IS_PAUSED
    return {"is_paused": IS_PAUSED, "message": f"Game {'paused' if IS_PAUSED else 'resumed'}"}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
