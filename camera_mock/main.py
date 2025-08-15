from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import os
import copy

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
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
    # Копия структуры из MockCameraController в drone/camera.py
    white = {
        'king':      {'x': 0.0,  'y': -3.5, 'cell': 'e1'},
        'queen':     {'x': -0.5, 'y': -3.5, 'cell': 'd1'},
        'rook_a1':   {'x': -3.5, 'y': -3.5, 'cell': 'a1'},
        'rook_h1':   {'x': 3.5,  'y': -3.5, 'cell': 'h1'},
        'knight_b1': {'x': -2.5, 'y': -3.5, 'cell': 'b1'},
        'knight_g1': {'x': 2.5,  'y': -3.5, 'cell': 'g1'},
        'bishop_c1': {'x': -1.5, 'y': -3.5, 'cell': 'c1'},
        'bishop_f1': {'x': 1.5,  'y': -3.5, 'cell': 'f1'},
        'pawn_a2':   {'x': -3.5, 'y': -2.5, 'cell': 'a2'},
        'pawn_b2':   {'x': -2.5, 'y': -2.5, 'cell': 'b2'},
        'pawn_c2':   {'x': -1.5, 'y': -2.5, 'cell': 'c2'},
        'pawn_d2':   {'x': -0.5, 'y': -2.5, 'cell': 'd2'},
        'pawn_e2':   {'x': 0.5, 'y': -2.5, 'cell': 'e2'},
        'pawn_f2':   {'x': 1.5, 'y': -2.5, 'cell': 'f2'},
        'pawn_g2':   {'x': 2.5, 'y': -2.5, 'cell': 'g2'},
        'pawn_h2':   {'x': 3.5, 'y': -2.5, 'cell': 'h2'},
    }
    black = {
        #'king':      {'x': 0.0,  'y': 3.5, 'cell': 'e8'},
        'queen': {'x': 0.0,  'y': 3.5, "cell": 'd8'}
    }
    return {'white': white, 'black': black}


# Глобальное состояние игры (максимально простое)
POSITIONS = make_initial_positions()
TURN = 'white'  # white начинает


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


class Move(BaseModel):
    move: str | None = None  # формат 'e2e4'
    from_cell: str | None = None
    to_cell: str | None = None


@app.get("/")
def read_root():
    return FileResponse(os.path.join(ui_directory, "index.html"))


@app.get("/board_state")
def get_board_state():
    fen = positions_to_fen(POSITIONS, TURN)
    return {"board_fen": fen, "turn": TURN}


@app.get("/api/positions")
def api_positions():
    # Совместимо с ожидаемым форматом в drone/camera.py
    payload = copy.deepcopy(POSITIONS)
    payload['turn'] = TURN
    fen = positions_to_fen(POSITIONS, TURN)
    payload['fen'] = fen
    return payload


@app.post("/make_move")
def make_move(move: Move):
    global TURN

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


@app.post("/reset_board")
def reset_board():
    global POSITIONS, TURN
    POSITIONS = make_initial_positions()
    TURN = 'white'
    return get_board_state()


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
