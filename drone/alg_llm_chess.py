import os
import time
import json
import httpx
from dataclasses import dataclass
from typing import Optional, Dict, Any, Union

from const import OUR_TEAM, get_current_drone_config
from utils import BoardState, MoveDecision, AlgTemporaryError, AlgPermanentError

# Для Python 3.7 совместимости
try:
    from typing import Literal
except ImportError:
    try:
        from typing_extensions import Literal
    except ImportError:
        def Literal(*args):
            return str

FILES = "abcdefgh"
RANKS = "12345678"

def _normalize_cell(cell: str) -> str:
    if not isinstance(cell, str) or len(cell) != 2:
        raise AlgPermanentError("cell must be like 'e2'")
    f, r = cell[0].lower(), cell[1]
    if f not in FILES or r not in RANKS:
        raise AlgPermanentError(f"invalid cell: {cell}")
    return f + r

def get_turn(board: BoardState, time_budget_ms: int = 5000, seed: Optional[int] = None) -> MoveDecision:
    """Получает ход от LLM сервера через HTTP API."""
    if board is None or not hasattr(board, 'fen'):
        raise AlgPermanentError("board must have fen attribute")

    fen = board.fen
    print(f"SENDING FEN TO LLM SERVER: {fen}")
    
    try:
        timeout_s = max(1.0, float(time_budget_ms) / 1000.0)
        
        # Запускаем LLM алгоритм через сервер
        with httpx.Client(timeout=timeout_s + 60) as client:  # +60 секунд для LLM
            resp = client.post("http://192.168.1.119:8080/run", json={"fen": fen})
            resp.raise_for_status()
            
            # Читаем потоковый ответ
            final_move = None
            for line in resp.iter_lines():
                if line:
                    # line уже строка, не нужно decode
                    line_str = line if isinstance(line, str) else line.decode('utf-8')
                    if line_str.startswith('data: '):
                        content = line_str[6:]  # Убираем 'data: '
                        print(f"LLM OUTPUT: {content}")
                        
                        # Ищем финальный ход
                        if content.startswith('FINAL_MOVE:'):
                            final_move = content[11:].strip()  # Убираем 'FINAL_MOVE:'
                            break
            
            if not final_move:
                raise AlgTemporaryError("LLM server did not return a valid move")
            
            # Парсим UCI ход
            if len(final_move) < 4:
                raise AlgTemporaryError(f"Invalid UCI move format: {final_move}")
            
            from_cell = final_move[0:2]
            to_cell = final_move[2:4]
            
            # Проверяем цвет команды
            try:
                drone_config = get_current_drone_config()
                our_color = drone_config['team']
            except (ValueError, KeyError):
                our_color = OUR_TEAM
            
            # Проверяем чей ход по FEN
            fen_parts = fen.split()
            if len(fen_parts) >= 2:
                current_turn = fen_parts[1]  # 'w' или 'b'
                if our_color == 'black' and current_turn == 'w':
                    print(f"NOT OUR TURN: white's turn in FEN")
                    return MoveDecision(
                        uci="",
                        from_cell="",
                        to_cell="",
                    )
                elif our_color == 'white' and current_turn == 'b':
                    print(f"NOT OUR TURN: black's turn in FEN")
                    return MoveDecision(
                        uci="",
                        from_cell="",
                        to_cell="",
                    )

            move_decision = MoveDecision(
                uci=final_move,
                from_cell=_normalize_cell(from_cell),
                to_cell=_normalize_cell(to_cell),
                score_cp=None,
                is_mate=False,
                reason="llm-chess",
                meta={
                    "engine": "llm-chess",
                    "fen": fen,
                },
            )
            
            # Логируем ход для polling
            try:
                log_data = {
                    'move': final_move,
                    'fen': fen,
                    'from_cell': _normalize_cell(from_cell),
                    'to_cell': _normalize_cell(to_cell),
                    'reason': 'llm-chess',
                    'engine': 'llm-chess'
                }
                log_resp = client.post("http://192.168.1.119:8080/log_move", json=log_data)
                if log_resp.status_code == 200:
                    print(f"Move logged successfully: {final_move}")
                else:
                    print(f"Failed to log move: {log_resp.status_code}")
            except Exception as log_e:
                print(f"Error logging move: {log_e}")
            
            return move_decision
            
    except httpx.RequestError as e:
        print(f"LLM Server connection error: {e}")
        raise AlgTemporaryError(f"LLM server connection failed: {e}")
    except Exception as e:
        print(f"LLM Server error: {e}")
        # Фолбэк на простой алгоритм
        meta = board.meta or {}
        cur = meta.get("current_cell")
        if not cur:
            raise AlgPermanentError("board.meta.current_cell is required for fallback")
        
        cur = _normalize_cell(cur)
        f_idx = FILES.index(cur[0])
        r_idx = RANKS.index(cur[1])
        if r_idx < 7:
            to_cell = f"{cur[0]}{RANKS[r_idx+1]}"
        elif f_idx < 7:
            to_cell = f"{FILES[f_idx+1]}1"
        else:
            to_cell = "a1"
        uci = f"{cur}{to_cell}"
        
        return MoveDecision(
            uci=uci,
            from_cell=cur,
            to_cell=to_cell,
            score_cp=0,
            is_mate=False,
            reason="llm-chess-fallback",
            meta={"policy": "simple_sequence"},
        )
