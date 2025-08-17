#!/usr/bin/env python3
"""
Тест интеграции LLM chess алгоритма
"""

import sys
import os
sys.path.append(os.path.abspath('drone'))

from drone.utils import BoardState
from drone.alg_llm_chess import get_turn

def test_llm_chess():
    """Тестирует LLM chess алгоритм"""
    print("=== Тест LLM Chess алгоритма ===")
    
    # Позиция после хода белых e2e4
    import time
    board = BoardState(
        fen="rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1",
        turn="b",
        move_number=1,
        timestamp=time.time(),
        meta={"current_cell": "e7"}
    )
    
    print(f"FEN: {board.fen}")
    print("Запрос хода к LLM серверу...")
    
    try:
        result = get_turn(board, time_budget_ms=30000)
        print(f"Результат: {result}")
        print(f"UCI ход: {result.uci}")
        print(f"От: {result.from_cell} -> К: {result.to_cell}")
        print(f"Причина: {result.reason}")
    except Exception as e:
        print(f"Ошибка: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_llm_chess()
