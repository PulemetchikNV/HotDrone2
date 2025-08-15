#!/usr/bin/env python3
"""
Simple test for alg2_stockfish.py
"""

import os
import sys
import time

# Add the drone directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'drone'))

from alg2_stockfish import (
    BoardState, MoveDecision, get_stockfish_status, test_stockfish_integration
)


def test_basic_functionality():
    """Test basic functionality of alg2_stockfish."""
    print("Testing alg2_stockfish.py...")
    
    # Test status
    status = get_stockfish_status()
    print(f"Stockfish status: {status}")
    
    # Test integration
    test_result = test_stockfish_integration()
    print(f"Integration test: {test_result}")
    
    # Test BoardState creation
    board = BoardState(
        fen="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
        turn="w",
        move_number=1,
        timestamp=time.time(),
        meta={"current_cell": "e2"}
    )
    print(f"BoardState created: {board.fen}")
    
    # Test MoveDecision creation
    move = MoveDecision(
        uci="e2e4",
        from_cell="e2",
        to_cell="e4",
        score_cp=50,
        is_mate=False,
        reason="test"
    )
    print(f"MoveDecision created: {move.uci}")
    
    print("Basic functionality test completed!")


if __name__ == '__main__':
    test_basic_functionality()
