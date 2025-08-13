import chess
import sys
from drone.alg import get_turn, BoardState, AlgPermanentError, AlgTemporaryError

def run_test():
    """
    Запускает автоматический тест для проверки основной логики alg.py.
    Этот тест не требует ввода пользователя и выводит лог взаимодействия.
    """
    sys.path.append('.')
    board = chess.Board()
    
    # Определяем ходы для одной стороны (белых). Stockfish будет играть за черных.
    white_test_moves = ["e2e4", "g1f3", "f1b5", "e1g1"] # Часть Испанской партии
    
    print("=" * 40)
    print("Запуск автоматического теста логики")
    print("=" * 40)
    print("Тест играет за белых, Stockfish - за черных.")

    for move_number, white_move_uci in enumerate(white_test_moves, 1):
        # --- Ход Белых (из тестового списка) ---
        print(f"\n--- Ход {move_number} (White) ---")
        
        if board.turn != chess.WHITE:
            print("ОШИБКА: Ожидался ход белых, но сейчас ход черных.")
            break

        move = chess.Move.from_uci(white_move_uci)
        if move not in board.legal_moves:
            print(f"ОШИБКА: Тестовый ход {white_move_uci} нелегален.")
            break
        
        san_move = board.san(move)
        board.push(move)
        print(f"Тестовый ход: {san_move} ({white_move_uci})")
        print(f"Новый FEN: {board.fen()}")

        if board.is_game_over():
            break

        # --- Ход Черных (Stockfish) ---
        print(f"\n--- Ход {move_number} (Black) ---")

        if board.turn != chess.BLACK:
            print("ОШИБКА: Ожидался ход черных, но сейчас ход белых.")
            break

        board_state = BoardState(
            fen=board.fen(),
            turn='b',
            move_number=move_number,
            timestamp=0,
        )
        
        print(f"\n[INPUT]  Отправка в get_turn():")
        print(f"  - fen: '{board_state.fen}'")
        print(f"  - turn: '{board_state.turn}'")

        try:
            decision = get_turn(board_state)
            stockfish_move = chess.Move.from_uci(decision.uci)

            if stockfish_move not in board.legal_moves:
                print(f"ОШИБКА: Stockfish предложил нелегальный ход: {decision.uci}")
                break
            
            # Получаем SAN ПЕРЕД тем, как сделать ход
            stockfish_san = board.san(stockfish_move)
            
            print(f"[OUTPUT] Получено из get_turn():")
            print(f"  - uci: '{decision.uci}'")
            print(f"  - from_cell: '{decision.from_cell}'")
            print(f"  - to_cell: '{decision.to_cell}'")
            print(f"  - reason: '{decision.reason}'")
            
            board.push(stockfish_move)
            print(f"Stockfish ответил ходом: {stockfish_san} ({decision.uci})")
            print(f"Новый FEN: {board.fen()}")

        except (AlgPermanentError, AlgTemporaryError) as e:
            print(f"ОШИБКА: Исключение от alg.py: {e}")
            break
        except Exception as e:
            print(f"ОШИБКА: Непредвиденное исключение: {e}")
            break
        
        if board.is_game_over():
            break

    print("\n" + "=" * 40)
    print("Тест завершен.")
    print(f"Финальная позиция: {board.fen()}")
    print(f"Результат: {board.result()}")
    print("=" * 40)

if __name__ == "__main__":
    run_test()
