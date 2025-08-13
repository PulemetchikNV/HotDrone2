import chess
import sys
from drone.alg import get_turn, BoardState, AlgPermanentError, AlgTemporaryError

def print_board(board: chess.Board):
    """Prints the chess board to the console."""
    print("\n  a b c d e f g h")
    print(" +-----------------+")
    for i in range(8, 0, -1):
        print(f"{i}|", end=" ")
        for j in range(8):
            square = chess.square(j, i - 1)
            piece = board.piece_at(square)
            print(piece.symbol() if piece else ".", end=" ")
        print(f"|{i}")
    print(" +-----------------+")
    print("  a b c d e f g h\n")

def main():
    """
    Интерактивный тест для alg.py.
    Позволяет вводить ходы и получать ответ от Stockfish.
    """
    # Убедимся, что drone/ находится в пути для импорта
    sys.path.append('.')

    board = chess.Board()
    move_number = 1

    print("=" * 30)
    print("Интерактивный тест Stockfish")
    print("=" * 30)
    print("Введите ход в формате UCI (например, e2e4).")
    print("Введите 'q' или 'quit' для выхода.")

    while not board.is_game_over():
        print_board(board)
        
        # Определяем, чей ход
        turn_color = "White" if board.turn == chess.WHITE else "Black"
        print(f"FEN: {board.fen()}")
        print(f"Ход {move_number}: {turn_color} to move.")

        # 1. Ход пользователя
        user_move_uci = input("Ваш ход (e.g., e2e4): ").strip().lower()

        if user_move_uci in ["q", "quit"]:
            print("Выход.")
            break

        try:
            user_move = chess.Move.from_uci(user_move_uci)
            if user_move not in board.legal_moves:
                print(f"Нелегальный ход: {user_move_uci}. Попробуйте снова.")
                continue
        except ValueError:
            print(f"Некорректный формат хода: {user_move_uci}. Используйте UCI (e.g. e2e4).")
            continue

        board.push(user_move)
        print_board(board)

        if board.is_game_over():
            print("Игра окончена.")
            break
            
        # 2. Ход Stockfish
        print("Stockfish думает...")
        
        # Создаем BoardState для передачи в alg.py
        board_state = BoardState(
            fen=board.fen(),
            turn='b' if board.turn == chess.BLACK else 'w',
            move_number=move_number,
            timestamp=0, # В тесте не используется
        )

        try:
            # Получаем ход от движка
            decision = get_turn(board_state, time_budget_ms=2000)
            print(f"Stockfish предлагает ход: {decision.uci}")
            
            stockfish_move = chess.Move.from_uci(decision.uci)
            if stockfish_move not in board.legal_moves:
                print(f"Stockfish предложил нелегальный ход: {decision.uci}")
                # Это может случиться, если FEN или логика ходов рассинхронизированы
                break

            board.push(stockfish_move)
            move_number += 1

        except (AlgPermanentError, AlgTemporaryError) as e:
            print(f"Ошибка при вызове Stockfish: {e}")
            break
        except Exception as e:
            print(f"Непредвиденная ошибка: {e}")
            break

    print("\nФинальная позиция:")
    print_board(board)
    print(f"Результат: {board.result()}")

if __name__ == "__main__":
    main()
