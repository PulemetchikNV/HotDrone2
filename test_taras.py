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

def is_fen_valid(fen: str) -> bool:
    """Проверяет, является ли FEN-строка валидной."""
    try:
        chess.Board(fen)
        return True
    except ValueError:
        return False

def main():
    """
    Интерактивный тест для alg.py.
    Позволяет вводить ходы и получать ответ от Stockfish.
    """
    sys.path.append('.')
    board = chess.Board()
    move_number = 1

    print("=" * 30)
    print("Интерактивный тест Stockfish")
    print("=" * 30)
    print("Команды:")
    print("  - <uci_move> (e.g., e2e4): сделать ход")
    print("  - 'board': показать доску")
    print("  - 'fen': показать FEN")
    print("  - 'q' или 'quit': выйти")
    print("=" * 30)

    while not board.is_game_over():
        turn_color = "White" if board.turn == chess.WHITE else "Black"
        prompt = f"Ход {move_number} ({turn_color}): "
        user_input = input(prompt).strip().lower()

        if user_input in ["q", "quit"]:
            print("Выход.")
            break
        elif user_input == "board":
            print_board(board)
            continue
        elif user_input == "fen":
            print(f"FEN: {board.fen()}")
            continue

        try:
            user_move = chess.Move.from_uci(user_input)
            if user_move not in board.legal_moves:
                print(f"Нелегальный ход: {user_input}. Попробуйте снова.")
                continue
        except ValueError:
            print(f"Некорректный формат хода: {user_input}. Используйте UCI.")
            continue

        san_move = board.san(user_move)
        if "x" in san_move:
            print("Вы съели фигуру!")
        board.push(user_move)
        print(f"Ваш ход: {san_move} ({user_input})")
        print_board(board)

        if board.is_game_over():
            break

        print("Stockfish думает...")
        
        current_fen = board.fen()
        if not is_fen_valid(current_fen):
            print(f"Ошибка: сгенерирован невалидный FEN: {current_fen}")
            break

        board_state = BoardState(
            fen=current_fen,
            turn='b' if board.turn == chess.BLACK else 'w',
            move_number=move_number,
            timestamp=0,
        )

        try:
            decision = get_turn(board_state)
            stockfish_move = chess.Move.from_uci(decision.uci)

            if stockfish_move not in board.legal_moves:
                print(f"Stockfish предложил нелегальный ход: {decision.uci}")
                break
            
            san_move = board.san(stockfish_move)
            if "x" in san_move:
                print("Stockfish съел вашу фигуру!")
            print(f"Stockfish предлагает ход: {san_move} ({decision.uci})")
            board.push(stockfish_move)
            print_board(board)
            move_number += 1

        except (AlgPermanentError, AlgTemporaryError) as e:
            print(f"Ошибка при вызове Stockfish: {e}")
            break
        except Exception as e:
            print(f"Непредвиденная ошибка: {e}")
            break

    print("\nИгра окончена.")
    print_board(board)
    print(f"Результат: {board.result()}")

if __name__ == "__main__":
    main()
