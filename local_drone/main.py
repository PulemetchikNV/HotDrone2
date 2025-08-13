import subprocess
import time

def get_stockfish_move(fen_position):
    """
    Starts a Stockfish process and gets the best move for a given FEN position.
    """
    try:
        # Start the Stockfish process
        stockfish_process = subprocess.Popen(
            "stockfish",
            universal_newlines=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Send the FEN position to Stockfish
        stockfish_process.stdin.write(f"position fen {fen_position}\n")
        stockfish_process.stdin.write("go movetime 2000\n")
        stockfish_process.stdin.flush()

        # Read Stockfish's output
        output_lines = []
        while True:
            line = stockfish_process.stdout.readline()
            if not line:
                break
            output_lines.append(line.strip())
            if "bestmove" in line:
                break
        
        # Extract the best move
        best_move_line = [line for line in output_lines if "bestmove" in line]
        if not best_move_line:
            raise Exception("Stockfish did not return a bestmove.")
            
        best_move = best_move_line[0].split(" ")[1].strip()
        
        return best_move, output_lines

    except FileNotFoundError:
        return "Stockfish not found. Make sure it's in your PATH or installed correctly."
    except Exception as e:
        return f"An error occurred: {e}"
    finally:
        if 'stockfish_process' in locals() and stockfish_process.poll() is None:
            stockfish_process.kill()


if __name__ == "__main__":
    # Example 1: Starting position
    start_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
    print("--- Test Case 1: Standard Opening ---")
    print(f"Getting move for FEN: {start_fen}")
    move, output = get_stockfish_move(start_fen)
    print("\nFull Stockfish Output:")
    for line in output:
        print(line)
    print(f"\nStockfish's best move: {move}")
    print("-" * 30)

    # Example 2: A different position
    fen_2 = "r1bqkbnr/pp1ppppp/2n5/2p5/4P3/5N2/PPPP1PPP/RNBQKB1R w KQkq - 2 3"
    print("\n--- Test Case 2: Sicilian Defense ---")
    print(f"Getting move for FEN: {fen_2}")
    move, output = get_stockfish_move(fen_2)
    print("\nFull Stockfish Output:")
    for line in output:
        print(line)
    print(f"\nStockfish's best move: {move}")
    print("-" * 30)
