import os
import time
import platform
from dataclasses import dataclass
from typing import Optional, Dict, Any, Union, List
import subprocess
import shutil
import re

# Для Python 3.7 совместимости
try:
    from typing import Literal
except ImportError:
    try:
        from typing_extensions import Literal
    except ImportError:
        def Literal(*args):
            return str

# Stockfish integration
try:
    from stockfish import Stockfish, StockfishException
    STOCKFISH_AVAILABLE = True
except ImportError:
    Stockfish = None
    StockfishException = Exception
    STOCKFISH_AVAILABLE = False

from camera import create_camera_controller, CameraTemporaryError, CameraPermanentError


# Импорт общих типов и исключений
from utils import BoardState, MoveDecision, AlgTemporaryError, AlgPermanentError


# -----------------------------
# Stockfish Engine Manager
# -----------------------------
class StockfishManager:
    """Manages Stockfish chess engine with automatic binary detection and fallback."""
    
    def __init__(self, depth: int = 20, skill_level: int = 20):
        self.depth = depth
        self.skill_level = skill_level
        self.engine = None
        self._initialize_engine()
    
    def _find_stockfish_binary(self) -> Optional[str]:
        """Find Stockfish binary with architecture detection."""
        # Check multiple possible paths
        possible_paths = [
            # From project root
            "./drone/chess/stockfish/stockfish",
            "./drone/chess/stockfish/stockfish-ubuntu-x86-64-avx2",
            "./drone/chess/stockfish/stockfish-android-armv8",
            "./drone/chess/stockfish/stockfish-android-armv7",
            # From drone directory
            "./chess/stockfish/stockfish",
            "./chess/stockfish/stockfish-ubuntu-x86-64-avx2",
            "./chess/stockfish/stockfish-android-armv8",
            "./chess/stockfish/stockfish-android-armv7",
            # System paths
            "/usr/bin/stockfish",
            "/usr/games/stockfish",
            "/usr/local/bin/stockfish"
        ]
        
        for path in possible_paths:
            if os.path.exists(path) and os.access(path, os.X_OK):
                return path
        
        return None
    
    def _initialize_engine(self):
        """Initialize Stockfish engine with error handling."""
        if not STOCKFISH_AVAILABLE:
            raise AlgPermanentError("Stockfish Python package not available")
        
        binary_path = self._find_stockfish_binary()
        if not binary_path:
            raise AlgPermanentError("Stockfish binary not found")
        
        try:
            # Initialize with optimized parameters
            self.engine = Stockfish(
                path=binary_path,
                depth=self.depth,
                parameters={
                    "Threads": min(2, os.cpu_count() or 1),
                    "Hash": 128,  # 128MB hash table
                    "Skill Level": self.skill_level,
                    "Minimum Thinking Time": 50,
                    "UCI_LimitStrength": "false"
                }
            )
            
            # Test the engine
            self.engine.get_fen_position()
            print(f"Stockfish engine initialized successfully: {binary_path}")
            
        except (StockfishException, Exception) as e:
            raise AlgPermanentError(f"Failed to initialize Stockfish engine: {e}")
    
    def get_best_move(self, fen: str, time_limit_ms: int = 5000) -> Dict[str, Any]:
        """Get best move from Stockfish engine."""
        if not self.engine:
            raise AlgPermanentError("Stockfish engine not initialized")
        
        try:
            # Set position
            if not self.engine.is_fen_valid(fen):
                raise AlgPermanentError(f"Invalid FEN position: {fen}")
            
            self.engine.set_fen_position(fen)
            
            # Get best move with time constraint
            best_move = self.engine.get_best_move_time(time_limit_ms)
            if not best_move:
                raise AlgPermanentError("Failed to get best move from Stockfish")
            
            # Get evaluation
            evaluation = self.engine.get_evaluation()
            
            # Get top moves for additional info
            top_moves = self.engine.get_top_moves(3)
            
            return {
                "move": best_move,
                "evaluation": evaluation,
                "top_moves": top_moves,
                "fen": fen
            }
            
        except (StockfishException, Exception) as e:
            raise AlgPermanentError(f"Stockfish error: {e}")
    
    def is_available(self) -> bool:
        """Check if Stockfish engine is available and working."""
        return self.engine is not None


# -----------------------------
# MPI/Cluster helpers (optional)
# -----------------------------
def _read_cluster_hosts(path: str) -> List[str]:
    try:
        with open(path, 'r') as f:
            return [l.strip() for l in f if l.strip() and not l.strip().startswith('#')]
    except Exception:
        return []


def _find_stockfish_binary_for_cluster() -> Optional[str]:
    possible_paths = [
        "./drone/chess/stockfish/stockfish",
        "./drone/chess/stockfish/stockfish-ubuntu-x86-64-avx2",
        "./drone/chess/stockfish/stockfish-android-armv8",
        "./drone/chess/stockfish/stockfish-android-armv7",
        "./chess/stockfish/stockfish",
        "./chess/stockfish/stockfish-ubuntu-x86-64-avx2",
        "./chess/stockfish/stockfish-android-armv8",
        "./chess/stockfish/stockfish-android-armv7",
        "/usr/bin/stockfish",
        "/usr/games/stockfish",
        "/usr/local/bin/stockfish",
    ]
    for path in possible_paths:
        if os.path.exists(path) and os.access(path, os.X_OK):
            return path
    return None


def _cluster_analyze_position(fen: str, movetime_ms: int, turn: str) -> Optional[Dict[str, Any]]:
    hostfile = os.getenv("CLUSTER_HOSTFILE", "cluster_hosts")
    hosts = _read_cluster_hosts(hostfile)
    try:
        np = int(os.getenv("CLUSTER_NP", "0")) or len(hosts)
    except Exception:
        np = len(hosts)

    if np <= 0 or not hosts:
        return None

    mpirun = shutil.which("mpirun")
    if not mpirun:
        return None

    binary = _find_stockfish_binary_for_cluster()
    if not binary:
        return None

    # Один поток на процесс для честного распределения
    uci_script = (
        "uci\n"
        "isready\n"
        "ucinewgame\n"
        "setoption name Threads value 1\n"
        f"position fen {fen}\n"
        f"go movetime {movetime_ms}\n"
        "quit\n"
    )

    cmd = [
        mpirun, "--hostfile", hostfile, "-map-by", "node",
        "-np", str(np), "--tag-output", binary
    ]

    try:
        timeout_s = max(10, movetime_ms // 1000 + 10)
        res = subprocess.run(
            cmd,
            input=uci_script,
            capture_output=True,
            text=True,
            timeout=timeout_s
        )
        out = res.stdout or ""
    except Exception:
        return None

    info_score_re = re.compile(r"score (cp|mate) (-?\d+)")
    bestmove_re = re.compile(r"bestmove\s+([a-h][1-8][a-h][1-8][qrbn]?)")

    results: List[Dict[str, Any]] = []
    last_score: Optional[Dict[str, Any]] = None
    for line in out.splitlines():
        m1 = info_score_re.search(line)
        if m1:
            score_type, score_val = m1.group(1), int(m1.group(2))
            last_score = {"type": score_type, "value": score_val}
            continue
        m2 = bestmove_re.search(line)
        if m2:
            mv = m2.group(1)
            results.append({"move": mv, "score": last_score})
            last_score = None

    if not results:
        return None

    def side_score(s: Optional[Dict[str, Any]]) -> int:
        if s is None:
            return -10 ** 12
        if s["type"] == "cp":
            val = s["value"]
            return val if turn == "w" else -val
        # type == mate: положительное — мат за нас; чем меньше по модулю, тем быстрее
        sign = 1 if s["value"] > 0 else -1
        win = (turn == "w" and sign > 0) or (turn == "b" and sign < 0)
        base = 10 ** 9 if win else -(10 ** 9)
        return base - abs(s["value"])

    best = max(results, key=lambda r: side_score(r.get("score")))
    return best

# -----------------------------
# Источник данных камеры
# -----------------------------
FILES = "abcdefgh"
RANKS = "12345678"

_camera_controller = None
_stockfish_manager = None


def _get_camera_controller():
    global _camera_controller
    if _camera_controller is None:
        _camera_controller = create_camera_controller()
    return _camera_controller


def _get_stockfish_manager():
    global _stockfish_manager
    if _stockfish_manager is None:
        depth = int(os.getenv("STOCKFISH_DEPTH", "20"))
        skill_level = int(os.getenv("STOCKFISH_SKILL_LEVEL", "20"))
        _stockfish_manager = StockfishManager(depth=depth, skill_level=skill_level)
    return _stockfish_manager


def _normalize_cell(cell: str) -> str:
    if not isinstance(cell, str) or len(cell) != 2:
        raise AlgPermanentError("cell must be like 'e2'")
    f, r = cell[0].lower(), cell[1]
    if f not in FILES or r not in RANKS:
        raise AlgPermanentError(f"invalid cell: {cell}")
    return f + r


def _camera_read_board() -> Dict[str, Any]:
    try:
        camera = _get_camera_controller()
        positions = camera.get_board_positions()
        board_info = {
            'positions': positions,
            'timestamp': time.time(),
        }
        # current_cell: выбираем первую фигуру для детерминизма
        current_cell = os.getenv("START_CELL", "e2")
        for color in ("white", "black"):
            if color in positions and isinstance(positions[color], dict):
                for piece in positions[color].values():
                    current_cell = piece.cell
                    break
                if current_cell != os.getenv("START_CELL", "e2"):
                    break
        board_info['current_cell'] = _normalize_cell(current_cell)
        # чей ход — если контроллер умеет, заберём; иначе white
        turn = getattr(camera, 'get_turn', lambda: None)() or 'white'
        board_info['turn'] = 'w' if turn.lower().startswith('w') else 'b'
        return board_info
    except CameraTemporaryError as e:
        raise AlgTemporaryError(f"Camera temporary error: {e}")
    except CameraPermanentError as e:
        raise AlgPermanentError(f"Camera permanent error: {e}")
    except Exception as e:
        raise AlgTemporaryError(f"Unexpected camera error: {e}")


def get_board_state() -> BoardState:
    """Считывает текущее состояние с камеры и возвращает BoardState."""
    board_info = _camera_read_board()
    # Пока fen упрощённый — берём из позиций через мок-сервер, либо дефолт
    fen = os.getenv("DEFAULT_FEN", "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")
    meta = {
        "current_cell": board_info['current_cell'],
        "positions": board_info['positions'],
        "camera_timestamp": board_info['timestamp']
    }
    return BoardState(
        fen=fen,
        turn=board_info['turn'],
        move_number=1,
        timestamp=time.time(),
        meta=meta,
    )


def _generate_stockfish_move(board: BoardState, time_budget_ms: int = 5000) -> MoveDecision:
    """Generate a move using Stockfish engine."""
    # Кластерный режим (опционально). Если не сработал — фолбэк на локальный движок
    if os.getenv("ALG_MODE", "api").lower() == "cluster":
        best = _cluster_analyze_position(board.fen, time_budget_ms, board.turn)
        if best and best.get("move"):
            move = best["move"]
            # Parse UCI move
            if len(move) < 4:
                raise AlgPermanentError(f"Invalid UCI move format: {move}")
            from_cell = move[:2]
            to_cell = move[2:4]

            score_cp = None
            is_mate = False
            score = best.get("score")
            if score:
                if score.get("type") == "cp":
                    score_cp = score.get("value")
                elif score.get("type") == "mate":
                    is_mate = True
                    score_val = score.get("value", 0)
                    score_cp = 10000 if score_val > 0 else -10000

            return MoveDecision(
                uci=move,
                from_cell=from_cell,
                to_cell=to_cell,
                score_cp=score_cp,
                is_mate=is_mate,
                reason="stockfish-mpi",
                meta={
                    "policy": "stockfish-mpi",
                    "np": os.getenv("CLUSTER_NP", None),
                    "hostfile": os.getenv("CLUSTER_HOSTFILE", "cluster_hosts"),
                },
            )

    # Фолбэк: локальный движок
    stockfish = _get_stockfish_manager()

    # Get best move from Stockfish
    result = stockfish.get_best_move(board.fen, time_budget_ms)

    move = result["move"]
    evaluation = result["evaluation"]
    top_moves = result["top_moves"]

    # Parse UCI move
    if len(move) != 4:
        raise AlgPermanentError(f"Invalid UCI move format: {move}")

    from_cell = move[:2]
    to_cell = move[2:4]

    # Extract score
    score_cp = None
    is_mate = False

    if evaluation["type"] == "cp":
        score_cp = evaluation["value"]
    elif evaluation["type"] == "mate":
        is_mate = True
        score_cp = 10000 if evaluation["value"] > 0 else -10000

    # Find move in top moves for additional info
    move_info = None
    for top_move in top_moves:
        if top_move.get("Move") == move:
            move_info = top_move
            break

    # Override score if available in top moves
    if move_info:
        if move_info.get("Mate") is not None:
            is_mate = True
            score_cp = 10000 if move_info["Mate"] > 0 else -10000
        elif move_info.get("Centipawn") is not None:
            score_cp = move_info["Centipawn"]

    return MoveDecision(
        uci=move,
        from_cell=from_cell,
        to_cell=to_cell,
        score_cp=score_cp,
        is_mate=is_mate,
        reason="stockfish-engine",
        meta={
            "policy": "stockfish",
            "evaluation": evaluation,
            "top_moves": top_moves,
            "depth": stockfish.depth,
            "skill_level": stockfish.skill_level
        },
    )


def get_turn(board: BoardState, time_budget_ms: int = 5000, seed: Optional[int] = None) -> MoveDecision:
    """Возвращает следующий ход используя Stockfish."""
    if board is None or not isinstance(board, BoardState):
        raise AlgPermanentError("board must be BoardState")
    
    return _generate_stockfish_move(board, time_budget_ms)


def update_after_execution(prev: BoardState, move: MoveDecision, success: bool) -> BoardState:
    """Обновляет предсказанное состояние после попытки исполнения хода."""
    if prev is None or move is None:
        raise AlgPermanentError("prev and move are required")
    if not isinstance(prev, BoardState) or not isinstance(move, MoveDecision):
        raise AlgPermanentError("prev must be BoardState and move must be MoveDecision")
    
    if not success:
        return BoardState(
            fen=prev.fen,
            turn=prev.turn,
            move_number=prev.move_number,
            timestamp=time.time(),
            meta=prev.meta,
        )
    
    meta = dict(prev.meta or {})
    meta["current_cell"] = _normalize_cell(move.to_cell)
    next_turn: str = "b" if prev.turn == "w" else "w"
    return BoardState(
        fen=prev.fen,
        turn=next_turn,
        move_number=max(1, prev.move_number) + 1,
        timestamp=time.time(),
        meta=meta,
    )


# -----------------------------
# Utility functions
# -----------------------------
def get_stockfish_status() -> Dict[str, Any]:
    """Get status information about Stockfish integration."""
    try:
        stockfish = _get_stockfish_manager()
        return {
            "stockfish_available": STOCKFISH_AVAILABLE,
            "engine_available": stockfish.is_available(),
            "depth": stockfish.depth if stockfish.engine else None,
            "skill_level": stockfish.skill_level if stockfish.engine else None,
            "use_stockfish": True
        }
    except Exception as e:
        return {
            "stockfish_available": STOCKFISH_AVAILABLE,
            "engine_available": False,
            "error": str(e),
            "use_stockfish": True
        }


def test_stockfish_integration() -> Dict[str, Any]:
    """Test Stockfish integration with a simple position."""
    try:
        stockfish = _get_stockfish_manager()
        
        # Test with starting position
        test_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        result = stockfish.get_best_move(test_fen, 1000)
        
        return {
            "status": "success",
            "test_fen": test_fen,
            "best_move": result["move"],
            "evaluation": result["evaluation"]
        }
            
    except Exception as e:
        return {"status": "error", "message": str(e)}
