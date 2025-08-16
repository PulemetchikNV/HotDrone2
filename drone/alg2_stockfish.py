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

# Импорт общих типов и исключений
from utils import BoardState, MoveDecision, AlgTemporaryError, AlgPermanentError
from board_utils import get_board_state


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
        """Find Stockfish binary with preference for system PATH."""
        # Check system PATH first (preferred for cluster mode)
        system_paths = [
            "/usr/local/bin/stockfish",
            "/usr/bin/stockfish", 
            "/usr/games/stockfish"
        ]
        
        # Also check if stockfish is available via which command
        try:
            result = subprocess.run(['which', 'stockfish'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0 and result.stdout.strip():
                stockfish_path = result.stdout.strip()
                if os.access(stockfish_path, os.X_OK):
                    return stockfish_path
        except Exception:
            pass
        
        # Check explicit system paths
        for path in system_paths:
            if os.path.exists(path) and os.access(path, os.X_OK):
                return path
        
        # Fallback to local project paths (for development)
        local_paths = [
            # From project root
            "./drone/chess/stockfish/stockfish",
            "./drone/chess/stockfish/stockfish-ubuntu-x86-64-avx2",
            "./drone/chess/stockfish/stockfish-android-armv8",
            "./drone/chess/stockfish/stockfish-android-armv7",
            # From drone directory
            "./chess/stockfish/stockfish",
            "./chess/stockfish/stockfish-ubuntu-x86-64-avx2",
            "./chess/stockfish/stockfish-android-armv8",
            "./chess/stockfish/stockfish-android-armv7"
        ]
        
        for path in local_paths:
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
            
        except ValueError as e:
            if "invalid literal for int()" in str(e) and ("dev" in str(e) or "b4ac3d6b" in str(e)):
                # Проблема с парсингом версии dev-сборки Stockfish
                print(f"Warning: Stockfish dev version detected, trying alternative initialization")
                try:
                    # Пытаемся инициализировать без проверки версии через прямой UCI
                    self.engine = self._create_direct_uci_stockfish_wrapper(binary_path)
                    print(f"Stockfish dev version initialized with UCI wrapper: {binary_path}")
                    return
                except Exception as wrapper_error:
                    print(f"UCI wrapper also failed: {wrapper_error}")
                    pass
                
            raise AlgPermanentError(f"Failed to initialize Stockfish engine (version issue): {e}")
            
        except (StockfishException, Exception) as e:
            raise AlgPermanentError(f"Failed to initialize Stockfish engine: {e}")
    
    def _create_direct_uci_stockfish_wrapper(self, binary_path):
        """Create a direct UCI wrapper for dev Stockfish versions that have parsing issues."""
        class DirectUCIStockfishWrapper:
            def __init__(self, path, depth=20, skill_level=20):
                self.path = path
                self.depth = depth
                self.skill_level = skill_level
                self._process = None
                self._initialize_process()
                
            def _initialize_process(self):
                """Initialize the Stockfish process with direct UCI communication."""
                import subprocess
                self._process = subprocess.Popen(
                    self.path,
                    universal_newlines=True,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
                
                # Send UCI initialization without version parsing
                self._send_command("uci")
                self._wait_for_response("uciok")
                
                # Set parameters
                self._send_command(f"setoption name Threads value {min(2, os.cpu_count() or 1)}")
                self._send_command(f"setoption name Hash value 128")
                self._send_command(f"setoption name Skill Level value {self.skill_level}")
                self._send_command("setoption name UCI_LimitStrength value false")
                self._send_command("isready")
                self._wait_for_response("readyok")
                
            def _send_command(self, command):
                """Send a UCI command to Stockfish."""
                if self._process and self._process.stdin:
                    self._process.stdin.write(f"{command}\n")
                    self._process.stdin.flush()
                    
            def _read_line(self):
                """Read a line from Stockfish output."""
                if self._process and self._process.stdout:
                    return self._process.stdout.readline().strip()
                return ""
                
            def _wait_for_response(self, expected):
                """Wait for a specific response from Stockfish."""
                while True:
                    line = self._read_line()
                    if expected in line:
                        break
                        
            def get_fen_position(self):
                """Return default starting position (test method)."""
                return "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
                
            def is_fen_valid(self, fen):
                """Simple FEN validation."""
                parts = fen.split()
                return len(parts) >= 4
                
            def set_fen_position(self, fen):
                """Set position using UCI."""
                self._send_command("ucinewgame")
                self._send_command("isready")
                self._wait_for_response("readyok")
                self._send_command(f"position fen {fen}")
                
            def get_best_move_time(self, time_ms):
                """Get best move with time limit."""
                self._send_command(f"go movetime {time_ms}")
                
                while True:
                    line = self._read_line()
                    if line.startswith("bestmove"):
                        parts = line.split()
                        if len(parts) >= 2 and parts[1] != "(none)":
                            return parts[1]
                        return None
                        
            def get_evaluation(self):
                """Get position evaluation."""
                self._send_command(f"go depth {min(10, self.depth)}")
                
                evaluation = {"type": "cp", "value": 0}
                while True:
                    line = self._read_line()
                    if line.startswith("bestmove"):
                        break
                    elif "score cp" in line:
                        parts = line.split()
                        if "cp" in parts:
                            cp_index = parts.index("cp")
                            if cp_index + 1 < len(parts):
                                try:
                                    evaluation = {"type": "cp", "value": int(parts[cp_index + 1])}
                                except ValueError:
                                    pass
                    elif "score mate" in line:
                        parts = line.split()
                        if "mate" in parts:
                            mate_index = parts.index("mate")
                            if mate_index + 1 < len(parts):
                                try:
                                    evaluation = {"type": "mate", "value": int(parts[mate_index + 1])}
                                except ValueError:
                                    pass
                                    
                return evaluation
                
            def get_top_moves(self, count):
                """Get top moves (simplified)."""
                best_move = self.get_best_move_time(1000)
                if best_move:
                    return [{"Move": best_move, "Centipawn": 0}]
                return []
                
            def __del__(self):
                """Clean up the process."""
                if self._process:
                    try:
                        self._send_command("quit")
                        self._process.wait(timeout=2)
                    except Exception:
                        pass
                    finally:
                        if self._process.poll() is None:
                            self._process.terminate()
        
        return DirectUCIStockfishWrapper(binary_path, self.depth, self.skill_level)
    
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


def _find_stockfish_binary_for_cluster() -> bool:
    """Check if stockfish is available in system PATH for cluster mode."""
    try:
        # Check if stockfish command is available
        result = subprocess.run(['which', 'stockfish'], 
                              capture_output=True, text=True, timeout=5)
        return result.returncode == 0
    except Exception:
        # Fallback: check common system paths
        system_paths = ["/usr/local/bin/stockfish", "/usr/bin/stockfish", "/usr/games/stockfish"]
        return any(os.path.exists(path) and os.access(path, os.X_OK) for path in system_paths)


def _cluster_analyze_position(fen: str, movetime_ms: int, turn: str) -> Optional[Dict[str, Any]]:
    hostfile = os.getenv("CLUSTER_HOSTFILE", "cluster_hosts")
    # Делаем путь абсолютным если он относительный
    if not os.path.isabs(hostfile):
        hostfile = os.path.abspath(hostfile)
    hosts = _read_cluster_hosts(hostfile)
    
    # Debug информация
    cluster_np_env = os.getenv("CLUSTER_NP", "0")
    print(f"==== DEBUG: hostfile={hostfile}, hosts={hosts}, CLUSTER_NP={cluster_np_env}")
    
    # ВАЖНО: всегда используем количество реальных хостов, игнорируем CLUSTER_NP
    # CLUSTER_NP может быть устаревшей или неправильной
    np = len(hosts)
    
    # Оставляем старую логику только для отладки
    try:
        old_np = int(cluster_np_env) or len(hosts)
        if old_np != np:
            print(f"==== DEBUG: CLUSTER_NP={old_np} != len(hosts)={np}, using len(hosts)")
    except Exception:
        pass
    
    print(f"==== DEBUG: final np={np}, len(hosts)={len(hosts)}")

    if np <= 0 or not hosts:
        return None

    mpirun = shutil.which("mpirun")
    if not mpirun:
        return None

    # Проверяем доступность stockfish в PATH
    if not _find_stockfish_binary_for_cluster():
        return None

    print(f"==== HOSTFILE: {open(hostfile, 'r').read()}")

    # Формируем команду MPI с динамическим количеством процессов
    binary_path = f"mpirun --hostfile cluster_hosts -map-by node -np {np} stockfish"

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

    # Используем shell=True для выполнения команды как строки
    cmd = binary_path

    try:
        timeout_s = max(10, movetime_ms // 1000 + 10)
        res = subprocess.run(
            cmd,
            input=uci_script,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            shell=True
        )
        out = res.stdout or ""
        print(f"==== GOT CMD: {cmd}")
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
# Локальные утилиты Stockfish
# -----------------------------
_stockfish_manager = None


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
    if f not in "abcdefgh" or r not in "12345678":
        raise AlgPermanentError(f"invalid cell: {cell}")
    return f + r


def _check_cluster_changes_needed(alive_drones: List[str]) -> bool:
    """
    Проверяет, нужны ли изменения в кластере.
    
    Args:
        alive_drones: Список имен живых дронов
        
    Returns:
        bool: True если нужно обновить кластер
    """
    try:
        from cluster_manager import ClusterManager
        import logging
        
        logger = logging.getLogger(__name__)
        cluster_mgr = ClusterManager(logger)
        
        return cluster_mgr.cluster_composition_changed(alive_drones)
        
    except Exception as e:
        # Если не можем проверить - считаем что изменений нет
        return False


def _generate_stockfish_move(board: BoardState, time_budget_ms: int = 5000) -> MoveDecision:
    """Generate a move using Stockfish engine."""
    # Кластерный режим (опционально). Если не сработал — фолбэк на локальный движок
    if os.getenv("ALG_MODE", "api").lower() == "cluster":
        print(f"==== DEBUG: cluster mode")
        # Проверяем актуальность кластера перед вычислением хода
        alive_drones_str = os.getenv("CLUSTER_ALIVE_DRONES", "")
        if alive_drones_str:
            alive_drones = [d.strip() for d in alive_drones_str.split(",") if d.strip()]
            
            # Проверяем изменения в кластере
            if _check_cluster_changes_needed(alive_drones):
                # Мягкая перезагрузка только stockfish-кластера без убийства процесса
                try:
                    from cluster_manager import ClusterManager
                    import logging
                    logger = logging.getLogger(__name__)
                    cluster_mgr = ClusterManager(logger)
                    restarted = cluster_mgr.restart_stockfish_cluster(alive_drones)
                    if restarted:
                        logger.info("Stockfish cluster restarted in-place successfully")
                    else:
                        logger.warning("Failed to restart stockfish cluster in-place; continuing")
                except Exception as e:
                    print(f"Soft stockfish cluster restart failed: {e}")
        
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
    print(f"==== DEBUG: fallback to local stockfish")
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
