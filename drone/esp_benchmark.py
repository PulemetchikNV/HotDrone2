import argparse
import json
import os
import time
import uuid
from typing import Optional

# rospy fallback
try:
    import rospy
except ImportError:
    class MockRospy:
        def is_shutdown(self):
            return False
        def init_node(self, name):
            pass
        def sleep(self, t):
            time.sleep(t)
    rospy = MockRospy()

try:
    from .stage1_mod import Stage1Mod
    from .helpers import setup_logging
except ImportError:
    from stage1_mod import Stage1Mod
    from helpers import setup_logging


class ESPBenchmark:
    def __init__(self, is_sender: bool, target: str, reliable: bool, count: int, size_bytes: int,
                 timeout: float, retries: int, interval: float, out_path: Optional[str]):
        self.is_sender = is_sender
        self.target = target
        self.reliable = reliable
        self.count = count
        self.size_bytes = size_bytes
        self.timeout = timeout
        self.retries = retries
        self.interval = interval
        self.out_path = out_path

        self.drone_name = os.environ.get("DRONE_NAME", "drone")
        self.logger = setup_logging(self.drone_name)

        self.mod = Stage1Mod()

        # Переопределим callback, чтобы считать входящие пакеты, и затем делегировать штатной логике ack
        self._rx_total = 0
        self._rx_last_ts = None
        self._orig_cb = None

        if self.mod.swarm:
            # Сохраним базовый обработчик (если был) и обернём
            self._orig_cb = self.mod.swarm._custom_message_callback if hasattr(self.mod.swarm, "_custom_message_callback") else None
            def wrapped_cb(message: str):
                self._rx_total += 1
                self._rx_last_ts = time.time()
                try:
                    # Делегируем стандартной обработке для ack
                    self.mod._on_custom_message(message)
                except Exception:
                    pass
                # Дополнительно вызовем исходный cb, если он был
                if self._orig_cb:
                    try:
                        self._orig_cb(message)
                    except Exception:
                        pass
            self.mod.swarm.set_custom_message_callback(wrapped_cb)
            started = self.mod.swarm.start()
            if not started:
                self.logger.warning("Skyros link not started; benchmark will not work")
        else:
            self.logger.warning("Skyros not available; benchmark will not work")

    def _generate_payload(self) -> dict:
        # Генерируем случайные байты указанного размера и кодируем в base64 для JSON
        import base64, os as _os
        raw = _os.urandom(max(0, self.size_bytes))
        blob_b64 = base64.b64encode(raw).decode("ascii")
        return {
            "type": "bench",
            "to": self.target,
            "ts": time.time(),
            "size": self.size_bytes,
            "blob_b64": blob_b64,
        }

    def _send_once_unreliable(self, payload: dict, wait_ack_timeout: float) -> tuple[bool, float, str]:
        if not self.mod.swarm:
            return False, 0.0, "no_swarm"

        msg_id = uuid.uuid4().hex[:6]
        payload = dict(payload)
        payload["msg_id"] = msg_id

        # Очистим возможный старый ack
        with self.mod._ack_lock:
            if msg_id in self.mod._received_acks:
                self.mod._received_acks.remove(msg_id)

        s = json.dumps(payload)
        t0 = time.time()
        try:
            self.mod.swarm.broadcast_custom_message(s)
        except Exception as e:
            return False, 0.0, f"tx_error:{e}"

        # Ждём ack без повторов
        deadline = t0 + wait_ack_timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self.mod._ack_lock:
                if msg_id in self.mod._received_acks:
                    dt = time.time() - t0
                    return True, dt, "ok"
            time.sleep(0.01)
        return False, time.time() - t0, "timeout"

    def _send_once_reliable(self, payload: dict, retries: int, timeout: float) -> tuple[bool, float, str]:
        if not self.mod.swarm:
            return False, 0.0, "no_swarm"
        t0 = time.time()
        ok = self.mod._broadcast_reliable(dict(payload), retries=retries, timeout=timeout)
        return ok, time.time() - t0, ("ok" if ok else "failed")

    def run_sender(self):
        success = 0
        latencies = []

        for i in range(1, self.count + 1):
            payload = self._generate_payload()
            if self.reliable:
                ok, dt, reason = self._send_once_reliable(payload, retries=self.retries, timeout=self.timeout)
            else:
                ok, dt, reason = self._send_once_unreliable(payload, wait_ack_timeout=self.timeout)

            if ok:
                success += 1
                latencies.append(dt)
                self.logger.info(f"[{i}/{self.count}] sent ✓ ({'reliable' if self.reliable else 'unreliable'}) {dt*1000:.1f} ms")
            else:
                self.logger.warning(f"[{i}/{self.count}] sent ✗ ({'reliable' if self.reliable else 'unreliable'}) reason={reason}")

            if self.interval > 0:
                time.sleep(self.interval)

        total = self.count
        loss = total - success
        avg_ms = (sum(latencies) / len(latencies) * 1000.0) if latencies else 0.0
        p50 = sorted(latencies)[len(latencies)//2]*1000.0 if latencies else 0.0
        p95 = sorted(latencies)[max(0, int(len(latencies)*0.95)-1)]*1000.0 if latencies else 0.0

        self.logger.info(f"Result: success={success}/{total}, loss={loss}, avg={avg_ms:.1f} ms, p50={p50:.1f} ms, p95={p95:.1f} ms")

        if self.out_path:
            try:
                with open(self.out_path, "w") as f:
                    f.write("index,ok,latency_ms\n")
                    for idx in range(total):
                        ok_flag = 1 if idx < success else 0
                        lat = latencies[idx]*1000.0 if idx < len(latencies) else 0.0
                        f.write(f"{idx+1},{ok_flag},{lat:.3f}\n")
                self.logger.info(f"Saved CSV: {self.out_path}")
            except Exception as e:
                self.logger.warning(f"Failed to save CSV: {e}")

    def run_receiver(self):
        self.logger.info("Receiver started. Waiting for broadcast messages… Press Ctrl+C to stop.")
        while not rospy.is_shutdown():
            time.sleep(1.0)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ESP broadcast benchmark (using Stage1Mod swarm)")
    p.add_argument("mode", choices=["sender", "receiver"], help="Run as sender or receiver")
    p.add_argument("--target", default="*", help="Target drone name for messages (default: * broadcast)")
    p.add_argument("--reliable", action="store_true", help="Use reliable broadcast with retries")
    p.add_argument("--count", type=int, default=20, help="Number of messages to send in sender mode")
    p.add_argument("--size", type=int, default=64, help="Payload size in bytes (approx)")
    p.add_argument("--timeout", type=float, default=0.5, help="Ack timeout per attempt (seconds)")
    p.add_argument("--retries", type=int, default=3, help="Retries for reliable mode")
    p.add_argument("--interval", type=float, default=0.1, help="Interval between sends (seconds)")
    p.add_argument("--out", default="", help="Optional path to save CSV results (sender only)")
    return p.parse_args()


if __name__ == "__main__":
    try:
        rospy.init_node("esp_benchmark")
    except Exception:
        pass

    args = parse_args()
    bench = ESPBenchmark(
        is_sender=(args.mode == "sender"),
        target=args.target,
        reliable=args.reliable,
        count=args.count,
        size_bytes=args.size,
        timeout=args.timeout,
        retries=args.retries,
        interval=args.interval,
        out_path=(args.out or None),
    )

    if args.mode == "sender":
        bench.run_sender()
    else:
        bench.run_receiver()


