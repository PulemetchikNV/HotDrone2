import argparse
import json
import os
import time
import uuid
import threading
from typing import Optional, Tuple

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
    from .helpers import setup_logging
except ImportError:
    from helpers import setup_logging

# skyros optional import
try:
    from skyros.drone import Drone as SkyrosDrone
except Exception:
    SkyrosDrone = None


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

        # Собственная реализация skyros без stage1_mod
        self.swarm = None
        if SkyrosDrone is not None:
            try:
                self.swarm = SkyrosDrone(name=self.drone_name)
            except Exception as e:
                self.logger.warning(f"Skyros init failed: {e}")
                self.swarm = None

        # Для надежной отправки
        self._received_acks = set()
        self._ack_lock = threading.Lock()
        
        # Переопределим callback, чтобы считать входящие пакеты, и затем делегировать штатной логике ack
        self._rx_total = 0
        self._rx_last_ts = None
        
        # Для сборки фрагментированных сообщений
        self._fragments = {}  # {msg_uuid: {total: int, received: {frag_id: data}}}
        self._fragment_timeout = 10.0  # секунд на сборку всех фрагментов

        if self.swarm:
            def message_callback(message: str):
                self._rx_total += 1
                self._rx_last_ts = time.time()
                
                try:
                    msg_obj = json.loads(message)
                    
                    # Обрабатываем ACK сообщения
                    if msg_obj.get('type') == 'ack':
                        ack_id = msg_obj.get('ack_id')
                        if ack_id:
                            with self._ack_lock:
                                self._received_acks.add(ack_id)
                        return
                    
                    # Обрабатываем фрагментированные сообщения (новый и старый формат)
                    if msg_obj.get('fragmented') or msg_obj.get('f'):
                        self._handle_fragment(msg_obj)
                        return  # Не передаем фрагменты дальше
                    
                    # Отправляем ACK если есть msg_id и сообщение для нас
                    target = msg_obj.get('to', '*')
                    if 'msg_id' in msg_obj and (target == '*' or target == self.drone_name):
                        ack_payload = {
                            'type': 'ack',
                            'ack_id': msg_obj['msg_id'],
                            'from': self.drone_name
                        }
                        try:
                            self.swarm.broadcast_custom_message(json.dumps(ack_payload))
                        except Exception as e:
                            self.logger.warning(f"Failed to send ack: {e}")
                            
                except Exception as e:
                    self.logger.debug(f"Message parsing error: {e}")
            
            self.swarm.set_custom_message_callback(message_callback)
            started = self.swarm.start()
            if not started:
                self.logger.warning("Skyros link not started; benchmark will not work")
        else:
            self.logger.warning("Skyros not available; benchmark will not work")

    def _handle_fragment(self, fragment_obj: dict):
        """Обработка фрагментированного сообщения (компактный формат)"""
        try:
            # Поддерживаем и старый и новый формат
            if 'f' in fragment_obj:  # Новый компактный формат
                msg_uuid = fragment_obj['u']  # uuid
                frag_id = fragment_obj['i']   # frag_id
                total_frags = fragment_obj['t']  # total_frags
                data = fragment_obj['d']      # data
                original_msg_id = fragment_obj.get('m', '')  # original_msg_id
            else:  # Старый формат
                msg_uuid = fragment_obj['msg_uuid']
                frag_id = fragment_obj['frag_id']
                total_frags = fragment_obj['total_frags']
                data = fragment_obj['data']
                original_msg_id = fragment_obj.get('original_msg_id', '')
            
            # Инициализируем структуру для сборки если нужно
            if msg_uuid not in self._fragments:
                self._fragments[msg_uuid] = {
                    'total': total_frags,
                    'received': {},
                    'timestamp': time.time(),
                    'original_msg_id': original_msg_id
                }
            
            # Добавляем фрагмент
            self._fragments[msg_uuid]['received'][frag_id] = data
            
            self.logger.debug(f"Fragment {frag_id+1}/{total_frags} received for {msg_uuid}")
            
            # Проверяем, все ли фрагменты получены
            if len(self._fragments[msg_uuid]['received']) == total_frags:
                # Собираем сообщение
                assembled_data = ""
                for i in range(total_frags):
                    if i in self._fragments[msg_uuid]['received']:
                        assembled_data += self._fragments[msg_uuid]['received'][i]
                
                self.logger.info(f"Message {msg_uuid} assembled from {total_frags} fragments ({len(assembled_data)} chars)")
                
                # Парсим собранное сообщение
                try:
                    assembled_msg = json.loads(assembled_data)
                    # Отправляем ACK для собранного сообщения
                    if 'msg_id' in assembled_msg:
                        self._send_ack(assembled_msg['msg_id'])
                    
                    # Обрабатываем собранное сообщение
                    self._rx_total += 1
                    
                except json.JSONDecodeError as e:
                    self.logger.error(f"Failed to parse assembled message: {e}")
                
                # Удаляем из буфера
                del self._fragments[msg_uuid]
            
        except Exception as e:
            self.logger.warning(f"Fragment handling error: {e}")
    
    def _send_ack(self, msg_id: str):
        """Отправляет ACK для полученного сообщения"""
        ack_payload = {
            'type': 'ack',
            'ack_id': msg_id,
            'from': self.drone_name
        }
        try:
            self.swarm.broadcast_custom_message(json.dumps(ack_payload))
        except Exception as e:
            self.logger.warning(f"Failed to send ack for {msg_id}: {e}")
    
    def _cleanup_old_fragments(self):
        """Очистка старых неполных фрагментов"""
        current_time = time.time()
        to_remove = []
        for msg_uuid, frag_data in self._fragments.items():
            if current_time - frag_data['timestamp'] > self._fragment_timeout:
                to_remove.append(msg_uuid)
        
        for msg_uuid in to_remove:
            self.logger.warning(f"Fragment timeout for {msg_uuid[:8]}, removing incomplete message")
            del self._fragments[msg_uuid]

    def _generate_payload(self) -> dict:
        # Генерируем случайные байты указанного размера и кодируем в base64 для JSON
        import base64, os as _os
        raw = _os.urandom(max(0, self.size_bytes))
        blob_b64 = base64.b64encode(raw).decode("ascii")
        payload = {
            "type": "bench",
            "to": self.target,
            "ts": time.time(),
            "size": self.size_bytes,
            "blob_b64": blob_b64,
        }
        return payload

    def _fragment_message(self, payload: dict, msg_id: str) -> list:
        """Разбивает большое сообщение на фрагменты"""
        payload_json = json.dumps(payload)
        
        if len(payload_json) <= 124:
            # Сообщение помещается в один пакет
            return [payload]
        
        # Нужно фрагментировать - используем компактный формат
        msg_uuid = uuid.uuid4().hex[:6]  # Еще короче UUID
        data_to_split = payload_json
        
        # Минимальный тестовый фрагмент для оценки накладных расходов
        test_fragment = {
            "f": 1,  # fragmented (короткое имя)
            "u": msg_uuid,  # uuid
            "i": 0,  # frag_id  
            "t": 1,  # total_frags
            "d": "",  # data
            "m": msg_id[:6],  # original_msg_id (сокращенный)
            "to": self.target[:6] if len(self.target) > 6 else self.target  # сокращенное имя
        }
        
        # Вычисляем накладные расходы на метаданные
        metadata_size = len(json.dumps(test_fragment))
        max_data_size = 124 - metadata_size - 10  # -10 для безопасности
        
        self.logger.info(f"Metadata size: {metadata_size}, max_data_size: {max_data_size}")
        
        if max_data_size <= 10:
            self.logger.error(f"Cannot fragment: metadata too large ({metadata_size} bytes), max_data_size={max_data_size}")
            # Попробуем отправить урезанное исходное сообщение
            truncated = payload_json[:120]
            truncated_payload = json.loads(truncated) if truncated.endswith('}') else {"error": "truncated"}
            return [truncated_payload]
        
        # Разбиваем данные на куски подходящего размера
        fragments = []
        total_chunks = (len(data_to_split) + max_data_size - 1) // max_data_size
        
        for i in range(total_chunks):
            start = i * max_data_size
            end = min(start + max_data_size, len(data_to_split))
            chunk_data = data_to_split[start:end]
            
            fragment = {
                "f": 1,
                "u": msg_uuid,
                "i": i,
                "t": total_chunks,
                "d": chunk_data,
                "m": msg_id[:6],
                "to": self.target[:6] if len(self.target) > 6 else self.target
            }
            
            # Финальная проверка размера с итеративным урезанием
            while True:
                frag_json = json.dumps(fragment)
                if len(frag_json) <= 124:
                    break
                    
                # Урезаем данные
                if len(chunk_data) > 5:
                    chunk_data = chunk_data[:-5]
                    fragment["d"] = chunk_data
                else:
                    self.logger.warning(f"Skipping fragment {i} - cannot fit in 124 chars")
                    fragment = None
                    break
            
            if fragment:
                fragments.append(fragment)
        
        self.logger.info(f"Message fragmented into {len(fragments)} parts (original: {len(payload_json)} chars)")
        return fragments

    def _broadcast_reliable(self, payload: dict, retries: int = 3, timeout: float = 0.5) -> bool:
        """Собственная реализация надежной отправки с ACK"""
        if not self.swarm:
            return False

        msg_id = uuid.uuid4().hex[:6]
        payload = dict(payload)
        payload['msg_id'] = msg_id

        attempt = 0
        while attempt < retries:
            attempt += 1
            
            # Очищаем старое подтверждение перед отправкой
            with self._ack_lock:
                if msg_id in self._received_acks:
                    self._received_acks.remove(msg_id)

            self.logger.debug(f"Broadcasting (attempt {attempt}/{retries}): msg_id={msg_id}")
            
            try:
                msg = json.dumps(payload)
                if len(msg) > 124:
                    self.logger.warning(f"Message too long ({len(msg)} chars), truncating to 124")
                    msg = msg[:124]
                    
                self.swarm.broadcast_custom_message(msg)
            except Exception as e:
                self.logger.warning(f"Broadcast failed: {e}")
                time.sleep(timeout)
                continue

            # Ждем подтверждения
            deadline = time.time() + timeout
            while time.time() < deadline and not rospy.is_shutdown():
                with self._ack_lock:
                    if msg_id in self._received_acks:
                        self.logger.debug(f"ACK received for msg_id: {msg_id}")
                        return True
                time.sleep(0.01)
            
            self.logger.warning(f"No ACK for {msg_id} (attempt {attempt}/{retries})")

        self.logger.error(f"Broadcast failed after {retries} retries for msg_id: {msg_id}")
        return False

    def _send_once_unreliable(self, payload: dict, wait_ack_timeout: float) -> Tuple[bool, float, str]:
        if not self.swarm:
            return False, 0.0, "no_swarm"

        msg_id = uuid.uuid4().hex[:6]
        payload = dict(payload)
        payload["msg_id"] = msg_id

        # Очистим возможный старый ack
        with self._ack_lock:
            if msg_id in self._received_acks:
                self._received_acks.remove(msg_id)

        # Проверяем размер исходного сообщения
        payload_json = json.dumps(payload)
        
        t0 = time.time()
        
        if len(payload_json) <= 124:
            # Сообщение помещается в один пакет
            try:
                self.swarm.broadcast_custom_message(payload_json)
                self.logger.debug(f"Sent single message ({len(payload_json)} chars)")
            except Exception as e:
                return False, 0.0, f"tx_error:{e}"
        else:
            # Нужно фрагментировать
            fragments = self._fragment_message(payload, msg_id)
            
            # Отправляем все фрагменты
            for i, fragment in enumerate(fragments):
                try:
                    frag_json = json.dumps(fragment)
                    self.swarm.broadcast_custom_message(frag_json)
                    self.logger.debug(f"Sent fragment {i+1}/{len(fragments)} ({len(frag_json)} chars)")
                    
                    # Небольшая задержка между фрагментами
                    if i < len(fragments) - 1:
                        time.sleep(0.01)
                        
                except Exception as e:
                    return False, 0.0, f"tx_error_frag_{i}:{e}"

        # Ждём ack для основного сообщения
        deadline = t0 + wait_ack_timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self._ack_lock:
                if msg_id in self._received_acks:
                    dt = time.time() - t0
                    return True, dt, "ok"
            time.sleep(0.01)
        return False, time.time() - t0, "timeout"

    def _send_once_reliable(self, payload: dict, retries: int, timeout: float) -> Tuple[bool, float, str]:
        if not self.swarm:
            return False, 0.0, "no_swarm"
        
        # Добавляем msg_id к payload
        msg_id = uuid.uuid4().hex[:6]
        payload = dict(payload)
        payload["msg_id"] = msg_id
        
        # Проверяем размер исходного сообщения
        payload_json = json.dumps(payload)
        
        t0 = time.time()
        
        if len(payload_json) <= 124:
            # Одно сообщение, используем собственную reliable логику
            ok = self._broadcast_reliable(payload, retries=retries, timeout=timeout)
            return ok, time.time() - t0, ("ok" if ok else "failed")
        else:
            # Нужно фрагментировать
            fragments = self._fragment_message(payload, msg_id)
            
            if not fragments:
                return False, time.time() - t0, "no_fragments"
            
            # Множественные фрагменты - отправляем каждый reliable
            success_count = 0
            for i, fragment in enumerate(fragments):
                ok = self._broadcast_reliable(fragment, retries=retries, timeout=timeout/len(fragments))
                if ok:
                    success_count += 1
                    self.logger.debug(f"Reliable fragment {i+1}/{len(fragments)} sent successfully")
                else:
                    self.logger.warning(f"Reliable fragment {i+1}/{len(fragments)} failed")
            
            # Считаем успешным если все фрагменты отправлены
            all_success = (success_count == len(fragments))
            return all_success, time.time() - t0, (f"ok_{success_count}/{len(fragments)}" if all_success else f"partial_{success_count}/{len(fragments)}")

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
        last_cleanup = time.time()
        
        while not rospy.is_shutdown():
            time.sleep(1.0)
            
            # Периодически очищаем старые фрагменты
            if time.time() - last_cleanup > 5.0:
                self._cleanup_old_fragments()
                last_cleanup = time.time()
                
            # Показываем статистику
            if self._rx_total > 0 and self._rx_total % 50 == 0:
                pending_fragments = len(self._fragments)
                self.logger.info(f"Received {self._rx_total} messages, {pending_fragments} pending fragments")


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


