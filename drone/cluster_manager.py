import os
import subprocess
import time
import signal
import sys
from typing import List, Optional, Dict, Any
from const import DRONES_CONFIG, get_drone_config


class ClusterManager:
    """Менеджер кластера для управления cluster_hosts файлом и перезапуском процессов"""
    
    def __init__(self, logger, esp_controller=None):
        self.logger = logger
        self.esp = esp_controller
        self.cluster_hosts_path = os.getenv("CLUSTER_HOSTFILE", "cluster_hosts")
        self.restart_delay = int(os.getenv("CLUSTER_RESTART_DELAY", "10"))  # секунд на перезапуск
        
    def get_alive_drone_ips(self, alive_drone_names: List[str]) -> List[str]:
        """Извлекает IP адреса живых дронов из конфигурации"""
        ips = []
        for drone_name in alive_drone_names:
            try:
                config = get_drone_config(drone_name)
                raw_ip = config.get('raw_ip')
                if raw_ip:
                    ips.append(raw_ip)
                    self.logger.debug(f"Drone {drone_name} -> IP {raw_ip}")
                else:
                    self.logger.warning(f"No raw_ip found for drone {drone_name}")
            except Exception as e:
                self.logger.warning(f"Failed to get IP for {drone_name}: {e}")
        return ips
    
    def read_current_cluster_hosts(self) -> List[str]:
        """Читает текущий cluster_hosts файл"""
        try:
            with open(self.cluster_hosts_path, 'r') as f:
                return [line.strip() for line in f if line.strip() and not line.strip().startswith('#')]
        except Exception as e:
            self.logger.debug(f"Failed to read cluster_hosts: {e}")
            return []
    
    def update_cluster_hosts(self, alive_drone_names: List[str]) -> bool:
        """Обновляет файл cluster_hosts с IP живых дронов"""
        ips = self.get_alive_drone_ips(alive_drone_names)
        if not ips:
            self.logger.error("No valid IPs found for alive drones")
            return False
            
        try:
            # Создаем бэкап старого файла
            backup_path = f"{self.cluster_hosts_path}.backup"
            if os.path.exists(self.cluster_hosts_path):
                os.rename(self.cluster_hosts_path, backup_path)
            
            # Записываем новый файл
            with open(self.cluster_hosts_path, 'w') as f:
                f.write("# Auto-generated cluster hosts file\n")
                f.write(f"# Updated at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                for ip in ips:
                    f.write(f"{ip}\n")
                    
            self.logger.info(f"Updated {self.cluster_hosts_path} with {len(ips)} IPs: {ips}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to update cluster_hosts: {e}")
            # Восстанавливаем бэкап при ошибке
            backup_path = f"{self.cluster_hosts_path}.backup"
            if os.path.exists(backup_path):
                try:
                    os.rename(backup_path, self.cluster_hosts_path)
                    self.logger.info("Restored cluster_hosts from backup")
                except Exception:
                    pass
            return False
    
    def cluster_composition_changed(self, alive_drone_names: List[str]) -> bool:
        """Проверяет, изменился ли состав кластера"""
        current_hosts = set(self.read_current_cluster_hosts())
        expected_ips = set(self.get_alive_drone_ips(alive_drone_names))
        
        changed = current_hosts != expected_ips
        if changed:
            self.logger.info(f"Cluster composition changed:")
            self.logger.info(f"  Current: {sorted(current_hosts)}")
            self.logger.info(f"  Expected: {sorted(expected_ips)}")
        
        return changed
    
    def notify_followers_about_restart(self, reason: str = "cluster_change") -> bool:
        """Уведомляет ведомых дронов о предстоящем перезапуске лидера"""
        if not self.esp or not self.esp.is_leader:
            return False
        
        restart_delay = self.restart_delay
        payload = {
            'type': 'leader_restart_notify',
            'reason': reason,
            'restart_delay': restart_delay,
            'from': self.esp.drone_name,
            'timestamp': time.time()
        }
        
        self.logger.info(f"Notifying followers about leader restart (delay: {restart_delay}s, reason: {reason})")
        return self.esp._broadcast_unreliable(payload)
    
    def notify_followers_restart_complete(self) -> bool:
        """Уведомляет ведомых дронов о завершении перезапуска лидера"""
        if not self.esp or not self.esp.is_leader:
            return False
        
        payload = {
            'type': 'leader_restart_complete',
            'from': self.esp.drone_name,
            'timestamp': time.time()
        }
        
        self.logger.info("Notifying followers that leader restart is complete")
        return self.esp._broadcast_unreliable(payload)
    
    def initiate_cluster_restart(self, alive_drone_names: List[str], reason: str = "cluster_change") -> bool:
        """
        Инициирует перезапуск кластера:
        1. Уведомляет ведомых о перезапуске
        2. Обновляет cluster_hosts
        3. Устанавливает флаг перезапуска
        """
        try:
            # 1. Уведомляем ведомых о предстоящем перезапуске
            if not self.notify_followers_about_restart(reason):
                self.logger.warning("Failed to notify followers about restart")
            
            # Даем время доставить сообщение
            time.sleep(0.5)
            
            # 2. Обновляем cluster_hosts файл
            if not self.update_cluster_hosts(alive_drone_names):
                self.logger.error("Failed to update cluster_hosts")
                return False
            
            # 3. Устанавливаем флаг для перезапуска с информацией
            restart_info = {
                'reason': reason,
                'timestamp': time.time(),
                'alive_drones': alive_drone_names,
                'restart_delay': self.restart_delay
            }
            
            self.set_restart_flag(restart_info)
            self.logger.info(f"Cluster restart initiated: {reason}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initiate cluster restart: {e}")
            return False
    
    def set_restart_flag(self, restart_info: Dict[str, Any]):
        """Устанавливает флаг перезапуска с информацией"""
        try:
            import json
            with open(".restart_required", "w") as f:
                json.dump(restart_info, f, indent=2)
            self.logger.info(f"Restart flag set: {restart_info['reason']}")
        except Exception as e:
            self.logger.error(f"Failed to set restart flag: {e}")
    
    def check_restart_flag(self) -> Optional[Dict[str, Any]]:
        """Проверяет наличие флага перезапуска"""
        try:
            import json
            with open(".restart_required", "r") as f:
                return json.load(f)
        except Exception:
            return None
    
    def clear_restart_flag(self):
        """Удаляет флаг перезапуска"""
        try:
            if os.path.exists(".restart_required"):
                os.remove(".restart_required")
        except Exception as e:
            self.logger.debug(f"Failed to remove restart flag: {e}")
    
    def perform_graceful_restart(self, reason: str = "cluster_change"):
        """Выполняет изящный перезапуск главного процесса"""
        self.logger.info(f"Performing graceful restart: {reason}")
        
        # Устанавливаем переменную окружения для индикации перезапуска
        os.environ["CLUSTER_RESTART_IN_PROGRESS"] = "1"
        os.environ["CLUSTER_RESTART_REASON"] = reason
        
        # Завершаем процесс с специальным кодом
        sys.exit(42)  # Специальный код для перезапуска кластера


def update_cluster_if_needed(alive_drones: List[str], logger, esp_controller=None) -> bool:
    """
    Проверяет и обновляет кластер если список дронов изменился.
    
    Returns:
        bool: True если нужен перезапуск, False если изменений нет
    """
    cluster_mgr = ClusterManager(logger, esp_controller)
    
    # Проверяем изменения в составе кластера
    if cluster_mgr.cluster_composition_changed(alive_drones):
        logger.info(f"Cluster composition changed, initiating restart")
        
        # Инициируем перезапуск кластера
        if cluster_mgr.initiate_cluster_restart(alive_drones, "drone_list_changed"):
            return True
        else:
            logger.error("Failed to initiate cluster restart")
    
    return False


def handle_restart_after_startup(logger, esp_controller=None):
    """
    Обрабатывает логику после перезапуска - уведомляет ведомых о завершении перезапуска
    """
    cluster_mgr = ClusterManager(logger, esp_controller)
    
    # Проверяем, был ли это перезапуск кластера
    restart_reason = os.getenv("CLUSTER_RESTART_REASON")
    if restart_reason:
        logger.info(f"Detected cluster restart completion (reason: {restart_reason})")
        
        # Даем время системе стабилизироваться
        time.sleep(2)
        
        # Уведомляем ведомых о завершении перезапуска
        if cluster_mgr.notify_followers_restart_complete():
            logger.info("Notified followers about restart completion")
        else:
            logger.warning("Failed to notify followers about restart completion")
        
        # Очищаем переменные окружения
        os.environ.pop("CLUSTER_RESTART_IN_PROGRESS", None)
        os.environ.pop("CLUSTER_RESTART_REASON", None)
        
        # Очищаем флаг перезапуска
        cluster_mgr.clear_restart_flag()
