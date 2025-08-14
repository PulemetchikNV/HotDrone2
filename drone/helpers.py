import logging

# Робастный импорт логгеров: сначала относительный, затем локальный пакет, затем абсолютный пакет
UDPLogHandler = None
UDPLogServer = None
try:
    from .log.log_handler import UDPLogHandler as _UDPLogHandler
    from .log.log_server import UDPLogServer as _UDPLogServer
    UDPLogHandler, UDPLogServer = _UDPLogHandler, _UDPLogServer
except Exception:
    try:
        # Запуск как скрипт из каталога drone
        from log.log_handler import UDPLogHandler as _UDPLogHandler
        from log.log_server import UDPLogServer as _UDPLogServer
        UDPLogHandler, UDPLogServer = _UDPLogHandler, _UDPLogServer
    except Exception:
        try:
            # Запуск из корня проекта: импорт через drone.log.*
            from drone.log.log_handler import UDPLogHandler as _UDPLogHandler
            from drone.log.log_server import UDPLogServer as _UDPLogServer
            UDPLogHandler, UDPLogServer = _UDPLogHandler, _UDPLogServer
        except Exception:
            UDPLogHandler = None
            UDPLogServer = None


def setup_logging(drone_name, log_level=logging.INFO):
    logger = logging.getLogger(drone_name)
    logger.setLevel(log_level)
    
    # Console handler (idempotent)
    if not any(isinstance(h, logging.StreamHandler) for h in logger.handlers):
        console_handler = logging.StreamHandler()
        console_formatter = logging.Formatter(
            f'[{drone_name}] %(asctime)s - %(levelname)s - %(message)s', '%H:%M:%S'
        )
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)
    
    # UDP handler (idempotent)
    if UDPLogHandler is not None and not any(isinstance(h, UDPLogHandler) for h in logger.handlers):
        try:
            udp_handler = UDPLogHandler(drone_name=drone_name)
            logger.addHandler(udp_handler)
        except Exception:
            # Мягко продолжаем без UDP-логгера
            pass
    
    return logger

def setup_log_server(drone_name):
    if UDPLogServer is None:
        return None
    log_server = UDPLogServer(drone_name=drone_name)
    log_server.start_server()
    return log_server