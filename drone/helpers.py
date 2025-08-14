import logging

try:
    from .log.log_handler import UDPLogHandler
    from .log.log_server import UDPLogServer
except ImportError:
    from .log.log_handler import UDPLogHandler
    from .log.log_server import UDPLogServer


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
    if not any(isinstance(h, UDPLogHandler) for h in logger.handlers):
        udp_handler = UDPLogHandler(drone_name=drone_name)
        logger.addHandler(udp_handler)
    
    return logger

def setup_log_server(drone_name):
    log_server = UDPLogServer(drone_name=drone_name)
    log_server.start_server()
    return log_server