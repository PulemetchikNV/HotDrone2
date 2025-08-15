import sys
import os
import logging

# командой `python3 -m drone.main`
from .env_loader import load_env_file
from .chess import ChessDroneSingle as Drone

def parse_args():
    drone_name = None
    for i, arg in enumerate(sys.argv):
        if arg.startswith('--drone-name='):
            drone_name = arg.split('=', 1)[1]
        elif arg == '--drone-name' and i + 1 < len(sys.argv):
            drone_name = sys.argv[i + 1]
    return drone_name

# Загружаем переменные окружения из .env файла
load_env_file()

# Мок rospy для локального тестирования, если он недоступен
try:
    import rospy
except ImportError:
    class MockRospy:
        def init_node(self, name):
            logging.warning(f"[MOCK] rospy.init_node('{name}')")
    rospy = MockRospy()

rospy.init_node('chess')

drone = Drone()

drone.run()
