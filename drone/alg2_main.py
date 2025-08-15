import sys
import os
import logging

try:
    # Относительные импорты (когда запускается как пакет)
    from . import env_loader
    from .chess import ChessDroneSingle as Drone
except ImportError:
    # Абсолютные импорты (когда запускается напрямую)
    import env_loader
    from chess import ChessDroneSingle as Drone

def parse_args():
    drone_name = None
    for i, arg in enumerate(sys.argv):
        if arg.startswith('--drone-name='):
            drone_name = arg.split('=', 1)[1]
        elif arg == '--drone-name' and i + 1 < len(sys.argv):
            drone_name = sys.argv[i + 1]
    return drone_name

env_loader.load_env_file()

try:
    import rospy
except ImportError:
    # Mock rospy for local testing
    class MockRospy:
        def init_node(self, name):
            logging.warning(f"[MOCK] rospy.init_node('{name}')")
    rospy = MockRospy()

rospy.init_node('chess_alg2')

# Create drone instance with Stockfish integration
drone = Drone()

# Run the drone
drone.run()
