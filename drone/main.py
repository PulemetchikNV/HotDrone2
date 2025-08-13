import sys
import os
import logging

import env_loader

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
    from .chess import ChessDroneSingle as Drone
except ImportError:
    from chess import ChessDroneSingle as Drone

try:
    import rospy
except ImportError:
    # Mock rospy for local testing
    class MockRospy:
        def init_node(self, name):
            logging.warning(f"[MOCK] rospy.init_node('{name}')")
    rospy = MockRospy()

rospy.init_node('chess')

drone = Drone()

drone.run() 