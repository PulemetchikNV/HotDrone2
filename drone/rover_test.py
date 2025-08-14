from rover import RoverControllerMain

rover = RoverControllerMain(initial_x=10, initial_y=10, initial_yaw=0)

rover.navigate(rover_id='1', x=150, y=150, yaw=0)