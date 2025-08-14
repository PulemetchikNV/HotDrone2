from rover import RoverControllerMain
import time

rover = RoverControllerMain(initial_x=0, initial_y=0, initial_yaw=0)

#rover.navigate(rover_id='2', x=1, y=0, yaw=0)


def square_move():
    # движемся по квадарту
    distance = 0.3
    drone_id = '2'

    rover.navigate(drone_id, current_x=0, current_y=0, current_yaw=0, x=distance, y=0)

    time.sleep(2)

    rover.navigate(drone_id, current_x=distance, current_y=0, current_yaw=0, x=distance, y=distance)

    time.sleep(2)

    rover.navigate(drone_id, current_x=distance, current_y=distance, current_yaw=0, x=0, y=distance)

    time.sleep(2)

    rover.navigate(drone_id, current_x=distance, current_y=0, current_yaw=0, x=0, y=0)

def turn_move():
    # едем вперед, разворачиваемся и едем назад
    distance = 0.4
    drone_id = '2'

    rover.navigate(drone_id, current_x=0, current_y=0, current_yaw=0, x=distance, y=0)

    time.sleep(2)

    rover.navigate(drone_id, current_x=distance, current_y=0, current_yaw=355, x=0, y=0)

turn_move()