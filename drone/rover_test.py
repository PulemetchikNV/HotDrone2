from rover import RoverControllerMain
import time

rover = RoverControllerMain()

#rover.navigate(rover_id='rover_b', current_x=0, current_y=0, current_yaw=0, target_x=1, target_y=0)


def square_move():
    # движемся по квадрату
    distance = 0.3
    rover_id = 'rover_b'  # Используем rover_id из const.py

    rover.navigate(rover_id, current_x=0, current_y=0, current_yaw=0, target_x=distance, target_y=0)

    time.sleep(2)

    rover.navigate(rover_id, current_x=distance, current_y=0, current_yaw=0, target_x=distance, target_y=distance)

    time.sleep(2)

    rover.navigate(rover_id, current_x=distance, current_y=distance, current_yaw=0, target_x=0, target_y=distance)

    time.sleep(2)

    rover.navigate(rover_id, current_x=0, current_y=distance, current_yaw=0, target_x=0, target_y=0)

def turn_move():
    # едем вперед, разворачиваемся и едем назад
    distance = 0.4
    rover_id = 'rover_b'  # Используем rover_id из const.py

    rover.navigate(rover_id, current_x=0, current_y=0, current_yaw=0, target_x=distance, target_y=0)

    time.sleep(2)

    rover.navigate(rover_id, current_x=distance, current_y=0, current_yaw=355, target_x=0, target_y=0)

turn_move()