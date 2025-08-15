from sys import argv
import time
import random
import json
from flight import FlightController
import os

FILES = "abcdefgh"
RANKS = "12345678"

def get_random_cell():
    return f"{random.choice(FILES)}{random.choice(RANKS)}"

def cell_to_xy(cell: str, cell_size: float, origin_x: float, origin_y: float):
    f, r = cell[0], cell[1]
    fi = FILES.index(f)
    ri = RANKS.index(r)
    # a1 в нижнем левом углу; X вправо по файлам, Y вверх по рангам
    x = origin_x + (fi - 3.5) * cell_size
    y = origin_y + (ri - 3.5) * cell_size
    return x, y

class Progon:
    
    def __init__(self):
        self.fc = FlightController()

        self.map_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "aruco_maps",
            "aruco_map2.json",
        )
        self.cell_markers = {}

        try:
            with open(self.map_path, "r") as f:
                data = json.load(f)
                if isinstance(data, dict):
                    self.cell_markers = data
                else:
                    print(f"Unexpected JSON structure in {self.map_path}")
        except Exception as e:
            print(f"Failed to load ArUco map from {self.map_path}: {e}")


    def move_to_xy(self, x: float, y: float, z: float):
        # 1. Взлёт на рабочую высоту
        self.fc.takeoff(z=1.2, delay=2, speed=0.5)
        time.sleep(3)

        self.fc.navigate_wait(
            x=x,
            y=y,
            z=1.2,
            speed=0.4,
            auto_arm=True,
        )
        self.fc.wait(0.5)
        
        self.fc.navigate_wait(
            x=x, 
            y=y, 
            z=0.15,
            speed=0.2,
            auto_arm=False,
        )


        self.fc.wait(0.8)

        self.fc.force_disarm()

    def get_cell_coordinates(self, cell):
        """Получает координаты клетки из карты или вычисляет их"""
        marker = self.cell_markers.get(cell)
        if isinstance(marker, dict) and "x" in marker and "y" in marker:
            x, y = float(marker["x"]), float(marker["y"])
            print(f"Using map coords for {cell}: x={x:.3f}, y={y:.3f}")
            return x, y
        else:
            x, y = cell_to_xy(cell, 0.5, 0, 0)
            print(f"No map coords for {cell}. Using computed coords: x={x:.3f}, y={y:.3f}")
            return x, y
    
    def run(self, to_cell):
        print(f"Moving to {to_cell}")
        x, y = self.get_cell_coordinates(to_cell)
        # Выполняем движение локально
        self.move_to_xy(x, y, 1.2)

if __name__ == "__main__":
    Progon().run(argv[1] if len(argv) > 1 else get_random_cell())
