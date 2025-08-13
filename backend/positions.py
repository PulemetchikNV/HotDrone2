import cv2
import requests
import numpy as np
import json
from .popukai import get_converted_coords
from .constants import DRONE_IDS, SHEEP_ID, stream_url, DRONE_ID_TO_DRONE_NAMES

# Получение кадров из MJPEG-потока
def get_latest_frame(url):
    stream = requests.get(url, stream=True)
    bytes_data = b''
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b'\xff\xd8')  # Start of JPEG
        b = bytes_data.find(b'\xff\xd9')  # End of JPEG

        while a != -1 and b != -1 and b > a:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            yield frame
            a = bytes_data.find(b'\xff\xd8')
            b = bytes_data.find(b'\xff\xd9')

# Настройка ArUco словаря и параметров
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters()



def to_algebraic(pos):
    """Converts (row, col) to algebraic notation like 'A1'."""
    r, c = pos
    return f"{chr(ord('A') + c)}{8 - r}"

def from_algebraic(cell_str):
    """Converts algebraic notation like 'A1' to (row, col)."""
    if not cell_str or len(cell_str) != 2:
        return None
    try:
        c = ord(cell_str[0].lower()) - ord('a')
        r = 8 - int(cell_str[1])
        if not (0 <= r < 8 and 0 <= c < 8):
            return None
        return r, c
    except (ValueError, IndexError):
        return None

# Настройка ArUco словаря и параметров
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

# Получение кадра из MJPEG-потока
def get_frame_from_stream(url):
    try:
        stream = requests.get(url, stream=True, timeout=2)
        bytes_data = b''
        for chunk in stream.iter_content(chunk_size=1024):
            bytes_data += chunk
            a = bytes_data.find(b'\xff\xd8')  # Start of JPEG
            b = bytes_data.find(b'\xff\xd9')  # End of JPEG
            if a != -1 and b != -1:
                jpg = bytes_data[a:b+2]
                bytes_data = bytes_data[b+2:]
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                return frame
    except requests.exceptions.RequestException as e:
        print(f"Error getting frame from stream: {e}")
        return None
    return None

def get_positions():
    frame = get_frame_from_stream(stream_url)
    if frame is None:
        return {"error": "Could not get frame from stream"}

    # Параметры для коррекции перспективы (взяты из detector.py)
    pts1 = np.float32([[117, 29], [473, 3], [487, 368], [136, 377]])

    width, height = 700, 700
    margin_top = -42
    margin_bottom = -90
    margin_left = -110
    margin_right = -30

    # 3. Новые целевые координаты (pts2)
    #    Они определяют прямоугольник ВНУТРИ изображения 700x700,
    #    куда будет помещено откалиброванное изображение.
    pts2 = np.float32([
        [margin_left, margin_top],                                   # Верхний левый угол
        [width - margin_right, margin_top],                          # Верхний правый
        [width - margin_right, height - margin_bottom],              # Нижний правый
        [margin_left, height - margin_bottom]                        # Нижний левый
    ])

    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    
    warped_frame = cv2.warpPerspective(frame, matrix, (width, height))
    gray = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2GRAY)
    
    corners, ids, _ = DETECTOR.detectMarkers(gray)
    
    positions = {}
    if ids is not None:
        for i, marker_id_array in enumerate(ids):
            marker_id = int(marker_id_array[0])
            
            marker_corners = corners[i].reshape((4, 2))
            cX = int(np.mean(marker_corners[:, 0]))
            cY = int(np.mean(marker_corners[:, 1]))
            coords = get_converted_coords(cX, cY)
            cell = get_cell_from_coords(coords)
            
            positions[marker_id] = {
                "x": coords["x"],
                "y": coords["y"],
                "cell": cell,
            }
            
    return positions



def get_drone_positions():
    """
    Gets the positions of only the drones.
    """
    all_positions = get_positions()
    if "error" in all_positions:
        return all_positions
    
    drone_positions = {id: pos for id, pos in all_positions.items() if id in DRONE_IDS}
    return drone_positions

def get_sheep_position():
    """
    Gets the position of the sheep.
    """
    all_positions = get_positions()
    if "error" in all_positions:
        return all_positions
        
    return all_positions.get(SHEEP_ID)

def get_cell_from_coords(coords):
    """
    Finds the closest cell in the aruco_map.json file to the given coordinates.
    """
    if not coords:
        return None

    try:
        with open('aruco_map.json', 'r') as f:
            aruco_map = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None

    black_cells = [item for item in aruco_map if item.get('length') == 0.27 and 'cell' in item]
    if not black_cells:
        return None

    min_dist = float('inf')
    closest_cell = None
    
    for cell_data in black_cells:
        dist = np.linalg.norm(np.array([coords['x'], coords['y'], 0]) - np.array([cell_data['x'], cell_data['y'], cell_data['z']]))
        if dist < min_dist:
            min_dist = dist
            closest_cell = cell_data['cell']
            
    return closest_cell

def get_block_sheep_positions(sheep_cell: str):
    """
    Finds the coordinates of the black cells adjacent to the sheep's current cell.
    """
    if not sheep_cell:
        return []

    try:
        with open('aruco_map.json', 'r') as f:
            aruco_map = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []

    # Create a quick lookup map for cell name to coordinates
    cell_map = {item['cell']: item for item in aruco_map if 'cell' in item and item.get('length') == 0.27}
    
    current_pos = from_algebraic(sheep_cell)
    if not current_pos:
        return []

    r, c = current_pos
    neighboring_cells = []
    
    # Define the four diagonal directions for blocking
    diagonal_moves = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    for dr, dc in diagonal_moves:
        nr, nc = r + dr, c + dc
        if 0 <= nr < 8 and 0 <= nc < 8:
            neighbor_cell_alg = to_algebraic((nr, nc))
            if neighbor_cell_alg in cell_map:
                neighboring_cells.append(cell_map[neighbor_cell_alg])
                
    # Return only the coordinates
    response = []
    for i, cell in enumerate(neighboring_cells):
        if i < len(DRONE_IDS):
            drone_id = DRONE_IDS[i]
            drone_name = DRONE_ID_TO_DRONE_NAMES.get(drone_id)
            if drone_name:
                response.append({
                    "cell": cell['cell'].lower(),
                    "drone_name": drone_name,
                    "coords": [cell['x'], cell['y'], cell['z']]
                })
    return response
