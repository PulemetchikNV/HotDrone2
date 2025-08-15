from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import logging
import os

# Настройка логгера
DRONE_NAME = os.getenv("DRONE_NAME", "unknown_drone")
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
formatter = logging.Formatter(f'%(asctime)s - {DRONE_NAME} - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

app = FastAPI()

# Глобальная переменная для хранения экземпляра ChessDroneSingle
# Это позволит нам вызывать его методы из эндпоинтов FastAPI
drone_instance = None

def set_drone_instance(drone):
    global drone_instance
    drone_instance = drone

class MoveCommand(BaseModel):
    to_cell: str

@app.post("/move")
async def move_drone(command: MoveCommand):
    if not drone_instance:
        logger.error("Drone instance is not set!")
        raise HTTPException(status_code=500, detail="Drone instance not initialized")

    logger.info(f"Received move command to {command.to_cell}")
    try:
        # Получаем координаты целевой клетки
        x, y = drone_instance.get_cell_coordinates(command.to_cell)
        
        # Выполняем движение
        # Важно: этот вызов должен быть асинхронным или неблокирующим
        # В данном случае, для простоты, мы делаем его блокирующим
        drone_instance.move_to_xy(x, y, drone_instance.flight_z)
        
        logger.info(f"Successfully completed move to {command.to_cell}")
        return {"status": "success", "message": f"Moved to {command.to_cell}"}
    except Exception as e:
        logger.error(f"Failed to execute move: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    return {"status": "ok"}
