from rover import RoverControllerMain
import time

rover = RoverControllerMain()

def test_basic_commands():
    """Тест базовых команд: движение, поворот, остановка"""
    rover_id = 'rover_a'
    
    print("=== Тест базовых команд ===")
    
    # Получение статуса
    status = rover.get_status(rover_id)
    print(f"Статус ровера {rover_id}: {status}")
    
    # Движение вперед на 200мм
    rover.move_forward(rover_id, 200)
    rover.wait_for_completion(rover_id)
    
    # Поворот направо на 90 градусов
    rover.turn(rover_id, 90)
    rover.wait_for_completion(rover_id)
    
    # Остановка
    rover.stop(rover_id)

def test_diagonal_movement():
    """Тест диагонального движения - новая функциональность"""
    rover_id = 'rover_a'
    
    print("=== Тест диагонального движения ===")
    
    # Диагональное движение: поворот на 45° и движение на 300мм
    print("Диагональное движение вперед-направо...")
    rover.move_diagonal(rover_id, distance_mm=300, angle=45)
    
    time.sleep(1)
    
    # Диагональное движение с возвратом в исходную ориентацию
    print("Диагональное движение с возвратом ориентации...")
    rover.move_diagonal_and_return(rover_id, distance_mm=200, angle=-45)

def test_chess_pawn_moves():
    """Тест движений пешки в шахматах"""
    rover_id = 'rover_b'  
    
    print("=== Тест движений пешки ===")
    
    # Размер клетки шахматной доски в мм (примерно)
    cell_size_mm = 150
    
    # Движение пешки вперед на одну клетку
    print("Пешка движется вперед на одну клетку...")
    rover.move_forward(rover_id, cell_size_mm)
    rover.wait_for_completion(rover_id)
    
    time.sleep(2)
    
    # Атака пешки по диагонали (вперед-направо)
    print("Пешка атакует по диагонали вперед-направо...")
    rover.move_diagonal(rover_id, distance_mm=cell_size_mm, angle=45)
    
    time.sleep(2)
    
    # Возврат в исходное положение для следующего теста
    print("Возврат в исходное положение...")
    rover.move_diagonal_and_return(rover_id, distance_mm=cell_size_mm, angle=-135)

def test_square_pattern_new():
    """Тест движения по квадрату с использованием нового API"""
    rover_id = 'rover_c'
    
    print("=== Тест движения по квадрату (новый API) ===")
    
    # Размер стороны квадрата в мм
    side_length_mm = 250
    
    # Создаем последовательность команд для квадрата
    square_commands = [
        {'type': 'forward', 'value': side_length_mm, 'wait': True},
        {'type': 'turn', 'value': 90, 'wait': True},
        {'type': 'forward', 'value': side_length_mm, 'wait': True},
        {'type': 'turn', 'value': 90, 'wait': True},
        {'type': 'forward', 'value': side_length_mm, 'wait': True},
        {'type': 'turn', 'value': 90, 'wait': True},
        {'type': 'forward', 'value': side_length_mm, 'wait': True},
        {'type': 'turn', 'value': 90, 'wait': True}
    ]
    
    rover.execute_sequence(rover_id, square_commands)

def test_legacy_navigate():
    """Тест legacy навигации (совместимость со старым API)"""
    rover_id = 'rover_d'
    
    print("=== Тест legacy навигации ===")
    
    # Движение по квадрату с использованием coordinate navigation
    distance = 0.3  # 30 см
    
    print("Legacy навигация: квадрат...")
    rover.navigate(rover_id, current_x=0, current_y=0, current_yaw=0, target_x=distance, target_y=0)
    time.sleep(3)
    
    rover.navigate(rover_id, current_x=distance, current_y=0, current_yaw=0, target_x=distance, target_y=distance)
    time.sleep(3)
    
    rover.navigate(rover_id, current_x=distance, current_y=distance, current_yaw=0, target_x=0, target_y=distance)
    time.sleep(3)
    
    rover.navigate(rover_id, current_x=0, current_y=distance, current_yaw=0, target_x=0, target_y=0)

def test_complex_chess_scenario():
    """Тест сложного шахматного сценария"""
    rover_id = 'rover_a'
    
    print("=== Тест сложного шахматного сценария ===")
    
    cell_size_mm = 150
    
    # Сценарий: пешка движется вперед, затем атакует по диагонали, 
    # потом отступает и поворачивается
    chess_scenario = [
        # Движение вперед на 2 клетки (начальный ход пешки)
        {'type': 'forward', 'value': cell_size_mm * 2, 'wait': True, 'delay': 1},
        
        # Поворот для атаки направо
        {'type': 'turn', 'value': 45, 'wait': True},
        
        # Атака по диагонали
        {'type': 'forward', 'value': cell_size_mm, 'wait': True, 'delay': 1},
        
        # Поворот для возврата к прямому движению
        {'type': 'turn', 'value': -45, 'wait': True},
        
        # Финальное движение вперед
        {'type': 'forward', 'value': cell_size_mm, 'wait': True}
    ]
    
    rover.execute_sequence(rover_id, chess_scenario)

def main():
    """Главная функция для запуска всех тестов"""
    print("Начало тестирования обновленного rover API...")
    print("=" * 50)
    
    try:
        # Включаем/выключаем нужные тесты
        
        # test_basic_commands()
        # print("\n" + "="*50 + "\n")
        
        test_diagonal_movement()
        print("\n" + "="*50 + "\n")
        
        # test_chess_pawn_moves() 
        # print("\n" + "="*50 + "\n")
        
        # test_square_pattern_new()
        # print("\n" + "="*50 + "\n")
        
        # test_legacy_navigate()
        # print("\n" + "="*50 + "\n")
        
        # test_complex_chess_scenario()
        
        print("Все тесты завершены!")
        
    except Exception as e:
        print(f"Ошибка во время тестирования: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()