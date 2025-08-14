import math
import time
import threading
import os
import logging

# rospy and ROS services fallback for local testing
try:
    import rospy
    from clover import srv
    from std_srvs.srv import Trigger
    from mavros_msgs.srv import CommandBool, SetMode
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    IS_MOCK_MODE = False
except ImportError as e:
    IS_MOCK_MODE = True
    logging.warning("=" * 60)
    logging.warning("!!! ROS (rospy) not found. Running in MOCK mode. !!!")
    logging.warning(f"!!! Import error: {e}")
    logging.warning("!!! All flight commands will be simulated.           !!!")
    logging.warning("=" * 60)

    class MockRospy:
        def is_shutdown(self):
            return False
        def wait_for_message(self, topic, msg_type, timeout=None):
            logging.warning(f"[MOCK] rospy.wait_for_message('{topic}') -> returning empty message")
            return MockMsg()
        def ServiceProxy(self, service_name, service_type):
            logging.warning(f"[MOCK] rospy.ServiceProxy('{service_name}') -> returning mock service")
            def mock_service(**kwargs):
                logging.info(f"[MOCK] Service '{service_name}' called with {kwargs}")
                return MockResponse()
            return mock_service
        def Publisher(self, topic, msg_type, queue_size=1):
            logging.warning(f"[MOCK] rospy.Publisher('{topic}') -> returning mock publisher")
            return MockPublisher(topic)
        def Subscriber(self, topic, msg_type, callback):
            logging.warning(f"[MOCK] rospy.Subscriber('{topic}') -> returning mock subscriber")
            return MockSubscriber()
        def sleep(self, duration):
            import time
            time.sleep(duration)
    
    class MockMsg:
        def __init__(self):
            self.data = ""
    
    class MockResponse:
        def __init__(self):
            self.success = True
            self.armed = True # Simulate successful arming
            self.mode_sent = True
    
    class MockPublisher:
        def __init__(self, topic):
            self.topic = topic
        def publish(self, msg):
            logging.info(f"[MOCK] Publishing to topic '{self.topic}': {msg}")
    
    class MockSubscriber:
        pass
    
    # Mock modules
    rospy = MockRospy()
    srv = type('srv', (), {
        'Navigate': type('Navigate', (), {}),
        'GetTelemetry': type('GetTelemetry', (), {}),
        'SetLEDEffect': type('SetLEDEffect', (), {})
    })()
    Trigger = type('Trigger', (), {})
    CommandBool = type('CommandBool', (), {})
    SetMode = type('SetMode', (), {})
    PoseStamped = type('PoseStamped', (), {})
    String = type('String', (), {'data': ''})

try:
    from .helpers import setup_logging
except ImportError:
    from helpers import setup_logging


def scan_qr(logger, timeout=5.0):
    try:
        msg = rospy.wait_for_message('qr_results', String, timeout=timeout)
        data = (msg.data or '').strip()
        if not data:
            logger.info("QR: ничего не найдено")
            return []
        codes = [s.strip() for s in data.splitlines() if s.strip()]
        logger.info(f"QR: найдены {codes}")
        return codes
    except rospy.ROSException:
        logger.warning("QR: таймаут ожидания результата")
        return []

class FlightControllerCustom:
    def __init__(self, drone_name=None, logger=None):
        self.autoland = rospy.ServiceProxy("land", Trigger)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.set_led = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.force_arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.initial_z = 0.0
        self.drone_name = drone_name or os.environ.get('DRONE_NAME', 'unknown_drone')
        self.logger = logger or setup_logging(self.drone_name)

    def land(self, prl_aruco="aruco_map", prl_speed=0.5, prl_bias_x=-0.08, prl_bias_y=0.1, prl_z=0.6, prl_tol=0.1, delay=4.0, fall_time=1, fall_z=-1, fall_speed=1):
        telem = self.get_telemetry(frame_id="aruco_map")
        self.logger.info("Pre-landing")
        self.set_led(effect='blink', r=255, g=255, b=255)
        if prl_aruco is None:
            self.navigate_wait(x=telem.x, y=telem.y, z=prl_z, speed=prl_speed, frame_id="aruco_map", tolerance=prl_tol)
        else:
            self.navigate_wait(x=prl_bias_x, y=prl_bias_y, z=prl_z, speed=prl_speed, frame_id=prl_aruco, tolerance=prl_tol)
        self.wait(2.0)
        self.logger.info("Landing")
        self.set_led(effect='blink', r=255, g=165, b=0)
        telem = self.get_telemetry(frame_id="base_link")
        self.navigate(x=telem.x, y=telem.y, z=fall_z, speed=fall_speed, frame_id="base_link")
        self.wait(fall_time)
        self.navigate(x=telem.x, y=telem.y, z=fall_z, speed=0, frame_id="base_link")
        self.force_arm(False)
        self.logger.info("Landed")
        self.set_mode_service(custom_mode="AUTO.LAND")
        self.wait(0.2)
        while self.get_telemetry(frame_id="body").mode != "STABILIZED":
            self.set_mode_service(custom_mode="STABILIZED")
            self.wait(3)
        self.wait(1)
        self.force_arm(True)
        self.wait(1)
        while self.get_telemetry(frame_id="body").armed:
            self.force_arm(False)
            self.wait(3)
        self.set_led(r=0, g=255, b=0)
        # Threading control for async publisher
        self.publisher_thread = None
        self.stop_publisher = threading.Event()

    # --- Core flight primitives ---
    def navigate_wait(
        self,
        x=0.0,
        y=0.0,
        z=0.0,
        yaw=float("nan"),
        speed=0.5,
        frame_id="",
        auto_arm=False,
        tolerance=0.2,
    ):
        self.logger.info(f"Navigating to x={x:.2f} y={y:.2f} z={z:.2f} in {frame_id}")
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        telem = self.get_telemetry(frame_id="body")
        if not telem.armed:
            raise RuntimeError("Arming failed!")

        while True:
            telem = self.get_telemetry(frame_id="navigate_target")
            if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
                self.wait(0.1)
                self.logger.info("Arrived at target")
                break
            self.wait(0.1)

    def takeoff(self, z=1.1, delay=0.5, time_spam=2.5, time_warm=2, time_up=0.5):
        self.logger.info(f"Taking off to z={z:.2f}")
        self.set_led(effect='blink', r=255, g=255, b=255)

        self.force_arm(True)
        self.send_fake_pos_async(duration=time_spam)
        self.wait(time_warm)
        self.navigate(z=100, speed=0.5, frame_id="body", auto_arm=True)
        self.wait(time_up)
        telem = self.get_telemetry(frame_id="aruco_map")
        if not telem.armed:
            raise RuntimeError("Arming failed!")
        self.set_led(effect='blink', r=255, g=165, b=0)

        self.navigate(x=telem.x, y=telem.y, z=z, yaw=math.pi, speed=0.3, frame_id="aruco_map")
        self.wait(delay)

        self.logger.info("Takeoff done")
        self.set_led(r=0, g=255, b=0)
        self.stop_fake_pos_async()



    def wait(self, duration):
        rospy.sleep(duration)
        if rospy.is_shutdown():
            raise RuntimeError("rospy shutdown")

    # --- QR ---
    def scan_qr_code(self, timeout=5.0):
        return scan_qr(self.logger, timeout)

    # --- Vision pose spoofing ---
    def _fake_pos_publisher(self, duration=5.0):
        self.logger.info(f"Started vision pose publishing for {duration}s")
        pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        rate = rospy.Rate(50)
        start_time = time.time()
        target_z = self.initial_z + 20.0
        current_z_temp = 0
        while (time.time() - start_time < duration and 
               not rospy.is_shutdown() and 
               not self.stop_publisher.is_set()):
            elapsed = time.time() - start_time
            progress = elapsed / duration
            current_z = self.initial_z + (target_z - self.initial_z) * progress
            current_z_temp = current_z
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "body"
            msg.pose.position.x = 0
            msg.pose.position.y = 0
            msg.pose.position.z = -current_z
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 0.7958915586785147
            pub.publish(msg)
            rate.sleep()
        self.logger.info("Stopped vision pose publishing")
        self.initial_z = current_z_temp

    def send_fake_pos_async(self, duration=5.0):
        if self.publisher_thread is not None and self.publisher_thread.is_alive():
            self.logger.warning("Publisher thread already running")
            return
        self.stop_publisher.clear()
        self.publisher_thread = threading.Thread(target=self._fake_pos_publisher, args=(duration,), daemon=True)
        self.publisher_thread.start()
        self.logger.info("Started async fake position publisher")

    def stop_fake_pos_async(self):
        if self.publisher_thread is not None and self.publisher_thread.is_alive():
            self.stop_publisher.set()
        
        if self.publisher_thread is not None:
            self.publisher_thread.join(timeout=1.0)
            self.logger.info("Stopped async fake position publisher")

    def send_fake_pos(self, duration=5.0):
        self._fake_pos_publisher(duration)

    def emergency_land(self):
        self.stop_fake_pos_async()
        self.land()

class FlightControllerMain:
    """Реализация на стандартных сервисах Clover без фейковой publish-поддержки."""
    def __init__(self, drone_name=None, logger=None):
        self.autoland = rospy.ServiceProxy("land", Trigger)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.set_led = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.force_arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.drone_name = drone_name or os.environ.get('DRONE_NAME', 'unknown_drone')
        self.logger = logger or setup_logging(self.drone_name)

    def wait(self, duration):
        rospy.sleep(duration)
        if rospy.is_shutdown():
            raise RuntimeError("rospy shutdown")

    def navigate(self, x=0.0, y=0.0, z=0.0, yaw=float("nan"), speed=0.5, frame_id="", auto_arm=False):
        self.logger.info(f"Navigating to x={x:.2f} y={y:.2f} z={z:.2f} in {frame_id}")
        self.navigate_service(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    def navigate_wait(self, x=0, y=0, z=1.15, yaw=float('nan'),speed=0.4, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
        self.logger.info(f"Navigating to x={x:.2f} y={y:.2f} z={z:.2f} in {frame_id}")
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)


    def takeoff(self, z=1.1, delay=0.5, speed=0.5):
        self.logger.info(f"Taking off to z={z:.2f}")
        self.set_led(effect='blink', r=255, g=255, b=255)
        # Взлёт стандартным navigate с автоармингом
        self.navigate(z=z, speed=speed, frame_id="body", auto_arm=True)
        # Подождать стабилизацию
        self.wait(max(delay, 0.5))
        self.logger.info("Takeoff done")
        self.set_led(r=0, g=255, b=0)



    def land(self, prl_aruco="aruco_map", prl_speed=0.5, prl_bias_x=-0.08, prl_bias_y=0.1, prl_z=0.6, prl_tol=0.1, delay=4.0, fall_time=1, fall_z=-1, fall_speed=1):
        telem = self.get_telemetry(frame_id="aruco_map")
        self.logger.info("Pre-landing")
        self.set_led(effect='blink', r=255, g=255, b=255)
        if prl_aruco is None:
            self.navigate_wait(x=telem.x, y=telem.y, z=prl_z, speed=prl_speed, frame_id="aruco_map", tolerance=prl_tol)
        else:
            self.navigate_wait(x=prl_bias_x, y=prl_bias_y, z=prl_z, speed=prl_speed, frame_id=prl_aruco, tolerance=prl_tol)
        self.wait(2.0)
        self.logger.info("Landing")
        self.set_led(effect='blink', r=255, g=165, b=0)
        telem = self.get_telemetry(frame_id="base_link")
        self.navigate(x=telem.x, y=telem.y, z=fall_z, speed=fall_speed, frame_id="base_link")
        self.wait(fall_time)
        self.navigate(x=telem.x, y=telem.y, z=fall_z, speed=0, frame_id="base_link")
        self.force_arm(False)
        self.logger.info("Landed")
        self.set_mode_service(custom_mode="AUTO.LAND")
        self.wait(0.2)
        while self.get_telemetry(frame_id="body").mode != "STABILIZED":
            self.set_mode_service(custom_mode="STABILIZED")
            self.wait(3)
        self.wait(1)
        self.force_arm(True)
        self.wait(1)
        while self.get_telemetry(frame_id="body").armed:
            self.force_arm(False)
            self.wait(3)
        self.set_led(r=0, g=255, b=0)
        # Threading control for async publisher
        self.publisher_thread = None
        self.stop_publisher = threading.Event()

        self.logger.info("Land requested")

    def engine_turnoff(self):
        print('MOCK')

    def scan_qr_code(self, timeout=5.0):
        return scan_qr(self.logger, timeout)


class FlightControllerMock:
    """Mock контроллер для отладки логики без реальных полетов"""
    
    def __init__(self, drone_name=None, logger=None):
        self.drone_name = drone_name or os.environ.get('DRONE_NAME', 'unknown_drone')
        self.logger = logger or setup_logging(self.drone_name)
        self.logger.info("🔧 MOCK MODE: Flight controller initialized")
    
    def wait(self, duration):
        """Имитация ожидания"""
        import time
        time.sleep(duration)
    
    def takeoff(self, z=1.1, delay=0.5, **kwargs):
        """Имитация взлета с задержкой для ручного перемещения"""
        self.logger.info(f"🚁 MOCK TAKEOFF to z={z:.2f}m")
        self.logger.info("⏰ Waiting 6 seconds - manually lift the drone now!")
        self.wait(6.0)
        self.logger.info("✅ Mock takeoff completed")
    
    def navigate_wait(self, x=0.0, y=0.0, z=0.0, yaw=float("nan"), speed=0.5, 
                     frame_id="", auto_arm=False, tolerance=0.2):
        """Имитация навигации с задержкой для ручного перемещения"""
        self.logger.info(f"🎯 MOCK NAVIGATE to x={x:.2f} y={y:.2f} z={z:.2f} in {frame_id}")
        self.logger.info("⏰ Waiting 6 seconds - manually move the drone now!")
        self.wait(6.0)
        self.logger.info("✅ Mock navigation completed")
    
    def navigate(self, x=0.0, y=0.0, z=0.0, yaw=float("nan"), speed=0.5, 
                frame_id="", auto_arm=False):
        """Имитация навигации без ожидания завершения"""
        self.logger.info(f"🎯 MOCK NAVIGATE (no wait) to x={x:.2f} y={y:.2f} z={z:.2f} in {frame_id}")
    
    def land(self, **kwargs):
        """Имитация посадки с задержкой для ручного перемещения"""
        self.logger.info("🛬 MOCK LANDING")
        self.logger.info("⏰ Waiting 6 seconds - manually land the drone now!")
        self.wait(6.0)
        self.logger.info("✅ Mock landing completed")
    
    def land_vertical(self, fall_speed=0.5, fall_time=3.0, final_z=-0.5):
        """Имитация вертикальной посадки"""
        self.logger.info("🛬 MOCK VERTICAL LANDING")
        self.logger.info("⏰ Waiting 6 seconds - manually land the drone vertically!")
        self.wait(6.0)
        self.logger.info("✅ Mock vertical landing completed")
    
    def navigate_and_land(self, x=0.0, y=0.0, z=1.0, takeoff_z=1.5, speed=0.5, 
                         frame_id="aruco_map", tolerance=0.2, hover_time=2.0, 
                         fall_speed=0.5, fall_time=3.0, final_z=-0.5):
        """Имитация полного цикла: взлет -> полет -> посадка"""
        self.logger.info(f"🚁 MOCK NAVIGATE_AND_LAND sequence to x={x:.2f} y={y:.2f} z={z:.2f}")
        
        # 1. Взлет
        self.logger.info(f"🚁 MOCK: Taking off to {takeoff_z}m")
        self.logger.info("⏰ Step 1/4: Manually lift the drone (6 seconds)")
        self.wait(6.0)
        
        # 2. Полет к цели
        self.logger.info(f"🎯 MOCK: Flying to target ({x:.2f}, {y:.2f}, {z:.2f})")
        self.logger.info("⏰ Step 2/4: Manually move drone to target position (6 seconds)")
        self.wait(6.0)
        
        # 3. Зависание
        self.logger.info(f"⏸️  MOCK: Hovering for {hover_time}s")
        self.wait(hover_time)
        
        # 4. Вертикальная посадка
        self.logger.info("🛬 MOCK: Vertical landing")
        self.logger.info("⏰ Step 3/4: Manually land the drone (6 seconds)")
        self.wait(6.0)
        
        self.logger.info("✅ Mock navigate_and_land sequence completed")
    
    def scan_qr_code(self, timeout=5.0):
        """Имитация сканирования QR кода"""
        self.logger.info("📱 MOCK QR SCAN")
        self.logger.info("⏰ Simulating QR scan...")
        self.wait(2.0)
        # Возвращаем фейковый QR код для тестирования
        mock_qr = ["MOCK_QR_CODE_123"]
        self.logger.info(f"📱 Mock QR result: {mock_qr}")
        return mock_qr
    
    def get_telemetry(self, frame_id="body"):
        """Имитация телеметрии"""
        class MockTelemetry:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.mode = "OFFBOARD"
                self.armed = True
        
        return MockTelemetry()
    
    def set_led(self, effect=None, r=0, g=0, b=0):
        """Имитация управления светодиодами"""
        if effect:
            self.logger.info(f"💡 MOCK LED: {effect} RGB({r},{g},{b})")
        else:
            self.logger.info(f"💡 MOCK LED: RGB({r},{g},{b})")
    
    def force_arm(self, arm_state):
        """Имитация арминга/разарминга"""
        state = "ARM" if arm_state else "DISARM"
        self.logger.info(f"🔒 MOCK {state}")
    
    def set_mode_service(self, custom_mode):
        """Имитация переключения режимов"""
        self.logger.info(f"🔄 MOCK MODE: {custom_mode}")


# Выбор реализации по переменной окружения
_impl = os.getenv('FLIGHT_IMPL', 'main').lower()
if _impl == 'custom':
    FlightController = FlightControllerCustom
elif _impl == 'mock':
    FlightController = FlightControllerMock
else:
    FlightController = FlightControllerMain 