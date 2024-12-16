import Jetson.GPIO as GPIO
import time
import threading
import cv2
import os
from pynput import keyboard

# GPIO 핀 설정
SERVO_PIN = 18
PWM1 = 31
DIR1 = 35

# GPIO 초기화
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(PWM1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(DIR1, GPIO.OUT, initial=GPIO.LOW)

# DC 모터 소프트웨어 PWM 클래스 정의
class SoftwarePWM:
    def __init__(self, pin, frequency=100):
        self.pin = pin
        self.frequency = frequency
        self.duty_cycle = 0
        self.running = False

    def start(self, duty_cycle):
        self.duty_cycle = duty_cycle
        self.running = True
        self.thread = threading.Thread(target=self._pwm_cycle)
        self.thread.start()

    def change_duty_cycle(self, duty_cycle):
        self.duty_cycle = duty_cycle

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        GPIO.output(self.pin, GPIO.LOW)

    def _pwm_cycle(self):
        period = 1.0 / self.frequency
        while self.running:
            on_time = period * (self.duty_cycle / 100.0)
            off_time = period - on_time
            if self.duty_cycle > 0:
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(on_time)
            if self.duty_cycle < 100:
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(off_time)

motor1_pwm = SoftwarePWM(PWM1, frequency=100)
camera_running = True
motor_state = "stop"
servo_duty_cycle = 8
key_state = "none"  # 현재 누르고 있는 키 상태

def control_dc_motor(direction):
    if direction == "forward":
        GPIO.output(DIR1, GPIO.HIGH)
        motor1_pwm.change_duty_cycle(80)
    elif direction == "backward":
        GPIO.output(DIR1, GPIO.LOW)
        motor1_pwm.change_duty_cycle(80)
    elif direction == "stop":
        motor1_pwm.change_duty_cycle(0)

def set_servo_duty_cycle(duty_cycle):
    period = 20
    on_time = period * (duty_cycle / 100.0)
    off_time = period - on_time
    GPIO.output(SERVO_PIN, GPIO.HIGH)
    time.sleep(on_time / 1000.0)
    GPIO.output(SERVO_PIN, GPIO.LOW)
    time.sleep(off_time / 1000.0)

def dc_motor_thread():
    global motor_state
    try:
        motor1_pwm.start(0)
        while camera_running:
            if motor_state == "forward":
                control_dc_motor("forward")
            elif motor_state == "backward":
                control_dc_motor("backward")
            elif motor_state == "stop":
                control_dc_motor("stop")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

def servo_motor_thread():
    global servo_duty_cycle
    try:
        while camera_running:
            set_servo_duty_cycle(servo_duty_cycle)
    except KeyboardInterrupt:
        pass

def camera_thread():
    global camera_running, key_state
    cap = cv2.VideoCapture(0)
    while not cap.isOpened():
        print("카메라를 열 수 없습니다. 다시 시도 중...")
        time.sleep(1)
        cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    base_path = "/home/kc/data4"
    key_paths = {
        "none": f"{base_path}/none",
        "z": f"{base_path}/z",
        "c": f"{base_path}/c"
    }

    # 각 키 상태에 해당하는 디렉토리가 없으면 생성
    for path in key_paths.values():
        try:
            os.makedirs(path, exist_ok=True)
        except Exception as e:
            print(f"디렉토리 생성 실패: {path} -> {e}")
            camera_running = False
            return

    count = {"none": 1, "z": 1, "c": 1}
    last_key_state = "none"  # 이전 키 상태를 추적

    try:
        while camera_running:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다. 다시 시도 중...")
                time.sleep(0.5)
                continue

            # 키 상태가 변경된 경우에만 로그
            if key_state != last_key_state:
                print(f"키 상태 변경: {last_key_state} -> {key_state}")
                last_key_state = key_state

            # 프레임에 키 상태를 표시
            font = cv2.FONT_HERSHEY_SIMPLEX
            position = (10, 50)
            font_scale = 1
            color = {"none": (255, 0, 0), "z": (0, 255, 0), "c": (0, 0, 255)}
            thickness = 2
            cv2.putText(frame, f"{key_state}", position, font, font_scale, color.get(key_state, (255, 255, 255)), thickness)

            # 현재 키 상태에 맞는 경로로 저장
            if key_state in key_paths:
                try:
                    filename = f"{key_paths[key_state]}/data_{count[key_state]:03d}.jpeg"
                    cv2.imwrite(filename, frame)
                    print(f"사진 저장: {filename} (Key State: {key_state})")
                    count[key_state] += 1
                except Exception as e:
                    print(f"이미지 저장 실패: {e}")

            time.sleep(0.2)  # 저장 간격 조정 (0.2초 이상으로 설정)
    finally:
        cap.release()
        print("카메라 종료.")

def on_press(key):
    global motor_state, servo_duty_cycle, camera_running, key_state
    try:
        if key.char == "s":
            motor_state = "forward"
            key_state = "s"
            print("모터: 전진")
        elif key.char == "w":
            motor_state = "backward"
            key_state = "w"
            print("모터: 후진")
        elif key.char == "z":
            servo_duty_cycle = 5
            key_state = "z"
            print("서보 모터: 왼쪽")
        elif key.char == "c":
            servo_duty_cycle = 11
            key_state = "c"
            print("서보 모터: 오른쪽")
        elif key.char == "q":
            camera_running = False
            print("프로그램 종료 중...")
            return False
    except AttributeError:
        pass

def on_release(key):
    global motor_state, servo_duty_cycle, key_state
    try:
        if key.char in ["w", "s"]:
            motor_state = "stop"
            key_state = "none"
            print("모터: 정지")
        elif key.char in ["z", "c"]:
            servo_duty_cycle = 8
            key_state = "none"
            print("서보 모터: 중앙으로 복귀")
    except AttributeError:
        pass

if __name__ == "__main__":
    try:
        threading.Thread(target=camera_thread, daemon=True).start()
        threading.Thread(target=dc_motor_thread, daemon=True).start()
        threading.Thread(target=servo_motor_thread, daemon=True).start()

        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    except KeyboardInterrupt:
        camera_running = False
    finally:
        motor1_pwm.stop()
        GPIO.cleanup()

