import Jetson.GPIO as GPIO
import time
import threading
from pynput import keyboard

# GPIO 핀 설정
SERVO_PIN = 18  # 서보 모터 핀 (BOARD 모드 기준)
PWM1 = 31  # 모터 1 속도 제어
DIR1 = 35  # 모터 1 방향 제어

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

# DC 모터 객체 생성
motor1_pwm = SoftwarePWM(PWM1, frequency=100)

# 상태 변수
servo_duty_cycle = 8  # 초기 듀티 사이클
motor_state = "stop"

# 서보 모터 제어 함수 (듀티 사이클 사용)
def set_servo_duty_cycle(duty_cycle):
    period = 20  # PWM 주기(ms)
    on_time = period * (duty_cycle / 100.0)
    off_time = period - on_time
    GPIO.output(SERVO_PIN, GPIO.HIGH)
    time.sleep(on_time / 1000.0)
    GPIO.output(SERVO_PIN, GPIO.LOW)
    time.sleep(off_time / 1000.0)

# DC 모터 제어 함수
def control_dc_motor(direction):
    if direction == "forward":
        GPIO.output(DIR1, GPIO.HIGH)
        motor1_pwm.change_duty_cycle(50)
    elif direction == "backward":
        GPIO.output(DIR1, GPIO.LOW)
        motor1_pwm.change_duty_cycle(50)
    elif direction == "stop":
        motor1_pwm.change_duty_cycle(0)

# DC 모터 쓰레드
def dc_motor_thread():
    global motor_state
    try:
        motor1_pwm.start(0)
        while True:
            if motor_state == "forward":
                control_dc_motor("forward")
            elif motor_state == "backward":
                control_dc_motor("backward")
            elif motor_state == "stop":
                control_dc_motor("stop")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

# 서보 모터 쓰레드
def servo_motor_thread():
    global servo_duty_cycle
    try:
        while True:
            set_servo_duty_cycle(servo_duty_cycle)
    except KeyboardInterrupt:
        pass

# 키 입력 핸들러
def on_press(key):
    global servo_duty_cycle, motor_state
    try:
        if key.char == "w":
            motor_state = "forward"
        elif key.char == "s":
            motor_state = "backward"
        elif key.char == "a":
            servo_duty_cycle = 1.5  # 듀티 사이클 2.5%
        elif key.char == "d":
            servo_duty_cycle = 11.5  # 듀티 사이클 12.5%
    except AttributeError:
        pass

def on_release(key):
    global servo_duty_cycle, motor_state
    try:
        if key.char in ["w", "s"]:
            motor_state = "stop"
        elif key.char in ["a", "d"]:
            servo_duty_cycle = 8  # 듀티 사이클 7.5%로 복귀
    except AttributeError:
        pass

# 메인 함수
if __name__ == "__main__":
    try:
        # DC 모터 및 서보 모터 쓰레드 시작
        threading.Thread(target=dc_motor_thread, daemon=True).start()
        threading.Thread(target=servo_motor_thread, daemon=True).start()

        # 키보드 입력 감지
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    except KeyboardInterrupt:
        motor1_pwm.stop()
        GPIO.cleanup()

