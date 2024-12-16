import Jetson.GPIO as GPIO
import time
import threading

# GPIO 핀 설정 (BOARD 모드 기준)
PWM1 = 31    # 모터 1 속도 제어 (소프트웨어 PWM 핀)
DIR1 = 35    # 모터 1 방향 제어
PWM2 = 29    # 모터 2 속도 제어 (소프트웨어 PWM 핀)
DIR2 = 37    # 모터 2 방향 제어

# GPIO 초기화
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PWM1, GPIO.OUT, initial=GPIO.LOW)  # PWM 핀 초기화
GPIO.setup(DIR1, GPIO.OUT, initial=GPIO.LOW)  # DIR 핀 초기화
GPIO.setup(PWM2, GPIO.OUT, initial=GPIO.LOW)  # PWM 핀 초기화
GPIO.setup(DIR2, GPIO.OUT, initial=GPIO.LOW)  # DIR 핀 초기화

# 소프트웨어 PWM 클래스 정의
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

# 소프트웨어 PWM 객체 생성
motor1_pwm = SoftwarePWM(PWM1, frequency=100)
motor2_pwm = SoftwarePWM(PWM2, frequency=100)

# 모터 제어 함수 정의
def control_motor(pwm, dir_pin, direction, speed):
    """모터 제어 함수
    - pwm: PWM 객체
    - dir_pin: 방향 제어 핀
    - direction: "forward", "backward", "stop"
    - speed: 0~100 (듀티 사이클, %)
    """
    if direction == "forward":
        GPIO.output(dir_pin, GPIO.HIGH)  # 방향 제어 핀 HIGH
        pwm.change_duty_cycle(speed)    # 듀티 사이클 설정
        print(f"모터: FORWARD, Speed: {speed}%")
    elif direction == "backward":
        GPIO.output(dir_pin, GPIO.LOW)  # 방향 제어 핀 LOW
        pwm.change_duty_cycle(speed)    # 듀티 사이클 설정
        print(f"모터: BACKWARD, Speed: {speed}%")
    elif direction == "stop":
        pwm.change_duty_cycle(0)        # 듀티 사이클 0으로 설정 (정지)
        print("모터: STOP")

# 프로그램 종료 함수
def cleanup_and_exit():
    """모든 신호를 끄고 프로그램 종료"""
    print("\n프로그램 종료 중...")
    control_motor(motor1_pwm, DIR1, "stop", 0)  # 모터 1 정지
    control_motor(motor2_pwm, DIR2, "stop", 0)  # 모터 2 정지
    motor1_pwm.stop()
    motor2_pwm.stop()
    GPIO.cleanup()     # GPIO 정리
    print("모든 신호가 종료되었습니다.")
    exit()

# 메인 루프
try:
    motor1_pwm.start(0)  # 모터 1 PWM 초기화
    motor2_pwm.start(0)  # 모터 2 PWM 초기화

    print("키를 입력하세요: (a: 모터1 전진, s: 모터1 후진, d: 모터2 전진, f: 모터2 후진, w: 정지, q: 종료)")
    while True:
        user_input = input("명령 입력: ").strip().lower()

        if user_input == "a":
            print("모터 1: 전진")
            control_motor(motor1_pwm, DIR1, "forward", 50)  # 모터 1 전진 (50% 속도)
        elif user_input == "s":
            print("모터 1: 후진")
            control_motor(motor1_pwm, DIR1, "backward", 50)  # 모터 1 후진 (50% 속도)
        elif user_input == "d":
            print("모터 2: 전진")
            control_motor(motor2_pwm, DIR2, "forward", 50)  # 모터 2 전진 (50% 속도)
        elif user_input == "f":
            print("모터 2: 후진")
            control_motor(motor2_pwm, DIR2, "backward", 50)  # 모터 2 후진 (50% 속도)
        elif user_input == "w":
            print("모터 1 & 2: 정지")
            control_motor(motor1_pwm, DIR1, "stop", 0)  # 모터 1 정지
            control_motor(motor2_pwm, DIR2, "stop", 0)  # 모터 2 정지
        elif user_input == "q":
            cleanup_and_exit()  # 프로그램 종료
        else:
            print("잘못된 입력입니다. 다시 시도하세요.")

except KeyboardInterrupt:
    cleanup_and_exit()

