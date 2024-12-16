import Jetson.GPIO as GPIO
import time

# GPIO 핀 설정
SERVO_PIN = 18  # 사용할 GPIO 핀 번호 (BOARD 모드 기준)
PWM_FREQUENCY = 50  # 서보 모터의 PWM 주파수 (50Hz, 주기: 20ms)

# GPIO 초기화
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

def software_pwm(pin, duty_cycle, period=20):
    """
    소프트웨어 기반 PWM 신호 생성
    :param pin: GPIO 핀 번호
    :param duty_cycle: 듀티 사이클 (0~100%)
    :param period: PWM 주기 (ms 단위, 기본값 20ms)
    """
    on_time = period * (duty_cycle / 100.0)  # 켜져 있는 시간(ms)
    off_time = period - on_time  # 꺼져 있는 시간(ms)
    
    # 정확한 타이밍을 위해 perf_counter 사용
    start_time = time.perf_counter()
    GPIO.output(pin, GPIO.HIGH)
    while (time.perf_counter() - start_time) < (on_time / 1000.0):
        pass
    GPIO.output(pin, GPIO.LOW)
    while (time.perf_counter() - start_time) < (period / 1000.0):
        pass

def set_angle(angle):
    """
    서보 모터 각도를 설정 (소프트웨어 PWM 기반)
    :param angle: 설정할 각도 (75, 135, 180)
    """
    # 서보 모터의 동작 범위 확인 후 듀티 사이클 계산
    min_duty = 2.5  # 0도에 해당하는 듀티 사이클
    max_duty = 12.5  # 180도에 해당하는 듀티 사이클
    duty_cycle = min_duty + (angle / 180.0) * (max_duty - min_duty)
    for _ in range(50):  # 일정 시간 동안 신호 유지
        software_pwm(SERVO_PIN, duty_cycle)

try:
    while True:
        # 허용된 각도만 입력 가능
        allowed_angles = [75, 135, 180]
        print(f"사용 가능한 각도: {allowed_angles}")
        angle = int(input("목표 각도를 입력하세요: "))

        if angle in allowed_angles:
            set_angle(angle)
            print(f"서보 모터를 {angle}°로 설정했습니다.")
        else:
            print(f"입력한 각도는 허용되지 않습니다. {allowed_angles} 중에서 선택하세요.")
except KeyboardInterrupt:
    print("\n프로그램 종료")
finally:
    GPIO.cleanup()

