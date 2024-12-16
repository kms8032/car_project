import Jetson.GPIO as GPIO
import time
import threading
import cv2
import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
import numpy as np

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

# TensorRT 엔진 로드 함수
def load_engine(engine_path):
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

# TensorRT 실행 컨텍스트 초기화
def allocate_buffers(engine):
    inputs = []
    outputs = []
    bindings = []
    stream = cuda.Stream()

    for binding in engine:
        size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
        dtype = trt.nptype(engine.get_binding_dtype(binding))
        host_mem = cuda.pagelocked_empty(size, dtype)
        device_mem = cuda.mem_alloc(host_mem.nbytes)
        bindings.append(int(device_mem))
        if engine.binding_is_input(binding):
            inputs.append((host_mem, device_mem))
        else:
            outputs.append((host_mem, device_mem))

    return inputs, outputs, bindings, stream

def preprocess_frame(frame):
    resized = cv2.resize(frame, (150, 150))
    normalized = resized.astype(np.float32) / 255.0
    transposed = np.transpose(normalized, (2, 0, 1))  # (HWC -> CHW)
    return np.expand_dims(transposed, axis=0)

def infer(context, inputs, outputs, bindings, stream):
    [input_host, input_device] = inputs[0]
    [output_host, output_device] = outputs[0]

    cuda.memcpy_htod_async(input_device, input_host, stream)
    context.execute_async(bindings=bindings, stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(output_host, output_device, stream)
    stream.synchronize()

    return output_host

# TensorRT 엔진 로드
engine_path = "/home/kc/duty_cycle_model_fp16.trt"
engine = load_engine(engine_path)
context = engine.create_execution_context()
inputs, outputs, bindings, stream = allocate_buffers(engine)

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
        motor1_pwm.change_duty_cycle(20)
    elif direction == "backward":
        GPIO.output(DIR1, GPIO.LOW)
        motor1_pwm.change_duty_cycle(20)
    elif direction == "stop":
        motor1_pwm.change_duty_cycle(0)

# 실시간 추론 및 제어
try:
    cap = cv2.VideoCapture(1)
    motor1_pwm.start(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 영상 전처리
        input_frame = preprocess_frame(frame)
        np.copyto(inputs[0][0], input_frame.ravel())

        # TensorRT 추론 실행
        output = infer(context, inputs, outputs, bindings, stream)
        predicted_duty_cycle = output[0]

        # DC 모터 및 서보 모터 제어
        if predicted_duty_cycle < 7:
            control_dc_motor("backward")
            set_servo_duty_cycle(11)  # Right
            direction = "Backward + Right"
        elif predicted_duty_cycle > 9:
            control_dc_motor("forward")
            set_servo_duty_cycle(5)   # Left
            direction = "Forward + Left"
        else:
            control_dc_motor("forward")  # 직진 시에도 전진하도록 수정
            set_servo_duty_cycle(8)     # Straight
            direction = "Forward + Straight"

        # 카메라 영상에 정보 추가
        cv2.putText(
            frame, 
            f"Direction: {direction}", 
            (10, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, 
            (0, 255, 0), 
            2
        )
        cv2.putText(
            frame, 
            f"Duty Cycle: {predicted_duty_cycle:.2f}", 
            (10, 60), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, 
            (0, 255, 0), 
            2
        )

        # 화면에 표시
        cv2.imshow("Driving View", frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass
finally:
    motor1_pwm.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()

