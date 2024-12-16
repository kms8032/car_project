import Jetson.GPIO as GPIO
import time
import threading
import cv2
import os
import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
import numpy as np

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

# TensorRT 관련 함수 정의
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def load_engine(engine_path):
    with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

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
engine_path = "/home/kc/duty_cycle_mobilenet_model.trt"
engine = load_engine(engine_path)
context = engine.create_execution_context()
inputs, outputs, bindings, stream = allocate_buffers(engine)

def control_dc_motor(direction):
    if direction == "forward":
        GPIO.output(DIR1, GPIO.HIGH)
        motor1_pwm.change_duty_cycle(30)
    elif direction == "backward":
        GPIO.output(DIR1, GPIO.LOW)
        motor1_pwm.change_duty_cycle(30)
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
    global camera_running, motor_state, servo_duty_cycle
    cap = cv2.VideoCapture(0)
    while not cap.isOpened():
        print("카메라를 열 수 없습니다. 다시 시도 중...")
        time.sleep(1)
        cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    try:
        while camera_running:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다. 다시 시도 중...")
                time.sleep(0.5)
                continue

            # TensorRT 추론
            input_frame = preprocess_frame(frame)
            np.copyto(inputs[0][0], input_frame.ravel())
            output = infer(context, inputs, outputs, bindings, stream)
            predicted_duty_cycle = output[0]

            # DC 모터 및 서보 모터 제어
            if predicted_duty_cycle <= 2.5 :
                control_dc_motor("backward")
                servo_duty_cycle = 11
                direction = "Backward + Right"
            elif predicted_duty_cycle >= 12.5:
                control_dc_motor("backward")
                servo_duty_cycle = 5
                direction = "backward + Left"
            else:
                control_dc_motor("backward")
                servo_duty_cycle = 8
                direction = "Forward + Straight"

            # 디버그 정보 표시
            font = cv2.FONT_HERSHEY_SIMPLEX
            position = (10, 50)
            font_scale = 1
            color = (0, 255, 0)
            thickness = 2
            cv2.putText(frame, f"Direction: {direction}", position, font, font_scale, color, thickness)
            cv2.imshow("Autonomous Driving", frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord("q"):
                camera_running = False
                break

    finally:
        cap.release()
        print("카메라 종료.")

if __name__ == "__main__":
    try:
        threading.Thread(target=camera_thread, daemon=True).start()
        threading.Thread(target=dc_motor_thread, daemon=True).start()
        threading.Thread(target=servo_motor_thread, daemon=True).start()

        while camera_running:
            time.sleep(1)

    except KeyboardInterrupt:
        camera_running = False
    finally:
        motor1_pwm.stop()
        GPIO.cleanup()
