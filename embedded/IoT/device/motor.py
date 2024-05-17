mport
RPi.GPIO as GPIO
import time

# GPIO 핀 번호 설정
IN1_PIN = 17  # IN1 핀
IN2_PIN = 18  # IN2 핀

# 모터 제어 핀 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1_PIN, GPIO.OUT)
GPIO.setup(IN2_PIN, GPIO.OUT)


# 모터 초기화 함수
def motor_init():
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.LOW)

    # 모터를 전진시키는 함수
def motor_forward():
        GPIO.output(IN1_PIN, GPIO.HIGH)
        GPIO.output(IN2_PIN, GPIO.LOW)

        # 모터를 후진시키는 함수
def motor_backward():
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.HIGH)

try:
    motor_init()  # 모터 초기화
    while True:
        motor_forward()  # 모터를 전진
        time.sleep(2)  # 2초 대기
        motor_backward()  # 모터를 후진
        time.sleep(2)  # 2초 대기

except KeyboardInterrupt:
    # Ctrl+C를 누르면 프로그램 종료
    GPIO.cleanup()
