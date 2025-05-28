import socket
import time
import gpiod

# ────────────── GPIO 핀 정의 ──────────────
DIR_PIN = 17
STEP_PIN = 27
ENABLE_PIN = 22
SERVO_PIN = 18

# ────────────── gpiod 초기화 ──────────────
chip = gpiod.Chip('gpiochip0')
dir_line = chip.get_line(DIR_PIN)
step_line = chip.get_line(STEP_PIN)
enable_line = chip.get_line(ENABLE_PIN)
servo_line = chip.get_line(SERVO_PIN)

dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)
servo_line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)

# ────────────── 스텝모터 제어 함수 ──────────────
def move_belt(steps=5000, speed=0.00001):
    print(f"[모터] {steps}스텝 이동")
    enable_line.set_value(0)  # 모터 Enable (Active LOW)
    # 방향: direction > 0 → LOW, direction < 0 → HIGH (반대로!)
    dir_line.set_value(0)
    for _ in range(steps):
        step_line.set_value(1)
        time.sleep(speed)
        step_line.set_value(0)
        time.sleep(speed)

    enable_line.set_value(1)  # 모터 비활성화
    print("[모터] 정지")


# ────────────── 서보모터 제어 함수 (PWM 직접) ──────────────
def set_servo(angle):
    print(f"[서보] {angle}도 위치로 이동")
    min_pulse = 0.0005  # 0.5ms
    max_pulse = 0.0025  # 2.5ms
    period = 0.02       # 20ms(=50Hz)
    pulse_width = min_pulse + (max_pulse - min_pulse) * (angle / 180.0)
    for _ in range(25):    # 약 0.5초간 PWM 신호 줌
        servo_line.set_value(1)
        time.sleep(pulse_width)
        servo_line.set_value(0)
        time.sleep(period - pulse_width)

def servo_left():
    print("[서보] 왼쪽(정상 패널, 0도)")
    set_servo(0)

def servo_right():
    print("[서보] 오른쪽(비정상 패널, 180도)")
    set_servo(120)

# ────────────── 메인 클라이언트 루프 ──────────────
def main():
    SERVER_IP = "192.168.110.107"  # main_prog.py 서버 IP
    SERVER_PORT = 65432
    
    set_servo(95)

    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print(f"서버({SERVER_IP}:{SERVER_PORT})에 연결 시도...")
            s.connect((SERVER_IP, SERVER_PORT))
            print("서버 연결 성공!")
            break
        except Exception as e:
            print(f"서버 연결 실패: {e}")
            time.sleep(2)

    try:
        while True:
            data = s.recv(1024)
            if not data:
                print("서버 연결 끊김")
                break
            command = data.decode('utf-8').strip()
            print(f"수신 데이터: {command}")

            tokens = command.split()
            if len(tokens) == 2:
                color_num = int(tokens[1])

                if color_num == 1:
                    print("정상 패널: 중간까지 이동 → 멈춤 → 서보동작 → 마저 이동")
                    set_servo(95)
                    move_belt(steps=3000, speed=0.00001)
                    time.sleep(0.2)
                    servo_left()
                    time.sleep(0.2)
                    move_belt(steps=12000, speed=0.00001)
                    time.sleep(0.2)
                elif color_num == 2:
                    print("비정상 패널: 벨트 전진 → 서보동작")
                    set_servo(95)
                    move_belt(steps=3000, speed=0.00001)
                    time.sleep(0.2)
                    servo_right()
                    time.sleep(0.2)
                    move_belt(steps=12000, speed=0.00001)
                    time.sleep(0.2)
                else:
                    print("알 수 없는 패널/색상")
                s.sendall(b"done\n")
            else:
                print("명령어 형식 오류:", command)
    except KeyboardInterrupt:
        print("사용자 종료")
    finally:
        # GPIO 해제
        enable_line.set_value(1)
        step_line.set_value(0)
        servo_line.set_value(0)
        dir_line.release()
        step_line.release()
        enable_line.release()
        servo_line.release()
        print("자원 정리 및 종료 완료")

if __name__ == "__main__":
    main()
