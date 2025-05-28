import time
import gpiod

# ───────────────────── GPIO 초기화 ──────────────────────
dir_pin = 17
step_pin = 27
enable_pin = 22
servo_pin = 18  # 서보 핀

chip = gpiod.Chip('gpiochip0')

dir_line = chip.get_line(dir_pin)
step_line = chip.get_line(step_pin)
enable_line = chip.get_line(enable_pin)
servo_line = chip.get_line(servo_pin)

dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)
servo_line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)

# ───────────────────── 동작 함수 ──────────────────────
def move_belt(steps=5000, speed=0.00001):
    print(f"[모터] {steps}스텝 이동")
    enable_line.set_value(0)  # 모터 Enable (Active LOW)
    dir_line.set_value(0)     # 방향 고정(CW)
    for _ in range(steps):
        step_line.set_value(1)
        time.sleep(speed)
        step_line.set_value(0)
        time.sleep(speed)
    enable_line.set_value(1)
    print("[모터] 정지")

def set_servo(angle):
    print(f"[서보] {angle}도 위치로 이동")
    min_pulse = 0.0005
    max_pulse = 0.0025
    period = 0.02
    pulse_width = min_pulse + (max_pulse - min_pulse) * (angle / 180.0)
    for _ in range(50):    # 1초간 PWM 신호 줌 (25→50, 필요시 조정)
        servo_line.set_value(1)
        time.sleep(pulse_width)
        servo_line.set_value(0)
        time.sleep(period - pulse_width)

def test_sequence(cmd):
    if cmd == 'c':
        print("정상 진행(벨트만 200스텝)")
        move_belt(steps=5000, speed=0.00001)
    elif cmd == 'w':
        print("왼쪽")
        set_servo(95)
    elif cmd == 'e':
        print("오른쪽")
        set_servo(120)
    elif cmd == 'q':
        print("종료 요청")
        return False
    else:
        print("알 수 없는 명령:", cmd)
    return True

# ───────────────────── 메인 루프 ──────────────────────
try:
    while True:
        print("명령 입력 대기중...")
        cmd = input("\n명령 입력 (c=정상, w=왼쪽, e=오른쪽, q=종료): ").strip().lower()
        if not test_sequence(cmd):
            break

except KeyboardInterrupt:
    print("사용자 종료")
finally:
    enable_line.set_value(1)
    step_line.set_value(0)
    servo_line.set_value(0)
    dir_line.release()
    step_line.release()
    enable_line.release()
    servo_line.release()
    print("자원 정리 및 종료 완료")
