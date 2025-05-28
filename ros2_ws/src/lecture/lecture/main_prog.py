import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import time
import threading
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState

joint_state = [0.0, 0.0, 0.0, 0.0]  # motor_1,2,3,4
panel_progress = 0 # progress bar

# --- JointStateListener ---
class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.listener_callback, 10
        )
    def listener_callback(self, msg):
        global joint_state
        if len(msg.position) >= 4:
            joint_state[:] = [round(float(x), 4) for x in msg.position[:4]]
            # print("[JointStateListener] joint_state 갱신됨:", joint_state)  # dobot joint log

# --- PickAndPlace Node ---
class PickAndPlace(Node):
    def __init__(self, tasks_list, done_callback):
        super().__init__('pick_and_place')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=ReentrantCallbackGroup())
        self.cli = self.create_client(SuctionCupControl, 'dobot_suction_cup_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SuctionCupControl.Request()
        self.tasks_list = tasks_list
        self.goal_num = 0
        self.done_callback = done_callback

    def execute(self):
        if self.goal_num > len(self.tasks_list) - 1:
            self.get_logger().info('모든 PickAndPlace 동작 완료')
            if self.done_callback:
                self.done_callback()
            # executor에서 remove_node는 caller(PanelFlowNode)에서!
            return
        self.get_logger().info('*** TASK NUM ***: {0}'.format(self.goal_num))
        cmd = self.tasks_list[self.goal_num]
        if cmd[0] == "suction_cup":
            self.send_request(*cmd[1:])
            self.timer = self.create_timer(0.1, self.timer_callback, callback_group=ReentrantCallbackGroup())
            self.goal_num += 1
        elif cmd[0] == "move":
            self.send_goal(*cmd[1:])
            self.goal_num += 1

    def timer_callback(self):
        if self.srv_future.done():
            result = self.srv_future.result()
            self.get_logger().info('Result of suction_cup service call: {0}'.format(result))
            self.timer.cancel()
            self.execute()

    def send_request(self, enable_suction):
        self.req.enable_suction = enable_suction
        self.srv_future = self.cli.call_async(self.req)

    def send_goal(self, _target, _type):
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = _target
        goal_msg.motion_type = _type
        self.get_logger().info(f'>>> Move Goal: {_target}, type: {_type}')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.get_logger().info('>>> goal_response_callback called')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        self.get_logger().info(f'Action finished. Status={status}, Result={result}')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.execute()
        else:
            self.get_logger().error('Goal failed!')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

# --- TCP Server ---
class MultiTCPServer:
    def __init__(self, rpi_port=65432, robodk_port=20000, host='0.0.0.0'):
        self.rpi_conn = None
        self.robodk_conn = None
        self.rpi_thread = threading.Thread(target=self._listen_rpi, args=(host, rpi_port), daemon=True)
        self.robodk_thread = threading.Thread(target=self._listen_robodk, args=(host, robodk_port), daemon=True)
        self.rpi_thread.start()
        self.robodk_thread.start()
        print("서버: 라즈베리파이/로보DK 소켓 리스닝 시작!")

    def _listen_rpi(self, host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen()
            print(f"[서버] 라즈베리파이용 포트 {port}에서 연결 대기...")
            conn, addr = s.accept()
            self.rpi_conn = conn
            print(f"[서버] 라즈베리파이 연결됨: {addr}")

    def _listen_robodk(self, host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen()
            print(f"[서버] RoboDK용 포트 {port}에서 연결 대기...")
            conn, addr = s.accept()
            self.robodk_conn = conn
            print(f"[서버] RoboDK 연결됨: {addr}")

    def send_msg(self, msg):
        sent = False
        if self.rpi_conn:
            try:
                self.rpi_conn.sendall(msg.encode('utf-8'))
                print(f"[서버] 라즈베리파이로 송신: {msg}")
                sent = True
            except Exception as e:
                print(f"[서버] 라즈베리파이 송신 에러: {e}")
        if self.robodk_conn:
            try:
                self.robodk_conn.sendall(msg.encode('utf-8'))
                print(f"[서버] RoboDK로 송신: {msg}")
                sent = True
            except Exception as e:
                print(f"[서버] RoboDK 송신 에러: {e}")
        if not sent:
            print("[서버] 송신할 클라이언트가 없습니다.")

    def recv_from_rpi(self, timeout=0):
        if not self.rpi_conn:
            print("[서버] 라즈베리파이 연결 없음. 수신 건너뜀.")
            return None
        self.rpi_conn.settimeout(timeout if timeout > 0 else None)
        try:
            data = self.rpi_conn.recv(1024)
            if data:
                msg = data.decode('utf-8').strip()
                print(f"[서버] 라즈베리파이 수신: {msg}")
                return msg
        except Exception as e:
            print(f"[서버] 라즈베리파이 수신 에러: {e}")
        return None

    def close(self):
        if self.rpi_conn:
            self.rpi_conn.close()
        if self.robodk_conn:
            self.robodk_conn.close()

# --- PanelFlowNode ---
class PanelFlowNode(Node):
    def __init__(self, total_panels=6, executor=None):
        super().__init__('panel_flow_node')
        self.executor = executor
        self.panel_pick_poses = [
            [138.80, 212.49, -58.86, 0.0],
            [135.66, 151.69, -59.21, 0.0],
            [100.27, 212.73, -58.79, 0.0],
            [93.22, 146.99, -58.62, 0.0],
            [60.40, 211.41, -56.03, 0.0],
            [62.68, 157.07, -58.66, 0.0],
        ]
        self.panel_place_poses = [
            [155.23, 39.75, -2.92, 0.0]
        ]
        self.total_panels = total_panels
        self.cur_panel_idx = 0
        self.panel_ready = False
        self.waiting_for_detection = False   # *** 핵심 flag ***
        self.tcp_server = MultiTCPServer(rpi_port=65432, robodk_port=20000)
        self.sub = self.create_subscription(
            String, '/detection_results', self.detection_callback, 10
        )
        self.get_logger().info("패널 6개 작업 시작")
        self._wait_for_clients()
        self.start_next_panel()

    def _wait_for_clients(self):
        print("클라이언트가 연결될 때까지 대기 중...")
        # 둘 다 연결될 때까지 대기!
        while not (self.tcp_server.rpi_conn and self.tcp_server.robodk_conn):
            time.sleep(0.5)
        print("라즈베리파이 & RoboDK 둘 다 연결됨. 진행 시작.")


    def start_next_panel(self):
        if self.cur_panel_idx >= self.total_panels:
            self.get_logger().info("모든 패널 작업 완료!")
            self.tcp_server.close()
            rclpy.shutdown()
            return

        pick_pose = self.panel_pick_poses[self.cur_panel_idx]
        place_pose = self.panel_place_poses[0]
        self.get_logger().info(f"{self.cur_panel_idx+1}번 패널 픽&플레이스 시작")
        tasks = [
            ["move", [106.98, 183.12, 23.97, 0.0], 1],  # 어프로치/대기
            ["move", pick_pose, 1],                     # 픽 위치
            ["suction_cup", True],                      # 집기
            ["move", [106.98, 183.12, 23.97, 0.0], 1],  # 어프로치/이동
            ["move", [156.99, 52.91, 38.41, 0.0], 1],   # 어프로치/이동
            ["move", place_pose, 1],                    # 플레이스 위치
            ["suction_cup", False],                     # 놓기
            ["move", [156.99, 52.91, 38.41, 0.0], 1],   # 어프로치 복귀
            ["move", [106.98, 183.12, 23.97, 0.0], 1],  # 어프로치/대기
        ]
        def after_pick_and_place():
            self.panel_ready = False
            self.get_logger().info(f"{self.cur_panel_idx+1}번 패널 컨베이어에 올림. Dobot 치우는 중...")
            time.sleep(1.5)   # Dobot 완전히 화면 밖으로 이동 후 YOLO 대기 (시간 조절)
            self.waiting_for_detection = True
            self.get_logger().info("YOLO 패널 인식 대기 시작!")
            self.executor.remove_node(self.pick_and_place_node)
            self.pick_and_place_node.destroy_node()

        self.pick_and_place_node = PickAndPlace(tasks, after_pick_and_place)
        self.executor.add_node(self.pick_and_place_node)
        self.pick_and_place_node.execute()

    def detection_callback(self, msg):
        global panel_progress
        if not self.waiting_for_detection:
            return
        result = msg.data.strip()
        self.get_logger().info(f"Detection 결과 수신: {result}")
        self.waiting_for_detection = False  # 최초 1회만

        # 여러 개 결과가 오면 첫 번째만 사용
        first_result = result.split(';')[0].strip() if ';' in result else result

        try:
            panel, color = first_result.split('-')
        except Exception:
            self.get_logger().warn("Detection 결과 파싱 실패")
            return

        panel_num = 2 if panel == 'board_panel' else 1
        color_num = 1 if color in ['blue', 'white'] else 2

        # ---- 빨간색일 때 특수동작 ----
        if color == 'red':
            self.get_logger().info("빨간색 판넬이 감지됨! 특수 동작 수행 (여기에 코드)")

        msg_text = f"{panel_num} {color_num}"
        self.get_logger().info(f"라즈베리파이/RoboDK에 명령어 전송: {msg_text}")
        self.tcp_server.send_msg(msg_text)
        self.get_logger().info("라즈베리파이 처리 완료 신호 대기...")
        done = False
        if self.tcp_server.rpi_conn:
            while not done:
                recv = self.tcp_server.recv_from_rpi(timeout=0.5)
                if recv and recv.lower() == "done":
                    done = True
        else:
            print("[서버] 라즈베리파이 미연결: 'done' 신호 대기 생략.")
            done = True
        self.get_logger().info("라즈베리파이 완료 신호 수신! 다음 패널 이동")
        self.cur_panel_idx += 1
        panel_progress = int((self.cur_panel_idx / self.total_panels) * 100)
        time.sleep(1.0)
        self.start_next_panel()


# --- FastAPI + WebSocket 서버 추가 ---
from fastapi import FastAPI, WebSocket
import uvicorn
import asyncio

app = FastAPI()

@app.websocket("/ws")
async def ws_joint_data(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # print("[WebSocket] 전송:", joint_state)  # WebSocket trasmition log
            await websocket.send_json({
                "joint_angle": {
                    "motor_1": joint_state[0],
                    "motor_2": joint_state[1],
                    "motor_3": joint_state[2],
                    "motor_4": joint_state[3]
                },
                "progress_percent": panel_progress
            })
            await asyncio.sleep(0.1)
    except Exception as e:
        print("WebSocket 연결 종료:", e)

def run_fastapi():
    uvicorn.run(app, host="0.0.0.0", port=8080)

def main(args=None):
    rclpy.init(args=args)
    joint_listener = JointStateListener()
    executor = MultiThreadedExecutor()
    executor.add_node(joint_listener)
    panel_node = PanelFlowNode(total_panels=6, executor=executor)
    executor.add_node(panel_node)
    try:
        fastapi_thread = threading.Thread(target=run_fastapi, daemon=True)
        fastapi_thread.start()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        joint_listener.destroy_node()
        panel_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()