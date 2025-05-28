import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import threading

import pyrealsense2 as rs

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import torch

# ------ Flask MJPEG 서버 추가 ------
from flask import Flask, Response

app = Flask(__name__)
last_frame = [None]  # 리스트로 두면 참조 유지됨

@app.route('/stream')
def stream():
    def generate():
        while True:
            if last_frame[0] is not None:
                ret, jpeg = cv2.imencode('.jpg', last_frame[0])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            else:
                # 프레임 없을 때는 그냥 넘김
                pass
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    app.run(host="0.0.0.0", port=8082, threaded=True)

# -----------------------------------

class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolov5_node')

        # YOLOv5 모델 불러오기
        self.yolo_model = torch.hub.load(
            'ultralytics/yolov5', 'custom',
            path='/home/ssafy/ros2_ws/src/yolov5_model/best.pt'
        )

        # RealSense 파이프라인 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # ROS2 퍼블리셔
        self.detection_publisher = self.create_publisher(String, 'detection_results', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)
        self.bridge = CvBridge()

        self.detection_result = String()

        # 0.1초마다 콜백 실행
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            self.get_logger().warn("No frame received")
            return

        color_image = np.asanyarray(color_frame.get_data())
        results = self.yolo_model(color_image)

        detection_msgs = []

        for result in results.xyxy[0]:
            x1, y1, x2, y2 = map(int, result[:4])
            confidence = float(result[4])
            class_id = int(result[5])
            label = self.yolo_model.names[class_id]
            color_name = "unknown"
            box_color = (0, 255, 0)  # 기본 초록

            # 패널 객체면 HSV 색상 분석해서 label에 붙임
            if "panel" in label:
                object_roi = color_image[y1:y2, x1:x2]
                if object_roi.size > 0:
                    center_color, v_region, s_region = self.get_center_color(object_roi)
                    color_name = self.get_color_name_hsv(center_color, v_region, s_region)
                    # 박스 색상 지정 (BGR)
                    if color_name == "red":
                        box_color = (0, 0, 255)
                    elif color_name == "blue":
                        box_color = (255, 0, 0)
                    elif color_name == "white":
                        box_color = (200, 200, 200)

                label_text = f"{label}-{color_name}"
            else:
                label_text = f"{label}"

            # "unknown"이면 리스트에 **추가하지 않는다**
            if "unknown" in label_text:
                continue

            # 박스와 텍스트 표시
            cv2.rectangle(color_image, (x1, y1), (x2, y2), box_color, 2)
            cv2.putText(color_image, label_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

            detection_msgs.append(label_text)

        # --- 발행부 ---
        if detection_msgs:  # 값이 있을 때만 발행
            self.detection_result.data = '; '.join(detection_msgs)
            print(f"[PUBLISH] detection_results: {self.detection_result.data}")
            self.detection_publisher.publish(self.detection_result)
            ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            self.image_publisher.publish(ros_image_message)
        else:
            # 값이 없으면 아무 것도 발행하지 않음!
            pass

        # -------- MJPEG(Flask)용 프레임 최신값 저장 --------
        last_frame[0] = color_image.copy()
        # -----------------------------------------------

        # 디버그용 화면 표시
        cv2.imshow("YOLO Result", color_image)
        cv2.waitKey(1)

    def get_color_name_hsv(self, hsv_color, v_region=None, s_region=None):
        h, s, v = hsv_color
        bright = (v_region > 200) & (v_region < 246) if v_region is not None else None
        low_sat = s_region < 50 if s_region is not None else None
        too_bright = v_region > 249 if v_region is not None else None

        white_candidate = np.logical_and(bright, low_sat) if bright is not None and low_sat is not None else None
        white_ratio = np.sum(white_candidate) / max(1, len(v_region)) if white_candidate is not None else 0
        too_bright_ratio = np.sum(too_bright) / max(1, len(v_region)) if too_bright is not None else 0

        # DEBUG
        print(f"[DEBUG] HSV: ({h:.1f}, {s:.1f}, {v:.1f}), white_ratio={white_ratio:.2f}, bright_ratio={too_bright_ratio:.2f}")

        if white_ratio > 0.65 and too_bright_ratio < 0.2:
            return 'white'
        if v > 160 and s < 35:
            return 'white'
        elif (0 <= h <= 10 or 160 <= h <= 179) and s > 70:
            return 'red'
        elif 90 <= h <= 130 and s > 60:
            return 'blue'
        return 'unknown'

    def get_center_color(self, image):
        height, width = image.shape[:2]
        if height == 0 or width == 0:
            return np.array([0, 0, 0]), None, None
        center_y, center_x = height // 2, width // 2
        sample_size = max(2, min(width, height) // 4)
        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)
        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size // 2)
        center_region = image[start_y:end_y, start_x:end_x]
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        average_color = np.mean(hsv_region, axis=(0, 1))
        v_flat = hsv_region[:, :, 2].flatten()
        s_flat = hsv_region[:, :, 1].flatten()
        return average_color, v_flat, s_flat

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    # Flask 스트림 서버를 별도 스레드로 시작
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    rclpy.init(args=args)
    node = RealSenseYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
