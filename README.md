## 여는 말
#### 🙆 본 프로젝트는 RoboDk를 이용한 디지털 트윈과Ros2를 활용한 Dobot/라즈베리파이 연동을 통해 실제 TV 생산 공정이 진행되는 과정을 구현하는 프로젝트입니다.
![alt text](https://dh2zq2763s2kl.cloudfront.net/robot/img/Dobot-Magician-robot.png)

---
### 사용한 기술 스택
<br/>

![Static Badge](https://img.shields.io/badge/dobot-blue?style=flat&logo=ros&logoColor=white)<!-- dobot -->
&nbsp;
![Static Badge](https://img.shields.io/badge/raspberrypi-%23A22846?style=flat&logo=raspberrypi&logoColor=white)<!-- 라즈베리파이 -->
&nbsp;
![Static Badge](https://img.shields.io/badge/realsense-%230071C5?style=flat&logo=intel&logoColor=white)<!-- realsense -->
&nbsp;
![Static Badge](https://img.shields.io/badge/roboflow-%236706CE?style=flat&logo=roboflow&logoColor=white)<!-- roboflow -->


![Static Badge](https://img.shields.io/badge/python-python?style=flat&logo=python&logoColor=FFFFFF&color=%233776AB)<!-- 파이썬 -->
&nbsp;
![Static Badge](https://img.shields.io/badge/html-html?style=flat&logo=html5&logoColor=FFFFFF&color=%23E34F26)<!-- HTML -->
&nbsp;
![Static Badge](https://img.shields.io/badge/css-css?style=flat&logo=css&logoColor=FFFFFF&color=%23663399)<!-- CSS -->
&nbsp;
![Static Badge](https://img.shields.io/badge/tailwindcss-%2306B6D4?style=flat&logo=tailwindcss&logoColor=white)<!-- tailwindcss -->
&nbsp;
![Static Badge](https://img.shields.io/badge/javascript-javascript?style=flat&logo=javascript&logoColor=%23F7DF1E&color=555555)<!-- 자바스크립트 -->

![Static Badge](https://img.shields.io/badge/fastapi-%23009688?style=flat&logo=fastapi&logoColor=white)<!-- fastapi -->
&nbsp;
![Static Badge](https://img.shields.io/badge/flask-%23000000?style=flat&logo=flask&logoColor=white)<!-- flask -->
&nbsp;
![Static Badge](https://img.shields.io/badge/next.js-%23000000?style=flat&logo=nextdotjs&logoColor=white)<!-- next.js -->

![Static Badge](https://img.shields.io/badge/ros2-%2322314E?style=flat&logo=ros&logoColor=white)<!-- ros2 -->
&nbsp;
![Static Badge](https://img.shields.io/badge/opencv-%235C3EE8?style=flat&logo=opencv&logoColor=white)<!-- opencv -->
&nbsp;
![Static Badge](https://img.shields.io/badge/yolo-%23111F68?style=flat&logo=yolo&logoColor=white)<!-- yolo -->

---

## 👋 프로젝트 개요
### 개발 동기
최근 제조업계에서는 인력 부족, 생산 효율 저하, 품질 편차 등 다양한 문제에 직면하고 있습니다. 특히 단순 반복 작업이 많은 생산 공정에서 이러한 한계가 두드러지며, 이를 해결하기 위해서는 새로운 접근이 필요하다고 느꼈습니다.
저희는 로봇과 인공지능 기술이 이런 문제를 효과적으로 개선할 수 있다고 생각했습니다. 따라서 수업이나 기사에서 자주 접하는 스마트 팩토리가 실제로 어떻게 적용되는지 직접 확인해 보고자 프로젝트를 기획하게 되었습니다.
 

### 주제 선정이유
최근 AI의 발전으로 인해 기존에는 인력에만 의존하던 제조 공정이 로봇과 기계학습을 활용하는 스마트 팩토리로 빠르게 변화하고 있습니다. 이러한 변화는 생산 효율성과 품질 향상에 큰 영향을 미치고 있습니다. 저희는 현재 로봇에 대해 학습하고 있는 학생으로서, 스마트 팩토리의 구조와 운영 방식에 대해 더 깊이 이해하고자 했습니다. 이에 본 프로젝트의 주제를 'TV 생산 공정'으로 선정하여, 스마트 팩토리 내에서 실제로 로봇과 AI가 어떻게 적용되는지 자세히 조사해 보고자 합니다.

## 프로젝트 소개
![alt text](<Screenshot from 2025-05-27 14-02-55.png>)

- `서버`, `클라이언트`, `미들웨어`로 구성됩니다.

### 서버
서버는 `TCPSocket`, `WebSocket`, `Flask`, `fastapi`로 구성됩니다.

#### TCPSocket
 - TCP Socket은 roboDk와 라즈베리파이(컨베이어 벨트) 2개에 클라이언트에 재작성된 메세지를 송신하며 해당 내용을 통해 클라이언트들은 주어진 동작을 수행하게 됩니다.

#### WebSocket
 - Dobot의 Joint값(총 4개)을 실시간으로 프론트로 전송하는 역할을 수행합니다. 

#### Flask
 - `MJPEG`형식을 통해 realsense에서 YOLO를 통해 판넬을 감지하고 있는 실시간 영상을 압축하여 프론트에 전송하고 이를 압축 해제(Decompression)하여 웹 페이지에 보여줍니다.

#### fastapi
 - 작업이 총 6번 진행되는데 작업이 얼만큼 진행되었는지를 번호를 보내주는 역할을 수행합니다.

### 클라이언트
클라이언트는  `Dobot/라즈베리파이(컨베이어벨트)`, `roboDk`, `웹(next.js)`로 구성됩니다.

#### Dobot/라즈베리파이(컨베이어벨트)
 - ros2를 이용하여 dobot을 움직이고, 서버에서 보낸 `data`를 라즈베리파이에서 수신하고 받은 명령에 따른 동작을 수행합니다.

#### roboDk
 - 서버에서 보낸 `data`를 roboDk에서 TCPSocket을 통해 수신하고 받은 명령에 따른 동작을 수행합니다.

#### 웹(next.js) 
 - 1개에 서버가 아닌 3개에 서버에서 데이터를 보내게 되는데 각각에 서버에서 보낸 데이터를 가공하여 시각화해 사용자에게 보여줍니다.

### 미들웨어
미들웨어는 `ros2`로 구성되어있습니다.

#### ros2
 - ros2는 서버와 클라이언트(dobot)/yolo모델 사이에서 동작을 지원하거나 dobot의 joint와 같은 데이터를 중계하는 역할을 수행합니다.

## 시연 영상
[시연 영상](https://youtu.be/BOi_AOf3Yk0)

1. 먼저 `https://github.com/lright-psw/ssafy_1st_semester_pjt.git`에서 파일을 받아온다.
2. 연결할 라즈베리파이에 `ros2_ws/src/lecture/lecture/rp5_client.py`를 이동시킨다.
3. `npm i ws`로 웹소켓을 다운 받는다.
4. 서버를 구동시킬 컴퓨터에 접속해 `node.js`와 `python`을 다운한다
5. 컴퓨터에 [roboDk](https://robodk.com/ko/)를 다운 받는다.
6. robot-dashboard 폴더에 들어가 `Terminal` 혹은 `Powershell`을 키고 `npm i`를 입력한다.
7. roboDK 폴더 내부 압축파일에 압축을 풀고 `TV_factory_sequence.rdk`를 실행시키고 내부 파이썬 파일을 실행시킨다.
8. `realsense`와 `Dobot Magician`을 서버를 구동시킬 컴퓨터와 연결시킨다.
9. [`ros2 humble`](https://docs.ros.org/en/humble/index.html)을 서버를 구동시킬 컴퓨터에 설치한다.
10. ros2_ws로 이동하여 `colcon build`입력 후 `.bashrc`를 수정하고 `source ~/.bashrc`를 입력한다.
11. SERVER, CLIENT, 라즈베리파이를 각각 실행시킨다.
  - SERVER 실행 명령어
    - 서버 구동 명령어 `ros2 run lecture main_prog`
    - realsense/yolo 구동 명령어 `ros2 run lecture realsensewithYolov5`
  - CLIENT 실행 명령어 `npm run dev`
  - 라즈베리파이 실행 명령어 `python3 rp5_client.py`


## connet Us

