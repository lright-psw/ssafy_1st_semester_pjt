import { useEffect, useState } from "react";

export default function useJointWebSocket(url: string) {
  const [joint, setJoint] = useState([0, 0, 0, 0]);
  const [progress, setProgress] = useState(0);
  const [logs, setLogs] = useState<string[]>([]); // ★ 로그 state 추가

  useEffect(() => {
    let ws: WebSocket;
    let isMounted = true;
    function connect() {
      ws = new WebSocket(url);
      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          // joint 갱신
          if (data.joint_angle) {
            setJoint([
              data.joint_angle.motor_1,
              data.joint_angle.motor_2,
              data.joint_angle.motor_3,
              data.joint_angle.motor_4,
            ]);
            // 로그 메시지 만들기
            const msg = `[JointStateListener] joint_state 갱신됨: [${[
              data.joint_angle.motor_1,
              data.joint_angle.motor_2,
              data.joint_angle.motor_3,
              data.joint_angle.motor_4,
            ].join(", ")}]`;
            setLogs((prev) => [...prev.slice(-199), msg]);
          }
          if (typeof data.progress_percent === "number") {
            setProgress(data.progress_percent);
          }
        } catch (e) {
          setLogs((prev) => [...prev.slice(-199), `[ERROR] JSON parse 실패`]);
        }
      };
      ws.onclose = () => {
        if (isMounted) setTimeout(connect, 2000);
      };
    }
    connect();
    return () => {
      isMounted = false;
      ws && ws.close();
    };
  }, [url]);

  return { joint, progress, logs };
}
