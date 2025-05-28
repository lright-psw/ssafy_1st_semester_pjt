// src/components/Log.tsx
"use client";
import useJointWebSocket from "../hooks/useJointWebSocket";

export default function Log() {
  const { logs } = useJointWebSocket("ws://localhost:8080/ws");
  console.log("logs:", logs);

  return (
    <div className="w-full h-full flex flex-col">
      <div className="text-2xl font-bold mb-4">로그</div>
      <div
        className="flex-1 bg-[#f7f7f7] rounded-2xl p-8 overflow-y-auto border border-gray-200"
        style={{ minHeight: "500px", maxHeight: "600px" }}
      >
        {logs.length === 0
          ? <span className="text-gray-400">실시간 로그가 없습니다</span>
          : logs.map((log, idx) => (
              <div key={idx}>{log}</div>
            ))}
      </div>
    </div>
  );
}
