"use client";
import { useState } from "react";
import SideMenu from "../components/SideMenu";
import RobotJointDials from "../components/RobotJointDials";
import ProgressDonut from "../components/ProgressDonut";
import useJointWebSocket from "../hooks/useJointWebSocket";
import YoloStream from "../components/YoloStream";
import Log from "../components/Log";
import ChatGPTBox from "../components/ChatGPTBox";

export default function Home() {
  const [selectedMenu, setSelectedMenu] = useState("info");
  const { joint, progress } = useJointWebSocket("ws://localhost:8080/ws");

  return (
    <div className="min-h-screen bg-[#f7f8fa] flex items-center justify-center">
      <div className="flex w-[1200px] min-h-[700px] rounded-2xl shadow-lg bg-white border border-gray-200 overflow-hidden">
        {/* Side Menu */}
        <div className="w-[140px] border-r border-gray-200 bg-[#f8f8f8] flex flex-col items-center py-8">
          <SideMenu selected={selectedMenu} onSelect={setSelectedMenu} />
        </div>
        {/* Main Content */}
        <div className="flex-1 flex flex-col p-10">
          {selectedMenu === "info" && (
            <>
              {/* Title */}
              <div className="font-bold text-lg mb-2">현재 Dobot 관절</div>
              {/* Top - Robot Arm + Joints */}
              <div className="flex items-center mb-6">
                {/* 3D Robot Image */}
                <div className="flex flex-col items-center min-w-[180px] mr-8">
                  <img src="/dobot.png" className="w-[120px] h-[120px] mb-2 object-contain" alt="robot" />
                  <div className="text-gray-700 text-sm font-semibold">현재 로봇 팔 각도</div>
                </div>
                {/* 관절별 차트 */}
                <div className="flex flex-1 gap-8 justify-start">
                  <RobotJointDials jointAngles={joint} />
                </div>
              </div>
              {/* Divider */}
              <div className="h-[1px] bg-gray-200 w-full mb-5" />
              {/* Bottom - 2 Column Layout */}
              <div className="flex flex-1 gap-8">
                {/* Left: 작업 시간 계산 */}
                <div className="w-[320px] p-6 bg-[#f8f8f8] rounded-xl border border-gray-200 flex flex-col items-center">
                  <div className="font-semibold mb-4">작업 진행도</div>
                  <ProgressDonut percent={progress} />
                </div>
                {/* Right: YOLO 인식 화면 (실시간) */}
                <div className="flex-1 flex flex-col p-6 bg-[#f8f8f8] rounded-xl border border-gray-200">
                  <div className="font-semibold mb-4">YOLO 인식 화면</div>
                  <div className="flex-1">
                    <YoloStream url="http://192.168.110.107:8082/stream" />
                  </div>
                </div>
              </div>
            </>
          )}
          {selectedMenu === "controller" && (
            <div>미 완 성</div>
          )}
          {selectedMenu === "log" && (
            <Log />
          )}
          {selectedMenu === "AI" && (
            <ChatGPTBox />
          )}
        </div>
      </div>
    </div>
  );
}
