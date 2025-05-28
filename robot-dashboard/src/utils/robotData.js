// 여기에 fetch 등으로 실시간 데이터 가져오는 로직을 작성할 수 있습니다.
export function getMockRobotData() {
  return {
    jointAngles: [90, 30, 15, 120],
    speeds: [0.2, 0.3, 0.2, 0.1],
    positions: [{x:0.1, y:0.4}, {x:0.5, y:0.8}],
    totalTime: 0
  };
}
