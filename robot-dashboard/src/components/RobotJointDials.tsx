"use client";
import { Doughnut } from "react-chartjs-2";
import { Chart, ArcElement, Tooltip } from "chart.js";
Chart.register(ArcElement, Tooltip);

type Props = {
    jointAngles: number[]; // 라디안 값
};

// 관절별 degree 범위
const jointConfigs = [
    { min: -90,    max: 90   },    // 관절1: -90~90도
    { min: 0.0001, max: 85   },    // 관절2: 0~85도
    { min: -10,    max: 95   },    // 관절3: -10~95도
    { min: -90,    max: 90   },    // 관절4: -90~90도
];

export default function RobotJointDials({ jointAngles }: Props) {
    return (
        <div className="flex gap-6">
            {jointAngles.map((rad, idx) => {
                const degree = rad * 180 / Math.PI;
                const { min, max } = jointConfigs[idx];
                // 도넛 차트에서 min~max의 범위 안에서 비율(0~1)
                const ratio = Math.max(0, Math.min(1, (degree - min) / (max - min)));
                // 실제 채워질 도수 (0~360에서 비율만큼)
                const doughnutValue = ratio * 360;

                return (
                    <div key={idx} className="flex flex-col items-center">
                        <Doughnut
                            data={{
                                datasets: [
                                    {
                                        data: [doughnutValue, 360 - doughnutValue],
                                        backgroundColor: ["#7baaf7", "#f1f1f1"],
                                        borderWidth: 0,
                                    },
                                ],
                            }}
                            options={{
                                cutout: "70%",
                                plugins: { legend: { display: false } },
                            }}
                            style={{ width: 80, height: 80 }}
                        />
                        <span className="mt-2 text-sm">
                            관절{idx + 1}: {degree.toFixed(4)}°
                        </span>
                    </div>
                );
            })}
        </div>
    );
}
