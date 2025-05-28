// src/components/ProgressDonut.tsx
import { Doughnut } from "react-chartjs-2";
import { Chart, ArcElement, Tooltip } from "chart.js";
Chart.register(ArcElement, Tooltip);

export default function ProgressDonut({ percent }: { percent: number }) {
  const value = Math.max(0, Math.min(percent, 100));
  return (
    <div className="relative flex flex-col items-center"> {/* ← relative 추가!! */}
      <Doughnut
        data={{
          datasets: [
            {
              data: [value, 100 - value],
              backgroundColor: ["#7ae29e", "#f1f1f1"],
              borderWidth: 0,
            },
          ],
        }}
        options={{
          cutout: "70%",
          plugins: { legend: { display: false } },
        }}
        style={{ width: 110, height: 110 }}
      />
      <span
        className="absolute text-2xl font-bold"
        style={{ top: "50%", left: "50%", transform: "translate(-50%, -50%)" }} // ← 중앙 고정
      >
        {value}%
      </span>
    </div>
  );
}
