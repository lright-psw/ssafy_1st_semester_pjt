export default function YoloStream({ url }: { url: string }) {
    return (
      <div className="flex justify-center items-center w-full h-full bg-black rounded-xl">
        <img
          src={url}
          alt="YOLO Stream"
          className="rounded-xl border bg-black"
          style={{ width: "100%", height: "100%", objectFit: "contain", minHeight: 220 }}
        />
      </div>
    );
  }
  