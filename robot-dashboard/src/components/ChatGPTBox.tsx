// src/components/ChatGPTBox.tsx
"use client";
import { useState, useRef, useEffect } from "react";

type ChatMessage = {
  role: "user" | "assistant";
  content: string;
};

export default function ChatGPTBox() {
  const [input, setInput] = useState("");
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [loading, setLoading] = useState(false);
  const chatEndRef = useRef<HTMLDivElement>(null);

  // 스크롤 항상 아래로
  useEffect(() => {
    chatEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  async function handleSend() {
    if (!input.trim() || loading) return;

    // 내 메시지 추가
    setMessages((msgs) => [...msgs, { role: "user", content: input }]);
    setLoading(true);

    // OpenAI API 요청 (환경변수에 NEXT_PUBLIC_OPENAI_API_KEY 필요!)
    const apiKey = process.env.NEXT_PUBLIC_OPENAI_API_KEY;
    try {
      const res = await fetch("https://api.openai.com/v1/chat/completions", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Authorization: `Bearer ${apiKey}`,
        },
        body: JSON.stringify({
          model: "gpt-4o-mini", // 무조건 고정 절대 변경 금지
          messages: [
            ...messages.map(m => ({ role: m.role, content: m.content })),
            { role: "user", content: input },
          ],
        }),
      });
      const data = await res.json();
      const gptMsg = data.choices?.[0]?.message?.content?.trim() || "답변 오류!";
      setMessages((msgs) => [
        ...msgs,
        { role: "assistant", content: gptMsg },
      ]);
    } catch (e) {
      setMessages((msgs) => [
        ...msgs,
        { role: "assistant", content: "GPT 응답 실패!" },
      ]);
    }
    setInput("");
    setLoading(false);
  }

  // 엔터 입력 지원
  function handleKeyDown(e: React.KeyboardEvent<HTMLInputElement>) {
    if (e.key === "Enter") handleSend();
  }

  return (
    <div className="w-full h-full flex flex-col">
      <div className="text-2xl font-bold mb-4">AI</div>
      <div className="flex-1 bg-[#f7f7f7] rounded-2xl p-8 mb-4 overflow-y-auto border border-gray-200"
           style={{ minHeight: "400px", maxHeight: "500px" }}>
        {messages.length === 0 && (
          <div className="text-gray-400 text-center mt-16">아래에 메시지를 입력해보세요!</div>
        )}
        {messages.map((msg, idx) => (
          <div key={idx} className={`mb-4 flex ${msg.role === "user" ? "justify-end" : "justify-start"}`}>
            <div className={`px-4 py-2 rounded-xl shadow 
              ${msg.role === "user" ? "bg-white border ml-auto" : "bg-[#e2f1fa] border border-[#bae2fa] mr-auto"}
              `}>
              <span className="text-xs text-gray-400 font-bold mr-2">{msg.role === "user" ? "[본인]" : "[GPT]"}</span>
              <span className="whitespace-pre-line">{msg.content}</span>
            </div>
          </div>
        ))}
        <div ref={chatEndRef} />
      </div>
      <div className="flex items-center w-full bg-white border-2 border-black rounded-lg p-4">
        <input
          type="text"
          value={input}
          onChange={e => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="원하는 내용을 입력하세요"
          className="flex-1 outline-none bg-transparent text-xl px-4"
        />
        <button
          onClick={handleSend}
          disabled={loading || !input.trim()}
          className="w-[56px] h-[56px] bg-white text-3xl flex items-center justify-center border-none cursor-pointer"
        >
          <span style={{ fontSize: "2.2rem" }}>{loading ? "..." : "➤"}</span>
        </button>
      </div>
    </div>
  );
}
