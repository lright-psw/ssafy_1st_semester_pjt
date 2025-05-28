const menus = [
    'info', 'controller', 'log', 'AI'
];

export default function SideMenu({ onSelect, selected }: { onSelect: (menu: string) => void, selected: string }) {
    return (
        <div className="flex flex-col gap-2 w-full">
            {menus.map((item) => (
                <button
                    key={item}
                    onClick={() => onSelect(item)}
                    className={`py-2 rounded-lg border font-medium transition ${selected === item
                            ? "bg-blue-100 text-blue-600 border-blue-300"
                            : "bg-white text-gray-700 border-gray-200"
                        } hover:bg-blue-50`}
                >
                    {item}
                </button>
            ))}
        </div>
    );
}
