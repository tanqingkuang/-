"""GUI 主题与轻量复用控件。注意：不承载主窗口业务流程。"""

from __future__ import annotations

from PySide6.QtCore import QPoint, Signal
from PySide6.QtGui import QAction, QColor
from PySide6.QtWidgets import QMenu, QPushButton, QWidget

class Theme:
    """单个 UI 主题的集中配色。注意：主题切换时画布和控件共用这些颜色。"""

    def __init__(
        self,
        *,
        bg: str,
        panel: str,
        ink: str,
        muted: str,
        line: str,
        canvas: str,
        grid: str,
        route: str,
        leader: str,
        wingman: str,
        link: str,
        warn: str,
        accent: str,
        field: str,
    ) -> None:
        """初始化 Theme 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        # 把传入的颜色字符串统一转成 QColor，供样式表与自绘画布复用。
        self.bg = QColor(bg)  # 窗口背景
        self.panel = QColor(panel)  # 面板/分组背景
        self.ink = QColor(ink)  # 主文字色
        self.muted = QColor(muted)  # 次要文字/轴标注
        self.line = QColor(line)  # 边框/分隔线
        self.canvas = QColor(canvas)  # 画布底色
        self.grid = QColor(grid)  # 每五格出现的主网格线
        self.minor_grid = QColor(grid)  # 次网格沿用主网格色相，只降低透明度
        self.minor_grid.setAlphaF(0.48)
        self.route = QColor(route)  # 参考航线
        self.leader = QColor(leader)  # 长机
        self.wingman = QColor(wingman)  # 僚机
        self.link = QColor(link)  # 正常链路
        self.warn = QColor(warn)  # 异常/告警
        self.accent = QColor(accent)  # 强调色（滑块/选框等）
        self.field = QColor(field)  # 输入控件背景


THEMES = {
    "light": Theme(
        bg="#eaf2f8",
        panel="#edf6fd",
        ink="#17202a",
        muted="#667085",
        line="#cfdae6",
        canvas="#e2edf6",
        grid="#c5d4e2",
        route="#94a3b8",
        leader="#2563eb",
        wingman="#7c3aed",
        link="#0891b2",
        warn="#b45309",
        accent="#0f766e",
        field="#f4f9fe",
    ),
    "dark": Theme(
        bg="#0e141b",
        panel="#151d26",
        ink="#e7edf4",
        muted="#94a3b8",
        line="#2a3644",
        canvas="#101923",
        grid="#243141",
        route="#64748b",
        leader="#60a5fa",
        wingman="#c084fc",
        link="#22d3ee",
        warn="#f59e0b",
        accent="#14b8a6",
        field="#0f1720",
    ),
}


class SelectButton(QPushButton):
    """基于按钮的选项选择器。注意：弹出菜单位置由控件主动控制。"""

    currentIndexChanged = Signal()

    def __init__(self, min_width: int, popup_side: str = "below", parent: QWidget | None = None) -> None:
        """初始化 SelectButton 实例，建立后续运行所需状态。注意：构造阶段不应启动耗时流程。"""
        super().__init__(parent)
        # (显示文本, 附加数据) 列表与当前选中索引，-1 表示尚无选中项。
        self._items: list[tuple[str, object | None]] = []
        self._index = -1
        self._menu = QMenu(self)
        # 弹出方向："below" 在按钮下方，"right" 在按钮右侧（用于侧栏窄面板）。
        self._popup_side = popup_side
        self.setObjectName("selectButton")
        self.setMinimumWidth(min_width)
        # 点击按钮即弹出菜单；菜单收起时复位按钮按下态。
        self.clicked.connect(self.show_menu)
        self._menu.aboutToHide.connect(lambda: self.setDown(False))

    def clear(self) -> None:
        """清空全部选项并复位选中态。注意：不发信号，供动态重填选项前调用。"""
        self._items = []
        self._index = -1
        self.setText("  ▾")

    def currentIndex(self) -> int:
        """返回当前选中项索引。注意：无选项时返回 -1。"""
        return self._index

    def addItem(self, text: str, data: object | None = None) -> None:
        """向控件添加一个选项。注意：选项文本和附加数据需保持对应。"""
        self._items.append((text, data))
        # 添加第一项时自动选中它，但不触发信号（避免初始化期误触回调）。
        if self._index == -1:
            self.setCurrentIndex(0, emit=False)

    def addItems(self, texts: list[str]) -> None:
        """批量添加控件选项。注意：按输入顺序追加。"""
        for text in texts:
            self.addItem(text, text)

    def setCurrentIndex(self, index: int, *, emit: bool = True) -> None:
        """设置当前选中项。注意：索引越界时不应破坏控件状态。"""
        # 越界索引直接忽略，保持原状态不被破坏。
        if index < 0 or index >= len(self._items):
            return
        # 选中项未变化则不重复刷新文本/不重复发信号。
        if index == self._index:
            return
        self._index = index
        # 按钮文本带下三角符号提示这是可下拉的选择器。
        self.setText(f"{self._items[index][0]}  ▾")
        if emit:
            self.currentIndexChanged.emit()

    def setCurrentText(self, text: str, *, emit: bool = True) -> None:
        """按文本设置当前选项。注意：文本不存在时会追加为新选项。"""
        normalized = str(text)
        for index, (item_text, _) in enumerate(self._items):
            if item_text == normalized:
                self.setCurrentIndex(index, emit=emit)
                return
        self._items.append((normalized, normalized))
        self.setCurrentIndex(len(self._items) - 1, emit=emit)

    def currentText(self) -> str:
        """返回当前选项文本。注意：无选项时返回空字符串。"""
        if self._index < 0:
            return ""
        return self._items[self._index][0]

    def currentData(self) -> object | None:
        """返回当前选项附加数据。注意：无选项时返回空值。"""
        if self._index < 0:
            return None
        return self._items[self._index][1]

    def show_menu(self) -> None:
        """显示下拉菜单。注意：菜单项选择会同步当前索引。"""
        self.setDown(True)
        # 每次弹出都重建菜单项，保证与最新选项列表/选中态一致。
        self._menu.clear()
        self._menu.setMinimumWidth(self.width())
        for index, (text, _) in enumerate(self._items):
            action = QAction(text, self._menu)
            action.setCheckable(True)
            # 当前项打勾；点击某项时把 row 绑定进 lambda 以更新选中索引。
            action.setChecked(index == self._index)
            action.triggered.connect(lambda checked=False, row=index: self.setCurrentIndex(row))
            self._menu.addAction(action)
        # 计算弹出锚点（按钮局部坐标），右侧弹出留出 34px 横向间隙。
        if self._popup_side == "right":
            point = QPoint(self.width() + 34, 0)
        else:
            point = QPoint(0, self.height() + 2)
        # 转换为全局坐标后弹出菜单。
        self._menu.popup(self.mapToGlobal(point))
