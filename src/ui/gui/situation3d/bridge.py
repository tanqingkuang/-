"""3D 态势 Python-QML 数据桥。注意：桥只传展示数据，不调用仿真控制。"""

from __future__ import annotations

import json

from PySide6.QtCore import QObject, Signal, Slot


class Situation3DBridge(QObject):
    """把 Python 场景数据推送给 QML。注意：payload 使用 JSON 字符串降低绑定复杂度。"""

    sceneDataChanged = Signal(str)

    def __init__(self) -> None:
        """初始化 Situation3DBridge 实例，准备持有最近一次场景数据。"""
        super().__init__()
        self._scene_data = "{}"

    def set_scene_payload(self, payload: dict[str, object]) -> None:
        """更新场景数据并通知 QML。注意：QML 侧负责解析 JSON 和刷新模型。"""

        self._scene_data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.sceneDataChanged.emit(self._scene_data)

    @Slot(result=str)
    def sceneData(self) -> str:
        """返回最近一次场景 JSON。注意：供 QML 首次加载后主动拉取。"""

        return self._scene_data
