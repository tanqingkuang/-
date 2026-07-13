"""3D 态势 Python-QML 数据桥。注意：桥只传展示数据，不调用仿真控制。"""

from __future__ import annotations

import json

from PySide6.QtCore import QObject, Signal, Slot


class Situation3DBridge(QObject):
    """把 Python 场景数据推送给 QML。注意：payload 使用 JSON 字符串降低绑定复杂度。"""

    sceneDataChanged = Signal(str)
    modelSelected = Signal(str)

    def __init__(self) -> None:
        """初始化 Situation3DBridge 实例，准备持有最近一次场景数据。"""
        super().__init__()
        self._scene_data = "{}"
        # 记录上一次真正下发过完整填充网格的 staticKey；QML 只在这个签名变化的那一帧
        # 才会重建风险区填充模型（rebuildStaticModels 由 staticChanged 门控），
        # 后续签名不变的帧即使照常 10Hz 推送，也不需要再带上几十到几百 KB 的 meshValue。
        self._last_fill_static_key: object = None

    def set_scene_payload(self, payload: dict[str, object]) -> None:
        """更新场景数据并通知 QML。注意：QML 侧负责解析 JSON 和刷新模型。"""

        payload = self._strip_unchanged_fill_meshes(payload)
        self._scene_data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.sceneDataChanged.emit(self._scene_data)

    def _strip_unchanged_fill_meshes(self, payload: dict[str, object]) -> dict[str, object]:
        """静态签名未变时去掉填充网格的 meshValue，避免重复占用逐帧推送带宽。"""

        static_key = payload.get("staticKey")
        fills = payload.get("riskZoneFills")
        if not isinstance(fills, list) or not fills:
            # 空列表不代表"已完整下发过"，保留原记录，避免障碍从有到无再到有时误判为已发送。
            return payload
        if static_key == self._last_fill_static_key:
            # 浅拷贝 payload 本身，避免修改调用方仍持有的原始 dict；
            # riskZoneFills 内的每个条目也各自拷贝一份，只去掉体积大的 meshValue。
            trimmed = dict(payload)
            trimmed["riskZoneFills"] = [
                {key: value for key, value in item.items() if key != "meshValue"} for item in fills
            ]
            return trimmed
        # 签名变化（含首次调用）：记录新签名，原样返回携带完整 meshValue 的 payload。
        self._last_fill_static_key = static_key
        return payload

    @Slot(str)
    def selectModel(self, value: str) -> None:
        """通知 Python 侧切换显示机型。注意：只传显示设置，不触发仿真控制。"""

        self.modelSelected.emit(value)

    @Slot(result=str)
    def sceneData(self) -> str:
        """返回最近一次场景 JSON。注意：供 QML 首次加载后主动拉取。"""

        return self._scene_data
