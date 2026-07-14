"""3D 态势 Python-QML 数据桥。注意：桥只传展示数据，不调用仿真控制。"""

from __future__ import annotations

import json

from PySide6.QtCore import QObject, Signal, Slot

# 哨兵对象：区分"从未收到过 staticKey"和"上一帧 staticKey 恰好是 None/空字符串"，
# 避免用 None 当初始值时和真实 staticKey 撞车导致误判为"未变化"。
_UNSET_STATIC_KEY = object()


class Situation3DBridge(QObject):
    """把 Python 场景数据推送给 QML。注意：payload 使用 JSON 字符串降低绑定复杂度。"""

    sceneDataChanged = Signal(str)
    modelSelected = Signal(str)

    def __init__(self) -> None:
        """初始化 Situation3DBridge 实例，准备持有最近一次场景数据。"""
        super().__init__()
        self._scene_data = "{}"
        # 记录“上一帧”的 staticKey（不管那一帧填充是否为空），必须和 QML 侧
        # staticContentKey 的更新时机完全同步：QML 每一帧都会把 staticChanged 判定为
        # payload.staticKey != staticContentKey，其中 staticContentKey 在任意签名变化
        # （含变成空列表的那一帧）后都会被刷新。这里如果只在“非空填充”时更新，会在
        # A → 空填充 B → A 这种序列里误把第三帧当成“签名未变”，裁掉本该重新下发的
        # meshValue，导致 QML 恢复填充模型时拿不到网格数据。
        self._last_static_key: object = _UNSET_STATIC_KEY

    def set_scene_payload(self, payload: dict[str, object]) -> None:
        """更新场景数据并通知 QML。注意：QML 侧负责解析 JSON 和刷新模型。"""

        payload = self._strip_unchanged_fill_meshes(payload)
        self._scene_data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.sceneDataChanged.emit(self._scene_data)

    def _strip_unchanged_fill_meshes(self, payload: dict[str, object]) -> dict[str, object]:
        """静态签名未变时去掉填充网格的 meshValue，避免重复占用逐帧推送带宽。"""

        static_key = payload.get("staticKey")
        fills = payload.get("riskZoneFills")
        # 无论本帧是否携带填充，都先记录“上一帧”的签名，再用旧值做比较——
        # 这样空填充帧也会正确地让下一次同签名恢复帧判定为“签名已变化”。
        previous_key = self._last_static_key
        self._last_static_key = static_key
        if not isinstance(fills, list) or not fills:
            return payload
        if static_key == previous_key:
            # 浅拷贝 payload 本身，避免修改调用方仍持有的原始 dict；
            # riskZoneFills 内的每个条目也各自拷贝一份，只去掉体积大的 meshValue。
            trimmed = dict(payload)
            trimmed["riskZoneFills"] = [
                {key: value for key, value in item.items() if key != "meshValue"} for item in fills
            ]
            return trimmed
        # 签名相对上一帧发生了变化（含首次调用）：原样返回携带完整 meshValue 的 payload。
        return payload

    @Slot(str)
    def selectModel(self, value: str) -> None:
        """通知 Python 侧切换显示机型。注意：只传显示设置，不触发仿真控制。"""

        self.modelSelected.emit(value)

    @Slot(result=str)
    def sceneData(self) -> str:
        """返回最近一次场景 JSON。注意：供 QML 首次加载后主动拉取。"""

        return self._scene_data
