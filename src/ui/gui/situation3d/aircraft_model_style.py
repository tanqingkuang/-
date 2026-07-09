"""3D 态势机型样式策略。注意：这里只描述渲染资产参数，不触碰仿真坐标语义。"""

from __future__ import annotations

from abc import ABC
from enum import Enum


class AircraftModelType(str, Enum):
    """3D 态势机型枚举。注意：枚举值是 QML 下拉和 payload 共用的稳定标识。"""

    # payload 里使用短值，避免 QML 下拉和 Python 枚举受显示名改动影响。
    BAYRAKTAR_TB2 = "tb2"
    PREDATOR = "predator"
    RQ4_GLOBAL_HAWK = "rq4"


class AircraftModelStyle(ABC):
    """3D 态势机型渲染策略基类。注意：子类只声明模型资产和尺寸校正参数。"""

    # 子类用类属性声明静态资产参数，新增机型时不需要改调用方。
    model_type: AircraftModelType
    # label 只面向下拉显示，不能作为反查键。
    label: str
    # 路径保持相对 qml 目录，QML 侧统一用 Qt.resolvedUrl 解析。
    model_source: str
    # 只校正资产自身机头朝向，不叠加仿真航迹角。
    yaw_offset_deg: float
    # unit_wingspan 是 glb 坐标单位下的实测翼展。
    unit_wingspan: float
    # real_wingspan_m 是同一机型的真实翼展，负责 1:1 显示换算。
    real_wingspan_m: float

    @property
    def base_scale(self) -> float:
        """返回模型按真实尺寸显示所需缩放。注意：模型单位翼展必须由资产实测得到。"""

        # baseScale 保持纯尺寸换算，远景可辨识放大由 QML 另行计算。
        return self.real_wingspan_m / self.unit_wingspan

    def style_payload(self) -> dict[str, object]:
        """生成 QML 机型样式 payload。注意：位置和航迹角仍由 scene_data 单机数据提供。"""

        # 字段名使用 QML 习惯的 camelCase，避免前端再做键名转换。
        return {
            "value": self.model_type.value,
            "label": self.label,
            "modelSource": self.model_source,
            "yawOffsetDeg": self.yaw_offset_deg,
            "baseScale": self.base_scale,
            "unitWingspan": self.unit_wingspan,
            "realWingspanM": self.real_wingspan_m,
        }


class BayraktarTB2Style(AircraftModelStyle):
    """Bayraktar TB2 机型渲染策略。注意：参数来自当前 glb 资产实测。"""

    model_type = AircraftModelType.BAYRAKTAR_TB2
    label = "TB2 察打无人机"
    model_source = "assets/BayraktarTB2.glb"
    # TB2 资产机头朝 +Z，本场景显示约定机头朝 +X，所以绕 Y 轴补 +90 度。
    yaw_offset_deg = 90.0
    # 模型按真实尺寸建模但存在 11.957/12.0 的轻微测量差，用 baseScale 抹平。
    unit_wingspan = 11.957
    real_wingspan_m = 12.0


class PredatorStyle(AircraftModelStyle):
    """捕食者无人机渲染策略。注意：参数沿用历史 QML 资产校正口径。"""

    model_type = AircraftModelType.PREDATOR
    label = "捕食者无人机"
    model_source = "assets/PredatorUAV.glb"
    # 捕食者资产机头朝 +Z，本场景显示约定机头朝 +X，所以绕 Y 轴补 +90 度。
    yaw_offset_deg = 90.0
    # 旧 QML 使用约 8.5 倍缩放，对应 1.76 模型单位翼展映射到约 15m 真实翼展。
    unit_wingspan = 1.76
    real_wingspan_m = 15.0


class RQ4GlobalHawkStyle(AircraftModelStyle):
    """RQ-4 全球鹰渲染策略。注意：参数来自当前 glb 资产实测和真实翼展。"""

    model_type = AircraftModelType.RQ4_GLOBAL_HAWK
    label = "RQ-4 全球鹰"
    model_source = "assets/RQ4GlobalHawk.glb"
    # 全球鹰资产机头朝 +Z，本场景显示约定机头朝 +X，所以绕 Y 轴补 +90 度。
    yaw_offset_deg = 90.0
    # 模型单位翼展来自 glb 实测，真实翼展取公开机型参数 39.9m。
    unit_wingspan = 0.469
    real_wingspan_m = 39.9


DEFAULT_AIRCRAFT_MODEL_TYPE = AircraftModelType.BAYRAKTAR_TB2

# 简单注册表就是工厂的数据源，后续机型只追加枚举成员和策略类。
_STYLE_REGISTRY: dict[AircraftModelType, type[AircraftModelStyle]] = {
    BayraktarTB2Style.model_type: BayraktarTB2Style,
    PredatorStyle.model_type: PredatorStyle,
    RQ4GlobalHawkStyle.model_type: RQ4GlobalHawkStyle,
}


def create_aircraft_model_style(model_type: AircraftModelType) -> AircraftModelStyle:
    """按枚举创建机型渲染策略。注意：未注册枚举值表示代码注册遗漏。"""

    # 不在这里自动转换字符串，调用方必须先完成枚举解析和错误处理。
    style_class = _STYLE_REGISTRY.get(model_type)
    if style_class is None:
        raise ValueError(f"未注册的 3D 态势机型: {model_type}")
    return style_class()


def available_model_options() -> list[dict[str, str]]:
    """返回可选机型列表。注意：只暴露下拉需要的稳定值和显示名。"""

    # 按注册表顺序输出，保持下拉顺序和代码注册顺序一致。
    return [
        {"value": model_type.value, "label": style_class.label}
        for model_type, style_class in _STYLE_REGISTRY.items()
    ]
