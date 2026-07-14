"""GUI 扰动按钮、命令和显示文案的单一规格表。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from src.runner.sim_control import DisturbanceCommand, DisturbanceType


@dataclass(frozen=True)
class DisturbanceAction:
    """一个 GUI 扰动动作的完整规格。注意：params 使用不可变键值对保存。"""

    kind: DisturbanceType
    button_text: str
    log_text: str
    display_text: str
    target: str | None = None
    duration_s: float | None = None
    params: tuple[tuple[str, object], ...] = ()

    @property
    def command(self) -> DisturbanceCommand:
        """每次生成独立命令，避免共享规格中的参数被控制器或调用方改写。"""

        return DisturbanceCommand(
            type=self.kind,
            target=self.target,
            duration_s=self.duration_s,
            params=dict(self.params),
        )


# 元组顺序就是界面 2x2 按钮顺序，布局层不得另建一份文案列表重新排序。
DISTURBANCE_ACTIONS: tuple[DisturbanceAction, ...] = (
    # 风场只改变模型外部速度，不需要目标节点或链路。
    DisturbanceAction(
        DisturbanceType.WIND,
        "风场脉冲",
        "注入风场脉冲",
        "风场",
        duration_s=8.0,
        params=(("speed_mps", 8.0), ("direction_deg", 90.0)),
    ),
    # 节点故障演示固定指向 A02，便于状态表与回报区形成可重复观察结果。
    DisturbanceAction(
        DisturbanceType.NODE_FAULT,
        "节点故障",
        "注入 A02 控制效率下降",
        "节点故障",
        target="A02",
        duration_s=10.0,
        params=(("mode", "degraded"),),
    ),
    # 链路丢包演示固定使用配置中的 A01-A02，并保留原链路状态语义。
    DisturbanceAction(
        DisturbanceType.LINK_LOSS,
        "链路丢包",
        "注入链路丢包",
        "链路丢包",
        target="A01-A02",
        duration_s=12.0,
        params=(("loss_rate", 0.3),),
    ),
    # clear 作为显式动作保留在同一规格表，但不会出现在活跃扰动快照中。
    DisturbanceAction(
        DisturbanceType.CLEAR,
        "清除扰动",
        "清除运行期扰动",
        "无",
    ),
)

# kind 是跨层稳定键；按钮文案变更不会影响控制器命令分发。
_ACTION_BY_KIND = {action.kind: action for action in DISTURBANCE_ACTIONS}
# 显示文案复用动作规格，避免注入成功与后续快照刷新出现两套中文名称。
_DISPLAY_BY_KIND = {action.kind: action.display_text for action in DISTURBANCE_ACTIONS}
# link_fault 由脚本/控制器使用，GUI 没有独立按钮，但视觉上仍归入链路异常。
_DISPLAY_BY_KIND[DisturbanceType.LINK_FAULT] = _DISPLAY_BY_KIND[DisturbanceType.LINK_LOSS]


def disturbance_action(kind: DisturbanceType | str) -> DisturbanceAction:
    """返回指定扰动动作；非法类型直接抛出 ValueError，避免运行时裸字典 KeyError。"""

    normalized = DisturbanceType(kind)
    try:
        return _ACTION_BY_KIND[normalized]
    except KeyError as exc:
        raise ValueError(f"没有对应的 GUI 扰动动作：{normalized}") from exc


def active_disturbance_text(active_types: Iterable[DisturbanceType | str]) -> str:
    """把权威活跃类型映射为单个状态标签，按节点、链路、风场确定显示优先级。"""

    # 先转集合消除同类型重复注入；标签表达类别，不承担实例计数职责。
    active = {DisturbanceType(kind) for kind in active_types}
    # 节点故障直接影响编队算法，是多个扰动并存时最需要用户关注的状态。
    if DisturbanceType.NODE_FAULT in active:
        return _DISPLAY_BY_KIND[DisturbanceType.NODE_FAULT]
    # 链路异常次之；link_loss 与 link_fault 共享一个用户可理解的短标签。
    if active & {DisturbanceType.LINK_LOSS, DisturbanceType.LINK_FAULT}:
        return _DISPLAY_BY_KIND[DisturbanceType.LINK_LOSS]
    if DisturbanceType.WIND in active:
        return _DISPLAY_BY_KIND[DisturbanceType.WIND]
    return _DISPLAY_BY_KIND[DisturbanceType.CLEAR]
