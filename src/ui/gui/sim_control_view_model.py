"""仿真控制 ViewModel。注意：本模块只含纯 Python 状态与规则，不依赖 Qt。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from src.ui.gui.view_models import NodeState, Snapshot


@dataclass(frozen=True)
class SimControlDisplay:
    """仿真控制区需要执行的一次显示更新。注意：duration_text 为 None 表示不回填文本。"""

    report_text: str
    timeline_text: str
    cpu_text: str
    progress_permille: int
    play_enabled: bool
    play_text: str
    step_enabled: bool
    reset_enabled: bool
    disturbance_enabled: bool
    rally_enabled: bool
    duration_text: str | None
    duration_enabled: bool


class SimControlViewModel:
    """封装仿真控制区显示规则。注意：不复刻控制器命令合法性状态机。"""

    def on_snapshot(self, snapshot: Snapshot) -> SimControlDisplay:
        """根据快照生成控件显示状态。注意：只描述界面显示和使能，不下发命令。"""

        # 控件显示字段集中生成，避免 MainWindow 分散拼接同一批文案。
        return SimControlDisplay(
            report_text=f"回报：{snapshot.control_report}",
            timeline_text=f"{snapshot.time:.1f} / {snapshot.duration:.0f}s",
            cpu_text=f"CPU {snapshot.cpu_utilization * 100:.0f}%",
            progress_permille=progress_permille(snapshot.time, snapshot.duration),
            # 这里只决定按钮是否可点，真正的 start/pause 合法性仍由控制器裁决。
            play_enabled=snapshot.run_state != "UNLOADED" and snapshot.run_state != "FINISHED",
            play_text=play_button_text(snapshot.run_state),
            step_enabled=snapshot.run_state in {"READY", "PAUSED"},
            reset_enabled=snapshot.run_state != "UNLOADED",
            # 扰动按钮与播放按钮沿用同一加载/结束态显示守卫。
            disturbance_enabled=snapshot.run_state != "UNLOADED" and snapshot.run_state != "FINISHED",
            rally_enabled=rally_button_enabled(snapshot.run_state, snapshot.nodes),
            duration_text=duration_text_for_snapshot(snapshot),
            duration_enabled=duration_input_enabled(snapshot.run_state),
        )


def progress_permille(time_s: float, duration_s: float) -> int:
    """把仿真进度换算为千分刻度。注意：duration 为 0 时返回 0 防除零。"""

    # 进度条范围固定为 0..1000，因此这里保留原千分刻度换算。
    return round(time_s / duration_s * 1000) if duration_s else 0


def play_button_text(run_state: str) -> str:
    """生成播放按钮文案。注意：文案表示点击后会发生的动作。"""

    # RUNNING/PAUSED 是唯二改变按钮文案的状态，其余状态统一显示“开始”。
    return {"RUNNING": "暂停", "PAUSED": "继续"}.get(run_state, "开始")


def duration_input_enabled(run_state: str) -> bool:
    """判断时长输入框是否可编辑。注意：只有待命和暂停态允许编辑。"""

    # 运行中修改时长由控制器拒绝；输入框只在可编辑态开放。
    return run_state in {"READY", "PAUSED"}


def duration_text_for_snapshot(snapshot: Snapshot) -> str | None:
    """生成时长输入框回填文本。注意：未加载配置时不回填，避免覆盖占位状态。"""

    # 未加载态不回填文本，避免把默认时长误显示为已加载配置。
    if snapshot.run_state == "UNLOADED":
        return None
    return format_duration_text(snapshot.duration)


def format_duration_text(duration_s: float) -> str:
    """格式化仿真时长文本。注意：整数秒不显示小数。"""

    # 有限整数秒不带小数，非整数与 inf/nan 沿用 Python 格式化输出。
    if math.isfinite(duration_s) and duration_s.is_integer():
        return str(int(duration_s))
    return f"{duration_s:.3f}".rstrip("0").rstrip(".")


def parse_duration_text(text: str) -> float | None:
    """解析时长输入文本。注意：仅解析失败返回 None，命令合法性由控制器裁决。"""

    # 解析只判断文本能否转为 float，范围和当前时间约束交给控制器。
    try:
        return float(text)
    except ValueError:
        return None


def rally_button_enabled(run_state: str, nodes: Iterable[NodeState]) -> bool:
    """判断集结按钮是否可用。注意：集结中保持可点，用于返回明确提示。"""

    # READY 阶段只显示几何预览，算法还没开始本地盘旋，因此不能发开始集结。
    if run_state in {"UNLOADED", "READY", "FINISHED"}:
        return False
    # 普通保持/避障等非集结配置不显示可执行入口，避免按钮语义混淆。
    rally_nodes = [
        node
        for node in nodes
        if node.role.strip().lower() in {"rally_leader", "rally_follower"}
    ]
    if not rally_nodes:
        return False
    # 只从集结角色收集 phase，普通长机/僚机不参与按钮开放判定。
    phases = {node.rally_phase for node in rally_nodes}
    # 空 phase 只会出现在 READY 预览或异常冷启动场景，不能据此开放按钮。
    # 多机 phase 不完全一致时取并集判断，只要仍有集结活动阶段就允许重复点击提示。
    # HOLD 是集结的单向终态，本轮方案不支持回退到本地待命或重新集结。
    if "HOLD" in phases:
        return False
    # LOCAL_LOITER 触发真正开始；ACTIVE 阶段保留可点状态，让控制器返回“已在集结中”。
    return bool(phases & {"LOCAL_LOITER", "RALLY_TRANSIT", "RALLY_LOITER", "RALLY_EXITED", "CATCHUP", "LOOSE", "COMPRESS"})
