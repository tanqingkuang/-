"""实时监控与离线回放共用的图表通道和坐标轴规则。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

from PySide6.QtCharts import QValueAxis

from src.runner.sim_control import NodeState

# 节点颜色按发现顺序循环分配，窗口重建期间必须保持这一顺序稳定。
CHART_PALETTE: tuple[str, ...] = (
    "#1f77b4",
    "#2ca02c",
    "#ff7f0e",
    "#9467bd",
    "#8c564b",
)


@dataclass(frozen=True)
class ChannelSpec:
    """单个控制误差通道规格。注意：实时与离线窗口必须消费同一实例序列。"""

    # key 同时作为复选框、缓冲区和曲线索引，发布后应保持稳定。
    key: str
    # label/unit 只负责用户文案，不参与数值换算。
    label: str
    unit: str
    # group 决定侧栏分段，on 决定两个窗口共同的默认可见性。
    group: str
    on: bool
    # act 只读取快照节点，不保存窗口状态，因此同一规格可安全供实时和离线复用。
    act: Callable[[NodeState], float | None]


def heading_deviation(node: NodeState) -> float | None:
    """返回实际航迹角减指令航迹角，并归一化到负 180 至正 180 度。"""

    # 指令水平速度太小时航向没有稳定定义，避免 atan2(0, 0) 产生虚假偏差。
    speed = math.hypot(node.cmd_vel_east_mps, node.cmd_vel_north_mps)
    if speed < 1e-3:
        return None
    # 项目航向约定是 ENU：0 度指东、90 度指北。
    command_heading = math.degrees(
        math.atan2(node.cmd_vel_north_mps, node.cmd_vel_east_mps)
    )
    return (node.psi_v_deg - command_heading + 180.0) % 360.0 - 180.0


# 顺序同时决定侧栏显示顺序；扩展通道只修改此处，两个窗口会同步生效。
CONTROL_ERROR_CHANNELS: tuple[ChannelSpec, ...] = (
    # 前向轴 x
    ChannelSpec(
        "perr_x",
        "前向位置误差",
        "m",
        "前向轴 x",
        True,
        lambda node: node.track_pos_err_x_m,
    ),
    ChannelSpec(
        "verr_x",
        "前向速度误差",
        "m/s",
        "前向轴 x",
        False,
        lambda node: node.track_vel_err_x_mps,
    ),
    # 每个轴的位置误差默认显示，速度误差由用户按需开启，避免初始画面过密。
    # 垂向轴 y
    ChannelSpec(
        "perr_y",
        "垂向位置误差",
        "m",
        "垂向轴 y",
        True,
        lambda node: node.track_pos_err_y_m,
    ),
    ChannelSpec(
        "verr_y",
        "垂向速度误差",
        "m/s",
        "垂向轴 y",
        False,
        lambda node: node.track_vel_err_y_mps,
    ),
    # 侧向轴 z
    ChannelSpec(
        "perr_z",
        "侧向位置误差",
        "m",
        "侧向轴 z",
        True,
        lambda node: node.track_pos_err_z_m,
    ),
    ChannelSpec(
        "verr_z",
        "侧向速度误差",
        "m/s",
        "侧向轴 z",
        False,
        lambda node: node.track_vel_err_z_mps,
    ),
    ChannelSpec("hdg_dev", "航迹角偏差", "°", "侧向轴 z", False, heading_deviation),
)


def apply_y_range(y_axis: QValueAxis, values: list[float]) -> None:
    """用 5% 至 95% 百分位设置 Y 轴，并始终包含零以抑制离群尖峰。"""

    if not values:
        # 空通道仍保留对称单位范围，避免 Qt 轴退化成零宽区间。
        y_axis.setRange(-1.0, 1.0)
        return
    # 百分位截断防止单个尖峰把主要误差趋势压成一条线。
    # 上下界分别夹住零点，保证灰色零基准线始终落在可见范围内。
    low = min(_percentile(values, 5), 0.0)
    high = max(_percentile(values, 95), 0.0)
    # 最小 0.5 的边距让全零或近零误差也有可读的纵向空间。
    margin = max((high - low) * 0.15, 0.5)
    y_axis.setRange(low - margin, high + margin)


def _percentile(data: list[float], percentile: float) -> float:
    """以线性插值返回百分位数。注意：调用方保证数据非空。"""

    # 手工线性插值避免 GUI 为一个小计算引入 NumPy 运行时依赖。
    # 数据点较少时仍使用同一插值公式，保持坐标轴变化平滑。
    sorted_data = sorted(data)
    index = (len(sorted_data) - 1) * percentile / 100.0
    lower = int(index)
    upper = lower + 1
    if upper >= len(sorted_data):
        return sorted_data[lower]
    return sorted_data[lower] + (index - lower) * (sorted_data[upper] - sorted_data[lower])
