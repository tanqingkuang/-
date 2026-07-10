"""状态表 ViewModel。注意：本模块只含纯 Python 表格行换算，不依赖 Qt。"""

from __future__ import annotations

import math

from src.ui.gui.view_models import (
    WORLD_HEIGHT,
    WORLD_WIDTH,
    LinkState,
    NodeState,
    leader_node_from,
    link_direction_label,
)


def node_table_rows(nodes: list[NodeState]) -> list[list[str]]:
    """生成节点表行。注意：高飘/右偏按飞行诊断习惯显示本机相对目标偏差。"""

    # 先生成纯字符串矩阵，GUI 层只负责 setItem 和对齐。
    rows: list[list[str]] = []
    for node in nodes:
        # 健康枚举翻译成中文；未知值原样显示。
        status = {"normal": "正常", "degraded": "降级", "fault": "故障", "lost": "失联"}.get(node.health, node.health)
        # 节点表固定展示五列；rally_phase 仅保留在快照中，不写入隐藏的越界列。
        rows.append(
            [
                # 第一列保留节点 ID，后续三列保持既有飞行诊断符号约定。
                node.node_id,
                f"{node.track_pos_err_x:.1f}",
                f"{-node.track_pos_err_y:.1f}",
                f"{-node.track_pos_err_z:.1f}",
                status,
            ]
        )
    return rows


def overall_table_row(nodes: list[NodeState]) -> list[str] | None:
    """生成整体跟踪表行。注意：无节点时返回 None，由 GUI 清空表格行数。"""

    # 整体跟踪表：用长机代表当前全局航线跟踪情况，缺少显式长机时才回退首节点。
    # 复用通用长机选择规则，保证表格与俯视/侧视的 leader 口径一致。
    leader = leader_node_from(nodes)
    if leader is None:
        return None
    # 控制器未提供 route metric 时，用世界坐标估算保持旧演示数据可显示。
    side_offset = leader.cross_track_error
    if side_offset is None:
        side_offset = (leader.y - WORLD_HEIGHT / 2) * 0.8
    distance_to_go = leader.distance_to_go
    if distance_to_go is None:
        distance_to_go = max(0.0, (WORLD_WIDTH - leader.x) * 4)
    # 地速按水平面速度模长显示，不把垂向爬升率计入整体跟踪表。
    ground_speed = math.hypot(leader.vx, leader.vy)
    # 返回五列字符串，列顺序与 main_window_layout.py 的表头保持一致。
    # 链路方向文案来自 view_models，避免本 VM 依赖 adapter 形成反向导入。
    return [
        f"{side_offset:.0f}",
        f"{distance_to_go:.0f}",
        f"{leader.altitude:.0f}",
        f"{ground_speed:.0f}",
        f"{leader.vertical_speed:.0f}",
    ]


def link_table_rows(links: list[LinkState]) -> list[list[str]]:
    """生成链路表行。注意：丢包率换算成百分比，ok 标志映射为正常/丢包文案。"""

    # 返回五列字符串，列顺序与 main_window_layout.py 的表头保持一致。
    # 链路方向文案来自 view_models，避免本 VM 依赖 adapter 形成反向导入。
    return [
        [
            f"{link.source}-{link.target}",
            link_direction_label(link.direction),
            f"{link.latency_ms}ms",
            f"{link.loss * 100:.0f}%",
            "正常" if link.ok else "丢包",
        ]
        for link in links
    ]
