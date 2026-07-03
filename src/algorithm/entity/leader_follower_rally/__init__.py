"""领航跟随集结实体包。注意：长机与僚机实体保持独立导出，避免影响既有保持实体。"""

import math

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import (
    FormCommInitS,
    FormPosS,
    PosInEarthS,
    PosTrackDiagS,
    WayPointInputS,
    copy_motion,
    copy_pos_track_diag,
)
from src.algorithm.entity.types import EntityOutputS, VelCmdLimitS
from src.algorithm.units.algo.formation_math import horizontal_track_vector_to_enu


_MIN_FIRST_SEGMENT_HORIZ_M = 1e-6  # 第一航段水平长度下限；小于此值视为退化（水平重合，仅高度不同也算）
_DEFAULT_LOITER_SPEED_MIN_MPS = 14.0  # velCmdLimit 未给出有效下限时的兜底盘旋最小速度
_DEFAULT_LOITER_SPEED_MAX_MPS = 25.0  # velCmdLimit 未给出有效上限时的兜底盘旋最大速度


def loiter_speed_bounds(vel_cmd_limit: VelCmdLimitS) -> tuple[float, float]:
    """从 velCmdLimit.forwardMin/forwardMax 推导 RallyJoinPos 的盘旋速度上下限。

    注意：`RallyFollowerEntity.init`/`RallyLeaderEntity.init`/`_ConfigLoader.validate` 三处都要用
    同一套推导（未配置或非正值时退回固定翼速度下限 14/25 m/s），抽出来避免三份逻辑各自漂移。

    下限/上限各自独立回退：只显式配置其中一个时，另一个会退到默认值，可能与显式配置的值反了序
    （如只配 `forwardMax=10` → (14, 10)，只配 `forwardMin=30` → (30, 25)）。在这里统一校验顺序，
    让调用方（含 `_ConfigLoader.validate()`）都能在早期拿到同一个结论，不必各自补一遍序校验。
    """
    fwd_min = vel_cmd_limit.forwardMin
    fwd_max = vel_cmd_limit.forwardMax
    loiter_min = fwd_min if (math.isfinite(fwd_min) and fwd_min > 0) else _DEFAULT_LOITER_SPEED_MIN_MPS
    loiter_max = fwd_max if (math.isfinite(fwd_max) and fwd_max > 0) else _DEFAULT_LOITER_SPEED_MAX_MPS
    if loiter_max <= loiter_min:
        raise ValueError(
            f"loiter_speed_bounds: 推导出的盘旋速度上下限非法（min={loiter_min}, max={loiter_max}）："
            "velCmdLimit.forwardMin/forwardMax 只显式配置一侧、另一侧退回默认值（14/25 m/s）时，"
            "两者可能反序；请同时显式配置一对自洽的 forwardMin/forwardMax，或都不配置以使用默认值"
        )
    return loiter_min, loiter_max


def rally_route_heading_rad(route: list[WayPointInputS]) -> float:
    """按集结航线第一航段（A→A1）计算任务航向。注意：调用方需保证 route 至少含两个航点。"""
    a = route[0].pos
    a1 = route[1].pos
    d_e = a1.east - a.east
    d_n = a1.north - a.north
    if math.hypot(d_e, d_n) < _MIN_FIRST_SEGMENT_HORIZ_M:
        # atan2(0, 0) 静默返回 0（正东），会悄悄算出错误的 M_i/盘旋圆而不报错——必须显式拒绝。
        raise ValueError(
            "rally_route 第一航段水平长度退化为零（A/A1 水平坐标重合，仅高度不同也算）："
            "无法据此推导任务航向，请检查 rally_route 前两个航点"
        )
    return math.atan2(d_n, d_e)


def rally_loose_target(route_start: PosInEarthS, heading_rad: float, scale: float, slot: FormPosS) -> PosInEarthS:
    """按集结区起点 A、任务航向、松散放大倍数与队形槽位，计算本机松散目标点 M_i。"""
    east_off, north_off = horizontal_track_vector_to_enu(
        (slot.x, slot.z), (math.cos(heading_rad), math.sin(heading_rad))
    )
    return PosInEarthS(
        east=route_start.east + scale * east_off,
        north=route_start.north + scale * north_off,
        h=route_start.h + slot.y,  # 高度固定差，不随 looseScale 扩展
    )


def resolve_formation_slot(comm_init: FormCommInitS, target_pattern: int, node_id: str) -> FormPosS | None:
    """按目标队形索引在 formPos 中定位本机槽位。注意：`target_pattern` 是纯整型队形索引（formPos 行号，
    与 `FormSnapshotS.pattern` 同一语义），不再是需要在 `formPat`（仅供显示的队形名列表）中查找的枚举值。
    索引越界或本机槽位缺失时返回 None，由调用方决定报错方式——RallyFollowerEntity.init 需要按具体原因抛
    不同的 ValueError 文案，GUI 侧的静态几何预计算（sim_control_routes._build_rally_join_geometry）只需要
    "找不到就跳过"。
    """
    if not (0 <= target_pattern < len(comm_init.formPos)):
        return None
    return next((slot for slot in comm_init.formPos[target_pattern] if slot.id == node_id), None)


def fill_output(cxt: FormContextS, diag: PosTrackDiagS, outbox: list, y: EntityOutputS) -> None:
    """将 Context 中的计算结果回填到实体输出边界。"""
    if y.selfAccCmd is None:
        y.selfAccCmd = cxt.selfAccCmd
    else:
        y.selfAccCmd.accEast = cxt.selfAccCmd.accEast
        y.selfAccCmd.accNorth = cxt.selfAccCmd.accNorth
        y.selfAccCmd.accUp = cxt.selfAccCmd.accUp
    if y.selfCmd is None:
        y.selfCmd = cxt.selfCmd
    else:
        copy_motion(cxt.selfCmd, y.selfCmd)
    if y.controlDiag is None:
        y.controlDiag = diag
    else:
        copy_pos_track_diag(diag, y.controlDiag)
    y.outbox.clear()
    y.outbox.extend(outbox)
