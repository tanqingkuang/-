"""领航跟随保持场景的僚机实体。注意：依赖长机广播消息更新目标。"""

from __future__ import annotations

import math

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import PosTrackDiagS, copy_motion, copy_pos_track_diag
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.leader_follower_hold.leader import _follower_tracker_init
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS, VelCmdLimitS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS, SlotGeometryInputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose
from src.algorithm.units.process.inbound.base import InboundInputS
from src.algorithm.units.process.inbound.rally_leader_follower import RallyLeaderFollower, RallyLeaderFollowerOutputS
from src.algorithm.units.process.tra_plan.base import TraPlanInputS, TraPlanOutputS
from src.algorithm.units.process.tra_plan.noop import Noop


class FollowerEntity(EntityBase):
    """僚机实体：入站解析长机广播 -> 空航路规划 -> 按槽位几何解算目标 -> 位置跟踪。注意：僚机不规划航线也不广播，目标位置由长机状态加队形槽位推出。"""

    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 FollowerEntity。注意：调用方需先准备好必要依赖和输入数据。"""
        # 共享黑板与复用的收件箱列表，避免每帧重新分配
        self.cxt = FormContextS()
        self._inbox = []

        # 僚机处理链：入站解析 -> 空规划(占位) -> 槽位几何 -> 位置跟踪
        self._inbound = RallyLeaderFollower()
        self._tra_plan = Noop()  # 僚机不自规划航线，用空实现占位以对齐链路结构
        self._pos_calc = SlotGeometry()
        self._pos_track = PidCompose()

        # 槽位几何需注入本机 id 及队形配置，据此从长机状态推算自身目标位
        self._inbound.init(None)
        self._tra_plan.init(None)
        # control_period_s 启用相对槽位 TD：软化队形重构的槽位阶跃，并由 TD 的 x2 补出相对切换速度前馈。
        # TD 参考速度上界取各通道速度权限，防大阶跃参考跑飞：前向按速度指令区间半宽、垂向按爬升限、侧向给保守缺省。
        v_fwd, v_up, v_lat = _slot_td_vmax(cfg.velCmdLimit)
        self._pos_calc.init(SlotGeometryInitS(
            cfg.selfInit.id, cfg.commInit.formPat, cfg.commInit.formPos,
            control_period_s=cfg.control_period_s,
            vMaxForward=v_fwd, vMaxVertical=v_up, vMaxLateral=v_lat,
        ))
        self._pos_track.init(_follower_tracker_init(cfg.control_period_s, cfg.velCmdLimit))

        # 预绑定端口到黑板：入站把长机状态/指令写入黑板，供后续单元消费
        self._inbound_u = InboundInputS(inbox=self._inbox)
        # slotScale 端口必须绑定（RallyLeaderFollower 强制校验非 None），hold 场景只使用默认 scale=1.0；
        # 若接收到旧格式广播缺少 slot_scale/t_ref，则仍回退到 scale=1.0/t_ref_valid=False。
        self._inbound_y = RallyLeaderFollowerOutputS(
            leaderState=self.cxt.leaderState, cmd=self.cxt.cmd, slotScale=self.cxt.slotScale
        )
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine)
        # 槽位几何输入：长机状态 + 编队指令即可定出僚机目标位，前向待飞距闭环已下沉到 PidCompose，无需本机状态。
        # slotScale 端口绑定到 Context：hold 场景默认 scale=1.0/scaleRate=0.0，行为等价于未缩放槽位；
        # 集结场景复用同一 SlotGeometry 时可由 Rally 动态写入缩放因子。
        self._pos_calc_u = SlotGeometryInputS(
            leaderState=self.cxt.leaderState,
            cmd=self.cxt.cmd,
            slotScale=self.cxt.slotScale,
            selfState=self.cxt.selfState,  # 仅供 TD (重)挂载首拍按当前位置播种，稳态几何目标不依赖本机状态
        )
        self._pos_calc_y = PosCalcOutputS(selfCmd=self.cxt.selfCmd)
        self._pos_track_u = PosTrackInputS(selfCmd=self.cxt.selfCmd, selfState=self.cxt.selfState)
        self._pos_track_diag = PosTrackDiagS()
        self._pos_track_y = PosTrackOutputS(accCmd=self.cxt.selfAccCmd, diag=self._pos_track_diag)

    def step(self, u: EntityInputS, y: EntityOutputS) -> None:
        """推进 FollowerEntity 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        # 灌入本机状态反馈
        if u.selfState is not None:
            copy_motion(u.selfState, self.cxt.selfState)
        # 刷新收件箱为本帧消息，供入站单元解析长机广播
        self._inbox.clear()
        self._inbox.extend(u.inbox)
        # 按顺序推进处理链
        self._inbound.step(self._inbound_u, self._inbound_y)
        self._tra_plan.step(self._tra_plan_u, self._tra_plan_y)
        self._pos_calc.step(self._pos_calc_u, self._pos_calc_y)
        self._pos_track.step(self._pos_track_u, self._pos_track_y)
        # 回填加速度指令，逻辑同长机
        if y.selfAccCmd is None:
            y.selfAccCmd = self.cxt.selfAccCmd
        else:
            y.selfAccCmd.accEast = self.cxt.selfAccCmd.accEast
            y.selfAccCmd.accNorth = self.cxt.selfAccCmd.accNorth
            y.selfAccCmd.accUp = self.cxt.selfAccCmd.accUp
        if y.selfCmd is None:
            y.selfCmd = self.cxt.selfCmd
        else:
            copy_motion(self.cxt.selfCmd, y.selfCmd)
        if y.controlDiag is None:
            y.controlDiag = self._pos_track_diag
        else:
            copy_pos_track_diag(self._pos_track_diag, y.controlDiag)
        y.outbox.clear()  # 僚机不发消息，输出固定清空

    def reset(self) -> None:
        """复位 FollowerEntity 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        # 原地清空黑板与各单元运行期状态，保留构造期依赖
        reset_context(self.cxt)
        self._inbound.reset()
        self._tra_plan.reset()
        self._pos_calc.reset()
        self._pos_track.reset()
        copy_pos_track_diag(PosTrackDiagS(), self._pos_track_diag)
        self._inbox.clear()

    def close(self) -> None:
        """释放 FollowerEntity 持有的资源。注意：关闭后不应继续调用运行接口。"""
        return None


# 侧向无 config 速度限幅，按 g·tan(40°) 量级给保守缺省(下游还有航迹角变限幅兜底)。
_SLOT_TD_VMAX_LATERAL_DEFAULT = 6.0
# 速度限幅为 ±inf(未配置)时的兜底：前向相对速度半宽、垂向爬升限。
_SLOT_TD_VMAX_FORWARD_FALLBACK = 5.0
_SLOT_TD_VMAX_VERTICAL_FALLBACK = 3.0


def _slot_td_vmax(vel_limit: VelCmdLimitS | None) -> tuple[float, float, float]:
    """由速度指令限幅推出相对槽位 TD 的三轴参考速度上界 (前向, 垂向, 侧向)，单位 m/s。

    前向取速度指令区间半宽(相对巡航速度的可用增减速)，垂向取爬升/下降限的较小者，侧向给保守缺省。
    限幅缺省为 ±inf 时回退到兜底常量。
    """
    vel_limit = vel_limit or VelCmdLimitS()
    if math.isfinite(vel_limit.forwardMin) and math.isfinite(vel_limit.forwardMax):
        v_fwd = 0.5 * (vel_limit.forwardMax - vel_limit.forwardMin)
    else:
        v_fwd = _SLOT_TD_VMAX_FORWARD_FALLBACK
    v_up = min(abs(vel_limit.verticalMin), abs(vel_limit.verticalMax))
    if not math.isfinite(v_up):
        v_up = _SLOT_TD_VMAX_VERTICAL_FALLBACK
    # 参考速度取 0.8×通道速度权限：留 20% 余量给反馈纠偏，避免参考速度顶到硬限后回路没余量→过冲。
    return 0.8 * v_fwd, 0.8 * v_up, 0.8 * _SLOT_TD_VMAX_LATERAL_DEFAULT
