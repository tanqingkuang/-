"""领航跟随保持场景的僚机实体。注意：依赖长机广播消息更新目标。"""

from __future__ import annotations

from src.algorithm.context.context import FormContextS, reset_context
from src.algorithm.context.leaf_types import copy_motion
from src.algorithm.entity.base import EntityBase
from src.algorithm.entity.leader_follower_hold.leader import _default_tracker_init
from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS
from src.algorithm.units.algo.pos_calc.base import PosCalcOutputS
from src.algorithm.units.algo.pos_calc.slot_geometry import SlotGeometry, SlotGeometryInitS, SlotGeometryInputS
from src.algorithm.units.algo.pos_track.base import PosTrackInputS, PosTrackOutputS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose
from src.algorithm.units.process.inbound.base import InboundInputS, InboundOutputS
from src.algorithm.units.process.inbound.leader_follower import LeaderFollower
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
        self._inbound = LeaderFollower()
        self._tra_plan = Noop()  # 僚机不自规划航线，用空实现占位以对齐链路结构
        self._pos_calc = SlotGeometry()
        self._pos_track = PidCompose()

        # 槽位几何需注入本机 id 及队形配置，据此从长机状态推算自身目标位
        self._inbound.init(None)
        self._tra_plan.init(None)
        self._pos_calc.init(SlotGeometryInitS(cfg.selfInit.id, cfg.commInit.formPat, cfg.commInit.formPos))
        self._pos_track.init(_default_tracker_init())

        # 预绑定端口到黑板：入站把长机状态/指令写入黑板，供后续单元消费
        self._inbound_u = InboundInputS(inbox=self._inbox)
        self._inbound_y = InboundOutputS(leaderState=self.cxt.leaderState, cmd=self.cxt.cmd)
        self._tra_plan_u = TraPlanInputS(cmd=self.cxt.cmd, wayLine=self.cxt.wayLine, selfState=self.cxt.selfState)
        self._tra_plan_y = TraPlanOutputS(wayLine=self.cxt.wayLine)
        # 槽位几何输入：本机状态 + 长机状态 + 编队指令，三者共同定出僚机目标位
        self._pos_calc_u = SlotGeometryInputS(
            selfState=self.cxt.selfState,
            leaderState=self.cxt.leaderState,
            cmd=self.cxt.cmd,
        )
        self._pos_calc_y = PosCalcOutputS(selfCmd=self.cxt.selfCmd)
        self._pos_track_u = PosTrackInputS(selfCmd=self.cxt.selfCmd, selfState=self.cxt.selfState)
        self._pos_track_y = PosTrackOutputS(accCmd=self.cxt.selfAccCmd)

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
        y.outbox.clear()  # 僚机不发消息，输出固定清空

    def reset(self) -> None:
        """复位 FollowerEntity 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        # 原地清空黑板与各单元运行期状态，保留构造期依赖
        reset_context(self.cxt)
        self._inbound.reset()
        self._tra_plan.reset()
        self._pos_calc.reset()
        self._pos_track.reset()
        self._inbox.clear()

    def close(self) -> None:
        """释放 FollowerEntity 持有的资源。注意：关闭后不应继续调用运行接口。"""
        return None
