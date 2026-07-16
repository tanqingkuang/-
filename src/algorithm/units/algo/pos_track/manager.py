"""位置跟踪策略管理器。注意：产品只在初始化时创建，运行期由命令选择缓存对象。"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Callable

from src.algorithm.context.leaf_types import (
    PosTrackCommandE,
    PosTrackStrategyE,
    copy_motion,
    zero_acceleration,
)
from src.algorithm.units.algo.ctrl.ppi import PPIInitS
from src.algorithm.units.algo.pos_track.base import (
    PosTrackBase,
    PosTrackInitS,
    PosTrackInputS,
    PosTrackOutputS,
)
from src.algorithm.units.algo.pos_track.lateral_track_angle import LateralTrackAngleInitS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose, PidComposeInitS

if TYPE_CHECKING:
    from src.algorithm.entity.types import EntityInitS, EntityManagerInitS, VelCmdLimitS


_LATERAL_ROLL_MAX_RAD = math.radians(40.0)  # 执行层滚转角限幅
_LATERAL_GAMMA_MAX_RAD = math.radians(25.0)  # 大侧偏转弯半径尺度
_LATERAL_FLOOR_RAD = math.radians(7.0)  # 中心线附近最小拦截角
_LATERAL_PSI_CMD_MAX_RAD = math.radians(80.0)  # 指令航迹角上限
_LATERAL_R_MARGIN = 1.1  # 转弯半径裕度

_COMMAND_TO_STRATEGY = {
    # PosCalc 发布控制语义，PosTrack 只做稳定的一一映射。
    # 映射表禁止按任务阶段分支，避免控制层反向感知业务状态机。
    PosTrackCommandE.NOOP: PosTrackStrategyE.NOOP,
    PosTrackCommandE.SPEED_TRACK: PosTrackStrategyE.PID_SPEED,
    PosTrackCommandE.POSITION_TRACK: PosTrackStrategyE.PID_POSITION,
}


class _NoopPosTrack(PosTrackBase):
    """空位置跟踪产品。注意：保持旧 NONE 分支只清零加速度的语义。"""

    def init(self, cfg: PosTrackInitS) -> None:
        """初始化空产品。注意：无控制参数和动态状态。"""
        del cfg

    def step(self, u: PosTrackInputS, y: PosTrackOutputS) -> None:
        """输出停控结果。注意：有效指令保持为 PosCalc 生成的 selfCmd。"""
        # NOOP 只关闭加速度控制，不清除位置解算生成的诊断目标。
        # effectiveCmd 仍需同步，保证长机广播与本拍 selfCmd 一致。
        if y.accCmd is None:
            raise ValueError("NoopPosTrack accCmd port must be bound")
        zero_acceleration(y.accCmd)
        if y.effectiveCmd is not None and u.selfCmd is not None:
            copy_motion(u.selfCmd, y.effectiveCmd)

    def reset(self) -> None:
        """复位空产品。注意：无运行期状态。"""
        return None


def _tracker_init(control_period_s: float, gain_forward: PPIInitS, vel_limit: VelCmdLimitS) -> PidComposeInitS:
    """生成三轴组合控制配置。注意：参数保持与旧 Rally 装配完全一致。"""
    # 横向和垂向结构由两种前向产品共享，避免增益在建造函数间漂移。
    # control_period_s 进入积分和离散控制，必须在产品创建前校验。
    if control_period_s <= 0.0:
        raise ValueError("control_period_s must be positive")
    gain_lateral = LateralTrackAngleInitS(
        kpPos=0.2,
        kpVel=1.0,
        kiVel=0.2,
        dt=control_period_s,
        rollMaxRad=_LATERAL_ROLL_MAX_RAD,
        gammaMaxRad=_LATERAL_GAMMA_MAX_RAD,
        floorRad=_LATERAL_FLOOR_RAD,
        psiCmdMaxRad=_LATERAL_PSI_CMD_MAX_RAD,
        margin=_LATERAL_R_MARGIN,
    )
    gain_vertical = PPIInitS(
        kpPos=0.393,
        kpVel=0.689,
        kiVel=0.0,
        dt=control_period_s,
        accMin=-6.0,
        accMax=6.0,
        vCmdMin=vel_limit.verticalMin,
        vCmdMax=vel_limit.verticalMax,
    )
    return PidComposeInitS(0.5, gain_forward, gain_lateral, gain_vertical)


def _pid_speed_init(control_period_s: float, vel_limit: VelCmdLimitS) -> PidComposeInitS:
    """生成前向速度闭环产品配置。注意：前向位置增益固定为零。"""
    # 集结转场只按公共时间调速，前向位置误差不得参与控制。
    gain_forward = PPIInitS(
        kpPos=0.0,
        kpVel=1.0,
        kiVel=0.2,
        dt=control_period_s,
        accMin=-6.0,
        accMax=6.0,
        vCmdMin=vel_limit.forwardMin,
        vCmdMax=vel_limit.forwardMax,
    )
    return _tracker_init(control_period_s, gain_forward, vel_limit)


def _pid_position_init(control_period_s: float, vel_limit: VelCmdLimitS) -> PidComposeInitS:
    """生成前向位置和速度串级闭环产品配置。注意：保持既有僚机增益。"""
    # 编队槽位保持需要位置外环，因此保留既有 kpPos。
    gain_forward = PPIInitS(
        kpPos=0.2,
        kpVel=1.0,
        kiVel=0.2,
        dt=control_period_s,
        accMin=-6.0,
        accMax=6.0,
        vCmdMin=vel_limit.forwardMin,
        vCmdMax=vel_limit.forwardMax,
    )
    return _tracker_init(control_period_s, gain_forward, vel_limit)


def _build_noop(cfg: EntityInitS) -> PosTrackBase:
    """创建空控制产品。注意：配置参数只用于统一建造函数签名。"""
    del cfg
    strategy = _NoopPosTrack()
    strategy.init(PosTrackInitS())
    return strategy


def _build_pid_speed(cfg: EntityInitS) -> PosTrackBase:
    """创建前向速度 PID 组合产品。"""
    strategy = PidCompose()
    strategy.init(_pid_speed_init(cfg.control_period_s, cfg.velCmdLimit))
    return strategy


def _build_pid_position(cfg: EntityInitS) -> PosTrackBase:
    """创建前向位置和速度 PID 组合产品。"""
    strategy = PidCompose()
    strategy.init(_pid_position_init(cfg.control_period_s, cfg.velCmdLimit))
    return strategy


_StrategyBuilder = Callable[["EntityInitS"], PosTrackBase]
_BUILDERS: dict[PosTrackStrategyE, _StrategyBuilder] = {
    # 建造表只在 init 使用；运行期 registry 保存的是已建对象。
    PosTrackStrategyE.NOOP: _build_noop,
    PosTrackStrategyE.PID_SPEED: _build_pid_speed,
    PosTrackStrategyE.PID_POSITION: _build_pid_position,
}


class PosTrackManager:
    """创建、缓存并执行位置跟踪产品。注意：不感知任务阶段和实体角色。"""

    def __init__(self) -> None:
        """初始化空管理器。注意：必须先调用 init。"""
        self._registry: dict[PosTrackStrategyE, PosTrackBase] = {}

    def init(self, cfg: EntityManagerInitS) -> None:
        """按实体身份证创建全部控制产品。注意：不得隐式补充策略。"""
        # 实体配置明确声明能力集合，Manager 不按角色猜测缺失产品。
        # NOOP 是异常/停控路径的系统能力，也必须在配置中显式出现。
        strategies = tuple(_require_strategy(item) for item in cfg.process.strategies)
        if not strategies:
            raise ValueError("processes.pos_track.strategies 不得为空")
        if len(strategies) != len(set(strategies)):
            raise ValueError("processes.pos_track.strategies 不得包含重复策略")
        if PosTrackStrategyE.NOOP not in strategies:
            raise ValueError("processes.pos_track.strategies 必须显式包含 NOOP")
        # 每个 PID 产品只构造一次，积分器状态随产品对象跨帧保留。
        self._registry = {strategy: _BUILDERS[strategy](cfg.entity) for strategy in strategies}

    def step(self, u: PosTrackInputS, y: PosTrackOutputS) -> None:
        """按控制命令执行缓存产品。注意：命令与策略固定一一对应。"""
        # command 来自 PosCalc 的统一输出，不允许 Entity 二次改写。
        # 未配置产品属于装配错误，不能临时回退到另一控制器。
        if u.command is None:
            raise ValueError("PosTrackManager command port must be bound")
        if not isinstance(u.command.mode, PosTrackCommandE):
            raise ValueError("位置跟踪命令必须是 PosTrackCommandE")
        strategy_type = _COMMAND_TO_STRATEGY.get(u.command.mode)
        if strategy_type is None:
            raise ValueError(f"不支持的位置跟踪命令: {u.command.mode!r}")
        strategy = self._registry.get(strategy_type)
        if strategy is None:
            raise ValueError(f"位置跟踪策略未配置: {strategy_type.name}")
        # 运行期仅切换缓存引用，PID 积分状态不会因任务阶段变化丢失。
        strategy.step(u, y)

    def reset(self) -> None:
        """复位全部缓存产品。注意：保留配置和实例。"""
        # 重置每个产品的动态状态，但不重新读取配置或重建控制器。
        for strategy in self._registry.values():
            strategy.reset()


def _require_strategy(value: object) -> PosTrackStrategyE:
    """校验控制产品策略枚举。注意：禁止普通整数绕过配置语义。"""
    if not isinstance(value, PosTrackStrategyE):
        raise ValueError("processes.pos_track.strategies 必须是 PosTrackStrategyE")
    if value not in _BUILDERS:
        raise ValueError(f"不支持的位置跟踪策略: {value!r}")
    return value
