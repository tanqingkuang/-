"""位置跟踪策略管理器。注意：产品只在初始化时创建，运行期严格查询完整策略表。"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Callable

from src.algorithm.context.leaf_types import (
    PosTrackStrategyE,
    copy_motion,
    zero_acceleration,
)
from src.algorithm.units.algo.ctrl.ppi import PPIInitS
from src.algorithm.units.algo.pos_track.base import (
    PosTrackBase,
    PosTrackInitS,
)
from src.algorithm.units.algo.pos_track.lateral_track_angle import LateralTrackAngleInitS
from src.algorithm.units.algo.pos_track.pid_compose import PidCompose, PidComposeInitS

if TYPE_CHECKING:
    from src.algorithm.entity.types import (
        EntityInitS,
        EntityManagerInitS,
        EntityProfileS,
        EntityRuntimeS,
        VelCmdLimitS,
    )


_LATERAL_ROLL_MAX_RAD = math.radians(40.0)  # 执行层滚转角限幅
_LATERAL_GAMMA_MAX_RAD = math.radians(25.0)  # 大侧偏转弯半径尺度
_LATERAL_FLOOR_RAD = math.radians(7.0)  # 中心线附近最小拦截角
_LATERAL_PSI_CMD_MAX_RAD = math.radians(80.0)  # 指令航迹角上限
_LATERAL_R_MARGIN = 1.1  # 转弯半径裕度

class _NoopPosTrack(PosTrackBase):
    """空位置跟踪产品。注意：保持旧 NONE 分支只清零加速度的语义。"""

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定停控产品所需输入输出字段。"""
        cxt = runtime.context
        self._self_cmd = cxt.selfCmd
        self._effective_cmd = cxt.effectiveCmd
        self._acc_cmd = cxt.selfAccCmd

    def init(self, cfg: PosTrackInitS) -> None:
        """初始化空产品。注意：无控制参数和动态状态。"""
        del cfg

    def step(self) -> None:
        """输出停控结果。注意：有效指令保持为 PosCalc 生成的 selfCmd。"""
        # NOOP 只关闭加速度控制，不清除位置解算生成的诊断目标。
        # effectiveCmd 仍需同步，保证长机广播与本拍 selfCmd 一致。
        zero_acceleration(self._acc_cmd)
        copy_motion(self._self_cmd, self._effective_cmd)

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
    """创建、缓存并路由位置跟踪产品。注意：实体角色差异只来自 Profile。"""

    def __init__(self) -> None:
        """初始化空管理器。注意：必须先调用 init。"""
        self._registry: dict[PosTrackStrategyE, PosTrackBase] = {}
        self._profile: EntityProfileS | None = None
        self._cmd = None
        self._binding_runtime: EntityRuntimeS | None = None

    def bind(self, runtime: EntityRuntimeS) -> None:
        """绑定任务命令及产品运行环境。"""
        self._cmd = runtime.context.cmd
        self._binding_runtime = runtime

    def init(self, cfg: EntityManagerInitS) -> None:
        """按完整路由表创建全部位置跟踪产品。"""
        profile = cfg.profile
        strategies = {
            _require_strategy(item.pos_track, "route_table.pos_track")
            for item in profile.route_table.values()
        }
        # 每个 PID 产品只构造一次，积分器状态随产品对象跨帧保留。
        self._registry = {strategy: _BUILDERS[strategy](cfg.entity) for strategy in strategies}
        # 建造函数只负责静态增益初始化，黑板绑定统一在产品全部创建后完成。
        # 各产品共享 runtime，但只能读写自身控制契约内的字段。
        if self._binding_runtime is None:
            raise ValueError("PosTrackManager 尚未绑定运行环境")
        for strategy in self._registry.values():
            strategy.bind(self._binding_runtime)
        self._binding_runtime = None
        self._profile = profile

    def step(self) -> None:
        """严格查询完整表并执行缓存产品。"""
        if self._cmd is None:
            raise ValueError("PosTrackManager 尚未绑定命令端口")
        profile = self._profile
        if profile is None:
            raise ValueError("PosTrackManager 尚未初始化")
        strategy_type = _require_strategy(
            profile.require_strategies(self._cmd.stage, self._cmd.step).pos_track,
            "route_table.pos_track",
        )
        strategy = self._registry.get(strategy_type)
        if strategy is None:
            raise ValueError(f"位置跟踪策略未配置: {strategy_type.name}")
        # 运行期仅切换缓存引用，PID 积分状态不会因任务阶段变化丢失。
        strategy.step()

    def reset(self) -> None:
        """复位全部缓存产品。注意：保留配置和实例。"""
        # 重置每个产品的动态状态，但不重新读取配置或重建控制器。
        for strategy in self._registry.values():
            strategy.reset()


def _require_strategy(value: object, field_name: str) -> PosTrackStrategyE:
    """校验控制产品策略枚举。注意：禁止普通整数绕过配置语义。"""
    if not isinstance(value, PosTrackStrategyE):
        raise ValueError(f"{field_name} 必须是 PosTrackStrategyE")
    if value not in _BUILDERS:
        raise ValueError(f"不支持的位置跟踪策略: {value!r}")
    return value
