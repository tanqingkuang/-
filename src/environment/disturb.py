"""扰动管理公共能力。注意：运行级不确定性与动态扰动共用同一管理入口。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Mapping

from src.environment.comm import CommunicationChannel
from src.environment.model import ModelIterator

if TYPE_CHECKING:
    from src.runner.sim_control_types import SimulationEvent


UncertaintyApply = Callable[
    [ModelIterator, CommunicationChannel, Mapping[str, object]],
    None,
]


@dataclass(frozen=True)
class UncertaintyCase:
    """一个 seed 对应的不确定性算例。注意：apply 决定实际注入位置。"""

    seed: int
    name: str
    params: Mapping[str, object]
    apply: UncertaintyApply


def _apply_turbulence_uncertainty(
    model: ModelIterator,
    comm: CommunicationChannel,
    params: Mapping[str, object],
) -> None:
    """向模型注入全局轻度紊流。注意：通信句柄为统一回调签名的预留参数。"""

    del comm
    model.set_uncertainty_turbulence({"params": dict(params)})


def _apply_link_loss_uncertainty(
    model: ModelIterator,
    comm: CommunicationChannel,
    params: Mapping[str, object],
) -> None:
    """设置全链路初始化丢包率。注意：模型句柄为统一回调签名的预留参数。"""

    del model
    comm.set_uncertainty_loss_rate(float(params["loss_rate"]))


def _apply_link_frame_rate_uncertainty(
    model: ModelIterator,
    comm: CommunicationChannel,
    params: Mapping[str, object],
) -> None:
    """设置全链路初始化发送帧频。注意：模型句柄为统一回调签名的预留参数。"""

    del model
    comm.set_uncertainty_frame_rate_hz(float(params["frame_rate_hz"]))


def _apply_link_latency_uncertainty(
    model: ModelIterator,
    comm: CommunicationChannel,
    params: Mapping[str, object],
) -> None:
    """设置全链路初始化时延。注意：模型句柄为统一回调签名的预留参数。"""

    del model
    comm.set_uncertainty_latency_ms(float(params["latency_ms"]))


_UNCERTAINTY_REGISTRY: dict[int, UncertaintyCase] = {
    1: UncertaintyCase(
        seed=1,
        name="全链路丢包率 2.3%",
        params={"loss_rate": 0.023},
        apply=_apply_link_loss_uncertainty,
    ),
    2: UncertaintyCase(
        seed=2,
        name="全局轻度紊流风",
        params={
            "horizontal_sigma_mps": 0.8,
            "vertical_sigma_mps": 0.3,
            "correlation_time_s": 2.0,
        },
        apply=_apply_turbulence_uncertainty,
    ),
    3: UncertaintyCase(
        seed=3,
        name="全链路发送帧频 10 Hz",
        params={"frame_rate_hz": 10.0},
        apply=_apply_link_frame_rate_uncertainty,
    ),
    4: UncertaintyCase(
        seed=4,
        name="全链路时延 50 ms",
        params={"latency_ms": 50.0},
        apply=_apply_link_latency_uncertainty,
    ),
}


class DisturbanceManager:
    """统一管理运行级不确定性和动态扰动。注意：未注册 seed 保持标称状态。"""

    def __init__(self) -> None:
        """初始化管理器。注意：构造阶段不向模型或通信注入任何内容。"""

        self._uncertainty_case: UncertaintyCase | None = None

    @property
    def uncertainty_case(self) -> UncertaintyCase | None:
        """返回当前 seed 对应的不确定性算例。"""

        return self._uncertainty_case

    def apply_uncertainty(
        self,
        seed: int,
        model: ModelIterator,
        comm: CommunicationChannel,
    ) -> UncertaintyCase | None:
        """查表并应用一次运行级不确定性。注意：未注册 seed 是兼容性 no-op。"""

        # 先保存命中的算例，便于控制器、日志或测试读取本次实际选择。
        uncertainty_case = _UNCERTAINTY_REGISTRY.get(seed)
        self._uncertainty_case = uncertainty_case
        if uncertainty_case is not None:
            uncertainty_case.apply(model, comm, uncertainty_case.params)
        return uncertainty_case

    def tick(self, time_s: float, dt_s: float) -> list[SimulationEvent]:
        """推进模块内部时钟或动态状态一个周期。注意：调用频率应与仿真步长一致。"""

        raise NotImplementedError
