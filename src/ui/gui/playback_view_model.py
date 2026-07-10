"""播放控制 ViewModel。注意：本模块只含纯 Python 状态与规则，不依赖 Qt。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from src.ui.gui.view_models import playback_rate_to_slider_value, slider_value_to_playback_rate

PlaybackSliderSource = Literal["user", "program"]
PlaybackCommand = Literal["start", "pause", "none"]


@dataclass(frozen=True)
class PlaybackControlUpdate:
    """播放控件需要执行的一次更新。注意：None 表示该输出不需要外部动作。"""

    display_rate: float
    slider_value: int | None = None
    controller_rate: float | None = None

    @property
    def label_text(self) -> str:
        """生成倍率显示文案。注意：格式保持 GUI 既有一位小数约定。"""

        return f"{self.display_rate:.1f}x"


@dataclass(frozen=True)
class PlaybackPauseDecision:
    """一次播放/暂停按钮语义判断。注意：只描述应下发的控制器动作。"""

    command: PlaybackCommand

    @property
    def should_pause(self) -> bool:
        """判断是否应调用控制器 pause。注意：PAUSED 下仍是 pause 幂等请求。"""

        return self.command == "pause"

    @property
    def should_start(self) -> bool:
        """判断是否应调用控制器 start。注意：PAUSED 下继续运行才是 start。"""

        return self.command == "start"


class PlaybackViewModel:
    """封装播放倍率、滑条回填和播放暂停语义。注意：不读写任何 GUI 控件。"""

    def __init__(self, initial_rate: float = 1.0) -> None:
        """初始化播放控制状态。注意：默认倍率与控制器默认值保持一致。"""

        # current_rate 以控制器真实倍率为准，允许落在滑条档位之间。
        self.current_rate = float(initial_rate)
        # paused 只记录最近快照语义，具体启停仍由控制器执行。
        self.paused = False
        # 回填滑条期间 valueChanged 可能同步触发，用 depth 区分用户输入。
        self._program_slider_sync_depth = 0

    def begin_programmatic_slider_sync(self, rate: float) -> PlaybackControlUpdate:
        """开始一次程序回填滑条。注意：期间滑条信号不得重复下发倍率。"""

        # 程序回填先更新真实倍率，再让控件追上这个状态。
        self.current_rate = float(rate)
        self._program_slider_sync_depth += 1
        return PlaybackControlUpdate(
            display_rate=self.current_rate,
            slider_value=playback_rate_to_slider_value(self.current_rate),
            controller_rate=None,
        )

    def finish_programmatic_slider_sync(self) -> None:
        """结束一次程序回填滑条。注意：异常路径也应调用以恢复用户输入识别。"""

        # 用 depth 而非 bool，避免嵌套回填时过早解除防回环状态。
        self._program_slider_sync_depth = max(0, self._program_slider_sync_depth - 1)

    def on_slider_changed(
        self,
        value: int,
        *,
        source: PlaybackSliderSource = "user",
    ) -> PlaybackControlUpdate:
        """处理倍率滑条变化。注意：程序来源只刷新显示，不向控制器回写。"""

        # 程序来源只承认当前倍率，不把滑条信号反向写回控制器。
        if source == "program" or self._program_slider_sync_depth > 0:
            return PlaybackControlUpdate(display_rate=self.current_rate, controller_rate=None)

        # 用户拖动时按离散档位夹紧并下发；非法值也落到最近合法档位。
        # 这里返回滑条值，便于调用方把直接传入的非法值纠正到合法档。
        snapped_rate = slider_value_to_playback_rate(value)
        self.current_rate = snapped_rate
        return PlaybackControlUpdate(
            display_rate=snapped_rate,
            slider_value=playback_rate_to_slider_value(snapped_rate),
            controller_rate=snapped_rate,
        )

    def on_config_loaded(self, controller_rate: float) -> PlaybackControlUpdate:
        """加载配置后同步控制器真实倍率。注意：标签保留真实倍率，滑条吸附最近档。"""

        # 配置加载后的权威值来自控制器，而不是滑条最近档位。
        self.current_rate = float(controller_rate)
        return PlaybackControlUpdate(
            display_rate=self.current_rate,
            slider_value=playback_rate_to_slider_value(self.current_rate),
            controller_rate=None,
        )

    def on_rate_requested(self, rate: float) -> PlaybackControlUpdate:
        """记录外部已决定的播放倍率。注意：用于 adapter 直接下发控制器前统一状态。"""

        # 外部已经决定要下发倍率，这里只统一记录和回传。
        self.current_rate = float(rate)
        return PlaybackControlUpdate(
            display_rate=self.current_rate,
            slider_value=playback_rate_to_slider_value(self.current_rate),
            controller_rate=self.current_rate,
        )

    def on_reset(self) -> PlaybackControlUpdate:
        """处理重置后的倍率语义。注意：重置必须保持当前倍率并重新下发给控制器。"""

        # reset 会重建控制器内部配置，必须把当前倍率再下发一次。
        self.paused = False
        return PlaybackControlUpdate(
            display_rate=self.current_rate,
            slider_value=playback_rate_to_slider_value(self.current_rate),
            controller_rate=self.current_rate,
        )

    def command_for_toggle(self, run_state: str) -> PlaybackPauseDecision:
        """判断播放按钮点击后的动作。注意：按钮语义由当前运行态决定。"""

        # 播放按钮显示下一步动作：运行中点击才是暂停。
        if run_state == "RUNNING":
            return PlaybackPauseDecision("pause")
        # READY/PAUSED 下按钮语义都是开始或继续，具体合法性由控制器兜底。
        if run_state in {"READY", "PAUSED", "UNLOADED"}:
            return PlaybackPauseDecision("start")
        return PlaybackPauseDecision("none")

    def command_for_pause_request(self, run_state: str) -> PlaybackPauseDecision:
        """判断直接 pause 请求的动作。注意：PAUSED 下不得被解释为恢复运行。"""

        # 直接 pause 请求保持幂等，PAUSED 不会被翻译成 start。
        if run_state in {"RUNNING", "PAUSED"}:
            return PlaybackPauseDecision("pause")
        return PlaybackPauseDecision("none")

    def on_snapshot_state(self, run_state: str) -> None:
        """根据快照运行态同步暂停标记。注意：仅记录状态，不触发外部动作。"""

        # 该标记服务纯函数层断言，不抢控制器状态机的职责。
        self.paused = run_state == "PAUSED"
