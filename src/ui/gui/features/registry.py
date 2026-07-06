"""GUI 可选功能注册表。注意：profile 到具体实现的选择只在本模块发生。"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from typing import TYPE_CHECKING, Any

from src.ui.gui.features.contracts import ControlMonitorFeature, DataAnalysisFeature, Situation3DFeature
from src.ui.gui.features.profile import FULL_PROFILE, LITE_PROFILE, load_feature_profile, normalize_feature_profile

if TYPE_CHECKING:
    from src.ui.gui.main_window import MainWindow
    from src.ui.gui.view_models import Snapshot

_FEATURE_CLASSES = {
    # full 档位只声明模块路径，不在文件加载时导入真实窗口模块。
    FULL_PROFILE: {
        "control_monitor": ("src.ui.gui.features.full.control_monitor", "FullControlMonitorFeature"),
        "data_analysis": ("src.ui.gui.features.full.data_analysis", "FullDataAnalysisFeature"),
        "situation3d": ("src.ui.gui.features.full.situation3d", "FullSituation3DFeature"),
    },
    # lite 档位指向 disabled 实现，确保裁剪版构造主窗口时不触碰重依赖。
    LITE_PROFILE: {
        "control_monitor": ("src.ui.gui.features.disabled.control_monitor", "DisabledControlMonitorFeature"),
        "data_analysis": ("src.ui.gui.features.disabled.data_analysis", "DisabledDataAnalysisFeature"),
        "situation3d": ("src.ui.gui.features.disabled.situation3d", "DisabledSituation3DFeature"),
    },
}


@dataclass
class GuiFeatureRegistry:
    """GUI 功能注册表。注意：MainWindow 只通过本对象触发可选功能生命周期。"""

    profile: str
    control_monitor: ControlMonitorFeature
    data_analysis: DataAnalysisFeature
    situation3d: Situation3DFeature

    def register_primary_menus(self, window: MainWindow) -> None:
        """注册避障菜单之前的可选菜单。注意：保持全量版菜单顺序不变。"""

        # 控制监控和数据分析在避障之前，避免全量版顶部入口顺序漂移。
        self.control_monitor.register_menu(window)
        self.data_analysis.register_menu(window)

    def register_secondary_menus(self, window: MainWindow) -> None:
        """注册避障菜单之后的可选菜单。注意：当前只包含 3D 态势。"""

        # 3D 态势原本位于避障之后，拆到 feature 后仍维持这个位置。
        self.situation3d.register_menu(window)

    def on_snapshot_updated(self, window: MainWindow, snapshot: Snapshot) -> None:
        """处理主快照刷新。注意：只有已打开的可选功能会消费快照。"""

        # 目前只有 3D 态势需要订阅实时快照，其他功能不加入刷新链路。
        self.situation3d.update_snapshot(window, snapshot)

    def on_controller_ready(self, window: MainWindow) -> None:
        """处理控制器可用事件。注意：用于实时监控窗口延迟绑定控制器。"""

        # 实时监控窗口可能在 READY 态打开，控制器就绪时统一补绑定。
        self.control_monitor.follow_if_open(window)

    def on_controller_unavailable(self) -> None:
        """处理控制器重置或配置切换事件。注意：避免监控窗口持有旧控制器。"""

        # 配置切换后旧控制器快照不再可信，已打开监控窗口必须解绑。
        self.control_monitor.unfollow()

    def close(self) -> None:
        """关闭所有可选功能窗口。注意：主窗口关闭时统一调用。"""

        # 固定顺序关闭窗口，避免监控窗口在控制器释放后仍轮询。
        self.control_monitor.close()
        self.data_analysis.close()
        self.situation3d.close()


def _build_feature(module_name: str, class_name: str) -> Any:
    """按模块名和类名构造功能实现。注意：动态导入是编译裁剪的关键边界。"""

    # import_module 是裁剪版不导入 full 模块的关键，不能改回静态 import。
    module = import_module(module_name)
    feature_class = getattr(module, class_name)
    return feature_class()


def build_gui_feature_registry(profile: str | None = None) -> GuiFeatureRegistry:
    """按功能档位构造 GUI 注册表。注意：未指定时读取运行时环境变量。"""

    active_profile = load_feature_profile() if profile is None else normalize_feature_profile(profile)
    # 先规整 profile，再取表，避免未知档位落到隐式默认行为。
    classes = _FEATURE_CLASSES[active_profile]
    return GuiFeatureRegistry(
        profile=active_profile,
        control_monitor=_build_feature(*classes["control_monitor"]),
        data_analysis=_build_feature(*classes["data_analysis"]),
        situation3d=_build_feature(*classes["situation3d"]),
    )
