"""GUI 可选功能注册包。注意：功能裁剪入口统一从 registry 构造。"""

from src.ui.gui.features.registry import GuiFeatureRegistry, build_gui_feature_registry

__all__ = ["GuiFeatureRegistry", "build_gui_feature_registry"]
