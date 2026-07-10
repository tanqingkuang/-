"""避障面板 ViewModel。注意：本模块只含纯 Python 显示规则，不依赖 Qt。"""

from __future__ import annotations


def simplify_should_follow(
    source_is_clearance: bool,
    params_present: bool,
    explicit: bool,
) -> bool:
    """判断拉直间距是否跟随安全间距。注意：显式配置或手改后不再跟随。"""

    # 只有安全间距控件触发且当前参数未显式指定拉直间距时，才允许联动回填。
    return source_is_clearance and params_present and not explicit


def param_widgets_enabled(has_params: bool) -> bool:
    """判断规划参数控件是否可用。注意：缺少有效避障参数时统一禁用。"""

    # 参数存在性就是整组规划参数控件的唯一开放条件。
    return has_params


def export_enabled(has_params: bool, has_preview: bool) -> bool:
    """判断航线导出按钮是否可用。注意：参数和有效预览必须同时存在。"""

    # 导出依赖当前参数上下文和未失效的预览航线。
    return has_params and has_preview


def adopt_enabled(has_preview: bool) -> bool:
    """判断航线采用按钮是否可用。注意：只有未失效的预览航线可采用。"""

    # 预览失效后必须立即关闭采用入口。
    return has_preview


def avoidance_status_text(enabled_count: int, total_count: int) -> str:
    """生成避障面板空闲状态文案。注意：既有用户可见文字必须保持不变。"""

    # 空列表显示配置缺失提示，非空列表显示勾选计数和下一步操作。
    if total_count == 0:
        return "未加载障碍：当前配置无 avoidance.obstacles。"
    return (
        f"已勾选 {enabled_count}/{total_count} 个障碍。\n"
        "设置参数后点「生成航线」预览，满意再「采用航线」。"
    )
