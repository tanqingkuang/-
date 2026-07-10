"""侧视图控制 ViewModel。注意：本模块只含纯 Python 状态与规则，不依赖 Qt。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SideViewControlUpdate:
    """侧视图控件需要执行的一次同步。注意：apply_locked 为 None 表示视图状态已一致。"""

    lock_enabled: bool
    lock_checked: bool
    apply_locked: bool | None
    angle_value: int
    angle_controls_enabled: bool


@dataclass(frozen=True)
class SideViewLockUpdate:
    """锁定切换需要下发的动作。注意：view_angle_deg 为 None 表示无需回写当前角度。"""

    apply_locked: bool
    view_angle_deg: float | None


class SideViewControlViewModel:
    """封装侧视图锁定偏好与控件同步规则。注意：不读写任何 GUI 或视图对象。"""

    def __init__(self) -> None:
        """初始化侧视图控制状态。注意：默认偏好与原窗口锁定默认值保持一致。"""

        # 用户默认希望按当前航段锁定；暂不可用时仍保留这项偏好。
        self.preferred = True

    def on_lock_toggled(
        self,
        checked: bool,
        lock_enabled: bool,
        current_angle: float,
    ) -> SideViewLockUpdate:
        """处理锁定切换。注意：只有控件可用时才把选择记为用户偏好。"""

        # 禁用态变化来自程序同步，不代表用户主动修改偏好。
        if lock_enabled:
            self.preferred = checked
        # 解除锁定时保存视图的实际投影角，避免自由视角跳回旧的手动角度。
        view_angle_deg = None if checked else current_angle
        return SideViewLockUpdate(
            apply_locked=checked,
            view_angle_deg=view_angle_deg,
        )

    def on_sync(
        self,
        lock_available: bool,
        side_view_locked: bool,
        current_angle: float,
    ) -> SideViewControlUpdate:
        """生成侧视图控件同步状态。注意：可用性撤销不清除用户锁定偏好。"""

        locked = lock_available and self.preferred
        # 只在视图真实锁状态不一致时要求绑定层写入，避免重复刷新投影。
        apply_locked = locked if side_view_locked != locked else None
        return SideViewControlUpdate(
            lock_enabled=lock_available,
            lock_checked=locked,
            apply_locked=apply_locked,
            angle_value=normalized_view_angle(current_angle),
            angle_controls_enabled=not locked,
        )


def normalized_view_angle(angle_deg: float) -> int:
    """把视角归一化为整数角度。注意：保持原先先 round 再模 360 的顺序。"""

    # Python round 的半偶数规则属于既有行为，不能替换为自定义四舍五入。
    return round(angle_deg) % 360


def geodetic_click_text(geodetic: tuple[float, float] | None) -> str:
    """格式化俯视图点击坐标。注意：输入顺序为纬度、经度，显示时经度在前。"""

    # None 只表示当前配置缺少地理原点，不把它伪装成零坐标。
    if geodetic is None:
        return "当前配置无经纬 origin"
    latitude_deg, longitude_deg = geodetic
    # 复制文案沿用七位小数，满足原坐标显示精度与经纬顺序。
    return f"{longitude_deg:.7f}, {latitude_deg:.7f}"
