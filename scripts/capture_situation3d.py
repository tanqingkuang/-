"""抓取正式 3D 态势 QML 真实渲染帧。注意：用于阶段视觉验收，不走 offscreen platform。"""

from __future__ import annotations

import argparse
import os
import sys
import threading
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    # 允许从 scripts/ 入口直接运行，同时保持导入正式 src 包。
    sys.path.insert(0, str(PROJECT_ROOT))

from PySide6.QtCore import QEventLoop, QTimer
from PySide6.QtWidgets import QApplication

from src.ui.gui.simulation_adapter import ControllerSimulationAdapter
from src.ui.gui.situation3d.window import Situation3DWindow
from src.ui.gui.view_models import Snapshot, default_project_root


CAPTURE_WIDTH_PX = 1600
CAPTURE_HEIGHT_PX = 900
DEFAULT_WAIT_MS = 1800
DEFAULT_TIMEOUT_MS = 120000
DEFAULT_CONFIG = Path("configs/mountain_demo.json")
DEFAULT_OUTPUTS = {
    "top": Path("docs/assets/p1/p1_top_global.png"),
    "oblique": Path("docs/assets/p1/p1_oblique_global.png"),
    "follow": Path("docs/assets/p1/p1_low_follow_valley.png"),
    "parity": Path("docs/assets/p1/p1_style_parity.png"),
}


def parse_args() -> argparse.Namespace:
    """解析命令行参数。注意：默认输出覆盖 P1 验收截图。"""

    parser = argparse.ArgumentParser(description="抓取正式 Situation3DView.qml 的真实渲染截图。")
    parser.add_argument("--view", choices=("top", "oblique", "follow", "parity"), required=True, help="相机视角。")
    parser.add_argument("--output", type=Path, default=None, help="PNG 输出路径；缺省使用 docs/assets/p1 同名文件。")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="仿真配置路径，默认 configs/mountain_demo.json。")
    parser.add_argument("--wait-ms", type=int, default=DEFAULT_WAIT_MS, help="窗口显示后等待渲染稳定的毫秒数。")
    parser.add_argument("--timeout-ms", type=int, default=DEFAULT_TIMEOUT_MS, help="硬超时毫秒数，超时直接退出进程。")
    parser.add_argument("--backend", default="d3d11", help="QSG_RHI_BACKEND，默认 d3d11。")
    return parser.parse_args()


def configure_environment(backend: str) -> None:
    """设置 Qt 渲染环境。注意：主动移除 offscreen，确保抓真实窗口。"""

    if os.environ.get("QT_QPA_PLATFORM", "").lower() == "offscreen":
        # 该脚本的验收价值来自真实窗口；继承 offscreen 会导致 Quick3D 抓不到内容。
        os.environ.pop("QT_QPA_PLATFORM", None)
    os.environ.setdefault("QSG_RHI_BACKEND", backend)
    os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "0")


def load_snapshot(config_path: Path) -> tuple[ControllerSimulationAdapter, Snapshot]:
    """加载演示配置并返回正式 GUI Snapshot。注意：配置解析走 ControllerSimulationAdapter。"""

    adapter = ControllerSimulationAdapter()
    snapshot = adapter.load_config(str(config_path))
    if adapter.last_result_code != "OK":
        adapter.close()
        raise RuntimeError(f"加载配置失败：{adapter.last_result_code} {adapter.last_result_message}")
    return adapter, snapshot


def apply_view(root: object, view_name: str) -> None:
    """按验收视角设置 QML 相机。注意：只调用正式 QML 根对象属性和函数。"""

    if view_name == "top":
        # 俯视沿用 payload 焦点（内容中心），只调俯角和覆盖距离。
        root.setTopView()
        root.setProperty("pitch", -76.0)
        return
    if view_name == "oblique":
        # 全局斜俯：拉远看整个内容区，走廊与背景山链分层进雾。
        root.setProperty("cameraMode", "自由")
        root.setProperty("yaw", -38.0)
        root.setProperty("pitch", -30.0)
        root.setProperty("distance", 15500.0)
        return
    if view_name == "parity":
        # 对拍构图与 payload 默认机位一致：顺走廊低角度近景，山坡填充画面下部。
        root.setProperty("cameraMode", "自由")
        root.setProperty("focusX", 10500.0)
        root.setProperty("focusY", 900.0)
        root.setProperty("focusZ", 300.0)
        root.setProperty("yaw", -38.0)
        root.setProperty("pitch", -24.0)
        root.setProperty("distance", 10500.0)
        return
    # 跟随视角必须走正式入口:验证按钮真实行为(按长机角色跟踪),不允许手填相机绕过。
    root.setFollowView()
    if str(root.property("cameraMode")) != "跟随":
        raise RuntimeError("setFollowView 未进入跟随模式")


def wait_for_render(app: QApplication, wait_ms: int) -> None:
    """等待 Qt 渲染队列稳定。注意：真实窗口首帧需要给 Quick3D 上传 mesh。"""

    app.processEvents()
    loop = QEventLoop()
    QTimer.singleShot(max(100, wait_ms), loop.quit)
    loop.exec()
    app.processEvents()


def capture_view(snapshot: Snapshot, view_name: str, output_path: Path, wait_ms: int) -> None:
    """创建真实窗口、推送正式快照并保存抓图。注意：失败时抛出异常供 CI/人工发现。"""

    app = QApplication.instance() or QApplication(sys.argv)
    window = Situation3DWindow()
    window.resize(CAPTURE_WIDTH_PX, CAPTURE_HEIGHT_PX)
    window.set_snapshot(snapshot)
    window.show()
    window.raise_()
    window.activateWindow()
    wait_for_render(app, 300)
    root = window.quick_view.rootObject()
    if root is None:
        raise RuntimeError("QML rootObject 为空")
    apply_view(root, view_name)
    wait_for_render(app, wait_ms)
    image = window.quick_view.grabWindow()
    if image.isNull():
        raise RuntimeError("grabWindow 返回空图")
    if image.width() != CAPTURE_WIDTH_PX or image.height() != CAPTURE_HEIGHT_PX:
        image = image.scaled(CAPTURE_WIDTH_PX, CAPTURE_HEIGHT_PX)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not image.save(str(output_path), "PNG"):
        raise RuntimeError(f"保存截图失败：{output_path}")
    window.close()
    app.processEvents()
    app.quit()


def resolve_project_path(path: Path) -> Path:
    """按项目根解析相对路径。注意：脚本可从任意 cwd 启动。"""

    return path if path.is_absolute() else default_project_root() / path


def main() -> int:
    """脚本入口。注意：一次只抓一个视角，便于失败后重试。"""

    args = parse_args()
    timeout = threading.Timer(max(1.0, int(args.timeout_ms) / 1000.0), lambda: os._exit(124))
    timeout.daemon = True
    timeout.start()
    configure_environment(str(args.backend))
    config_path = resolve_project_path(args.config)
    output_path = resolve_project_path(args.output or DEFAULT_OUTPUTS[args.view])
    adapter, snapshot = load_snapshot(config_path)
    try:
        capture_view(snapshot, args.view, output_path, int(args.wait_ms))
    finally:
        adapter.close()
    size_kb = output_path.stat().st_size / 1024.0
    print(f"{output_path.resolve()}  {size_kb:.1f} KB", flush=True)
    # PySide6 解释器关停阶段会因 Quick3D 对象析构顺序死锁(曾把窗口挂成"未响应"20 分钟),
    # 截图落盘后直接硬退出;看门狗也保留到最后,不提前 cancel。
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
