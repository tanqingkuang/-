"""抓取正式 3D 态势 QML 真实渲染帧。注意：用于阶段视觉验收，不走 offscreen platform。"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    # 允许从 scripts/ 入口直接运行，同时保持导入正式 src 包。
    sys.path.insert(0, str(PROJECT_ROOT))

from PySide6.QtCore import QEventLoop, QTimer
from PySide6.QtWidgets import QApplication

from src.ui.gui.avoidance_tools import parse_avoidance_config
from src.ui.gui.simulation_adapter import ControllerSimulationAdapter
from src.ui.gui.situation3d.window import Situation3DWindow
from src.ui.gui.view_models import ObstacleView, Snapshot, default_project_root


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
    parser.add_argument("--timeout-ms", type=int, default=DEFAULT_TIMEOUT_MS, help="硬超时毫秒数，由父进程监督执行。")
    parser.add_argument("--backend", default="d3d11", help="QSG_RHI_BACKEND，默认 d3d11。")
    parser.add_argument("--child", action="store_true", help="内部参数:子进程实际执行抓图,外层父进程负责超时与产物校验。")
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


def capture_view(
    snapshot: Snapshot,
    view_name: str,
    output_path: Path,
    wait_ms: int,
    *,
    obstacles: list[ObstacleView] | None = None,
    clearance_m: float = 0.0,
) -> None:
    """创建真实窗口、推送正式快照并保存抓图。注意：障碍参数与主窗口保持同一链路。"""

    app = QApplication.instance() or QApplication(sys.argv)
    window = Situation3DWindow()
    window.resize(CAPTURE_WIDTH_PX, CAPTURE_HEIGHT_PX)
    window.set_snapshot(snapshot, obstacles=obstacles, clearance_m=clearance_m)
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


def _supervise(args: argparse.Namespace, output_path: Path) -> int:
    """父进程监督:超时杀子进程,校验退出码与产物后才接受截图。
    注意：threading.Timer 在 GIL 被原生调用占住时无法抢占,超时必须由独立进程执行。"""

    command = [sys.executable, "-X", "utf8", str(Path(__file__).resolve()), "--child", "--view", args.view]
    command += ["--output", str(output_path), "--config", str(args.config)]
    command += ["--wait-ms", str(int(args.wait_ms)), "--backend", str(args.backend)]
    marker = _success_marker(output_path)
    marker.unlink(missing_ok=True)
    try:
        completed = subprocess.run(command, timeout=max(5.0, int(args.timeout_ms) / 1000.0), check=False)
    except subprocess.TimeoutExpired:
        # subprocess.run 超时会先 kill 子进程再抛出;这里补一次树级清理防止残留窗口。
        if os.name == "nt":
            subprocess.run(["taskkill", "/F", "/T", "/FI", "WINDOWTITLE eq 3D态势*"], capture_output=True, check=False)
        print(f"抓图超时({args.timeout_ms}ms),产物不可信,已终止子进程", flush=True)
        return 124
    # PySide6 进程关停与 Qt 渲染线程存在竞态,退出码不可靠(直跑 0,监督下偶发异常码);
    # 成功判据 = 子进程截图落盘后写下的哨兵 + 产物内容校验,退出码仅作告警。
    if not marker.is_file():
        print(f"抓图子进程未写成功哨兵(退出码 {completed.returncode}),产物不可信", flush=True)
        return completed.returncode or 1
    marker.unlink(missing_ok=True)
    if completed.returncode != 0:
        print(f"提示:子进程退出码 {completed.returncode}(Qt 关停竞态),以产物校验为准", flush=True)
    return _validate_output(output_path)


def _success_marker(output_path: Path) -> Path:
    """返回截图成功哨兵路径。注意：哨兵只由子进程在落盘成功后写入。"""

    return output_path.with_name(output_path.name + ".ok")


def _validate_output(output_path: Path) -> int:
    """校验截图产物:存在、尺寸正确、内容非空(不是纯色黑帧)。"""

    if not output_path.is_file() or output_path.stat().st_size <= 0:
        print(f"产物缺失或为空: {output_path}", flush=True)
        return 2
    from PIL import Image
    import numpy as np

    image = np.asarray(Image.open(output_path).convert("RGB"), dtype=np.float32) / 255.0
    if image.shape[0] != CAPTURE_HEIGHT_PX or image.shape[1] != CAPTURE_WIDTH_PX:
        print(f"产物尺寸异常: {image.shape}", flush=True)
        return 3
    if float(image.std()) < 0.01:
        print(f"产物疑似纯色帧(std={image.std():.4f}),不可作为回归依据", flush=True)
        return 4
    print(f"{output_path.resolve()}  {output_path.stat().st_size / 1024.0:.1f} KB  校验通过", flush=True)
    return 0


def _run_child(args: argparse.Namespace, output_path: Path) -> int:
    """子进程实际抓图。注意：截图落盘后硬退出,规避 PySide6 关停阶段析构死锁。"""

    configure_environment(str(args.backend))
    config_path = resolve_project_path(args.config)
    adapter, snapshot = load_snapshot(config_path)
    obstacles, clearance_m = parse_avoidance_config(str(config_path))
    try:
        if snapshot.terrain_display_file:
            # 抓图脚本没有 10Hz 刷新循环,必须阻塞等高度场就绪,否则截到占位地形。
            from src.ui.gui.situation3d import scene_data
            from src.ui.gui.situation3d.terrain_field import get_terrain_field, load_terrain_layout

            layout = load_terrain_layout(snapshot.terrain_display_file)
            get_terrain_field(snapshot.terrain_display_file, resolution=scene_data._layout_resolution(layout))
        capture_view(
            snapshot,
            args.view,
            output_path,
            int(args.wait_ms),
            obstacles=obstacles,
            clearance_m=clearance_m,
        )
    finally:
        adapter.close()
    print(f"{output_path.resolve()}  {output_path.stat().st_size / 1024.0:.1f} KB", flush=True)
    # 成功哨兵必须在硬退出前写入:父进程以它为成功判据,退出码受 Qt 关停竞态影响不可靠。
    _success_marker(output_path).write_text("ok", encoding="utf-8")
    os._exit(0)


def main() -> int:
    """脚本入口。注意：默认作为父进程监督执行,一次只抓一个视角。"""

    args = parse_args()
    output_path = resolve_project_path(args.output or DEFAULT_OUTPUTS[args.view])
    if args.child:
        return _run_child(args, output_path)
    return _supervise(args, output_path)


if __name__ == "__main__":
    raise SystemExit(main())
