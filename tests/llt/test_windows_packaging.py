"""Windows 发布打包脚本回归测试。"""

from __future__ import annotations

from pathlib import Path
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
FULL_RELEASE_SCRIPT = PROJECT_ROOT / "scripts" / "build_windows_full_release.ps1"
LITE_RELEASE_SCRIPT = PROJECT_ROOT / "scripts" / "build_windows_lite_release.ps1"
WINDOWS_EXE_WORKFLOW = PROJECT_ROOT / ".github" / "workflows" / "build-windows-exe.yml"
APP_ICON_PNG = PROJECT_ROOT / "src" / "ui" / "gui" / "assets" / "app_icon.png"
APP_ICON_ICO = PROJECT_ROOT / "src" / "ui" / "gui" / "assets" / "app_icon.ico"


class WindowsPackagingScriptTests(unittest.TestCase):
    """验证 Windows 全量版发布包的 3D 资产打包契约。"""

    def test_full_release_keeps_glb_external_but_bundles_import_plugins(self) -> None:
        """全量 exe 应外置 glb，并内置 Qt Quick 3D 加载 glb 所需插件。"""

        script = FULL_RELEASE_SCRIPT.read_text(encoding="utf-8")

        self.assertIn("plugins\\assetimporters", script)
        self.assertIn("plugins\\geometryloaders", script)
        self.assertIn("plugins\\renderers", script)
        self.assertIn("$PySideRoot = @(& $Python -c", script)
        self.assertIn("src/ui/gui/situation3d/qml/Situation3DView.qml;src/ui/gui/situation3d/qml", script)
        self.assertIn("Get-ChildItem -LiteralPath $AircraftModelSourceDir -Filter *.glb", script)
        self.assertIn("Copy-Item -LiteralPath $_.FullName -Destination $DistDir -Force", script)
        self.assertNotIn("src/ui/gui/situation3d/qml;src/ui/gui/situation3d/qml", script)

    def test_full_release_artifact_uploads_exe_and_external_glb(self) -> None:
        """全量 GitHub artifact 应同时包含 exe 和平铺 glb，避免客户包漏模型。"""

        workflow = WINDOWS_EXE_WORKFLOW.read_text(encoding="utf-8")

        self.assertIn("artifact_paths: |\n              dist/编队仿真.exe\n              dist/*.glb", workflow)
        self.assertIn("artifact_paths: dist/编队仿真-裁剪版.exe", workflow)
        self.assertIn("path: ${{ matrix.artifact_paths }}", workflow)
        self.assertNotIn("path: ${{ matrix.exe_path }}", workflow)

    def test_release_scripts_bundle_project_application_icon(self) -> None:
        """全量版和裁剪版 exe 都应使用并内置同一套项目图标资源。"""
        self.assertTrue(APP_ICON_PNG.is_file())
        self.assertTrue(APP_ICON_ICO.is_file())
        for script_path in (FULL_RELEASE_SCRIPT, LITE_RELEASE_SCRIPT):
            script = script_path.read_text(encoding="utf-8")
            self.assertIn('$AppIconPath = Join-Path $ProjectRoot "src\\ui\\gui\\assets\\app_icon.ico"', script)
            self.assertIn("--icon $AppIconPath", script)
            self.assertIn("src/ui/gui/assets/app_icon.png;src/ui/gui/assets", script)

    def test_release_scripts_keep_pure_gui_entrypoint(self) -> None:
        """Windows GUI exe 应继续使用 main_window，不混入无界面批处理入口。"""

        for script_path in (FULL_RELEASE_SCRIPT, LITE_RELEASE_SCRIPT):
            script = script_path.read_text(encoding="utf-8")
            self.assertIn("src/ui/gui/main_window.py", script)
            self.assertNotIn("src/main.py", script)

if __name__ == "__main__":
    unittest.main()
