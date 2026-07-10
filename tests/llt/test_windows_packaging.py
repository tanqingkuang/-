"""Windows 发布打包脚本回归测试。"""

from __future__ import annotations

from pathlib import Path
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
FULL_RELEASE_SCRIPT = PROJECT_ROOT / "scripts" / "build_windows_full_release.ps1"


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


if __name__ == "__main__":
    unittest.main()
