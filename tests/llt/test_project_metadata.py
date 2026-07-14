"""项目元数据与依赖入口回归测试。"""

from __future__ import annotations

from pathlib import Path
import re
import tomllib
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PYPROJECT_PATH = PROJECT_ROOT / "pyproject.toml"
REQUIREMENTS_PATH = PROJECT_ROOT / "requirements-gui.txt"


def _dependency_names(requirements: list[str]) -> set[str]:
    """提取规范化依赖名。注意：这里只检查仓库内受控的简单 PEP 508 声明。"""

    names: set[str] = set()
    for requirement in requirements:
        name = re.split(r"[<>=!~; \[]", requirement, maxsplit=1)[0]
        names.add(name.strip().lower().replace("_", "-"))
    return names


class ProjectMetadataTests(unittest.TestCase):
    """确保 pyproject 是项目元数据、依赖和工具配置的唯一事实源。"""

    @classmethod
    def setUpClass(cls) -> None:
        """读取待检查配置。注意：文件缺失应直接使测试失败。"""

        cls.config = tomllib.loads(PYPROJECT_PATH.read_text(encoding="utf-8"))

    def test_project_metadata_describes_current_release(self) -> None:
        """项目名、版本、说明与 Python 基线应完整声明。"""

        project = self.config["project"]

        self.assertEqual(project["name"], "formation-simulation-platform")
        self.assertEqual(project["version"], "1.0.0")
        self.assertEqual(project["readme"], "README.md")
        self.assertEqual(project["requires-python"], ">=3.12")
        self.assertTrue(project["description"])

    def test_dependencies_are_grouped_by_purpose(self) -> None:
        """运行、测试、静态检查和构建依赖应分组，旧 requirements 只保留兼容转发。"""

        project = self.config["project"]
        optional = project["optional-dependencies"]

        self.assertEqual(
            _dependency_names(project["dependencies"]),
            {"numpy", "pyside6", "pyside6-addons"},
        )
        self.assertEqual(_dependency_names(optional["test"]), {"pytest", "pytest-cov"})
        self.assertEqual(_dependency_names(optional["lint"]), {"ruff", "mypy"})
        self.assertEqual(_dependency_names(optional["build"]), {"pyinstaller"})

        effective_lines = [
            line.strip()
            for line in REQUIREMENTS_PATH.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]
        self.assertEqual(effective_lines, [".[build]"])

    def test_build_and_test_tools_are_configured_centrally(self) -> None:
        """构建后端、包发现和 pytest 默认目录应由 pyproject 统一配置。"""

        self.assertEqual(self.config["build-system"]["build-backend"], "setuptools.build_meta")
        self.assertEqual(self.config["tool"]["setuptools"]["packages"]["find"]["include"], ["src*"])
        self.assertEqual(self.config["tool"]["pytest"]["ini_options"]["testpaths"], ["tests/llt"])

    def test_automation_installs_named_dependency_groups(self) -> None:
        """CI、开发入口和发布入口应直接使用 pyproject 中的依赖分组。"""

        test_gate = (PROJECT_ROOT / ".github" / "workflows" / "test-gate.yml").read_text(encoding="utf-8")
        self.assertIn('python -m pip install ".[test]"', test_gate)
        self.assertIn('python -m pip install ".[lint]"', test_gate)
        self.assertNotIn("requirements-gui.txt ruff mypy", test_gate)

        for name in ("run_windows_full_dev.ps1", "run_windows_lite_dev.ps1"):
            script = (PROJECT_ROOT / "scripts" / name).read_text(encoding="utf-8")
            self.assertIn("-m pip install .", script)
            self.assertNotIn("requirements-gui.txt", script)

        for name in ("build_windows_full_release.ps1", "build_windows_lite_release.ps1"):
            script = (PROJECT_ROOT / "scripts" / name).read_text(encoding="utf-8")
            self.assertIn('-m pip install ".[build]"', script)
            self.assertNotIn("requirements-gui.txt", script)

        for name in ("run_macos_full_dev.sh", "run_macos_lite_dev.sh"):
            script = (PROJECT_ROOT / "scripts" / name).read_text(encoding="utf-8")
            self.assertIn('-m pip install "$project_root"', script)
            self.assertNotIn("requirements-gui.txt", script)

        for name in ("build_macos_full_release.sh", "build_macos_lite_release.sh"):
            script = (PROJECT_ROOT / "scripts" / name).read_text(encoding="utf-8")
            self.assertIn('-m pip install "$project_root[build]"', script)
            self.assertNotIn("requirements-gui.txt", script)


if __name__ == "__main__":
    unittest.main()
