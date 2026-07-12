#!/usr/bin/env bash
# macOS 全量版发布构建入口：在当前 macOS 架构上生成 .app，不支持交叉构建。
# macOS app 必须使用 onedir，避免 onefile 与 app 沙盒及签名机制冲突。

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${PYTHON:-python3}"
app_name="${APP_NAME:-编队仿真}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --python)
            python_bin="$2"
            shift 2
            ;;
        --app-name)
            app_name="$2"
            shift 2
            ;;
        *)
            echo "未知参数：$1" >&2
            exit 2
            ;;
    esac
done

cd "$project_root"
release_app_dir="$project_root/dist/$app_name.app"
runtime_hook="$project_root/scripts/pyinstaller_hooks/set_full_feature_profile.py"
model_assets_dir="$project_root/src/ui/gui/situation3d/qml/assets"
export PYINSTALLER_CONFIG_DIR="$project_root/build/pyinstaller-cache-macos"

"$python_bin" -m pip install --upgrade pip
"$python_bin" -m pip install -r requirements-gui.txt
pyside_root="$("$python_bin" -c 'from pathlib import Path; import PySide6; print(Path(PySide6.__file__).resolve().parent)')"
rm -rf "$release_app_dir"

"$python_bin" -m PyInstaller \
    --noconfirm \
    --clean \
    --windowed \
    --onedir \
    --name "$app_name" \
    --paths "$project_root" \
    --workpath "$project_root/build/full-release-macos" \
    --runtime-hook "$runtime_hook" \
    --hidden-import src.ui.gui.features.full.control_monitor \
    --hidden-import src.ui.gui.features.full.data_analysis \
    --hidden-import src.ui.gui.features.full.situation3d \
    --add-data "src/ui/gui/situation3d/qml/Situation3DView.qml:src/ui/gui/situation3d/qml" \
    --add-data "src/ui/gui/situation3d/qml/assets/terrain_detail_normal.png:src/ui/gui/situation3d/qml/assets" \
    --add-binary "$pyside_root/Qt/plugins/assetimporters:PySide6/Qt/plugins/assetimporters" \
    --add-binary "$pyside_root/Qt/plugins/geometryloaders:PySide6/Qt/plugins/geometryloaders" \
    --add-binary "$pyside_root/Qt/plugins/renderers:PySide6/Qt/plugins/renderers" \
    src/ui/gui/main_window.py

find "$model_assets_dir" -maxdepth 1 -type f -name '*.glb' -exec cp {} "$release_app_dir/Contents/MacOS/" \;

echo "全量版 macOS app：$release_app_dir"
echo "外置 3D 模型：$release_app_dir/Contents/MacOS/*.glb"
