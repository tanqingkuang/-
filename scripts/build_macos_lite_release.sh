#!/usr/bin/env bash
# macOS 裁剪版发布构建入口：在当前 macOS 架构上生成 .app，不支持交叉构建。
# macOS app 必须使用 onedir，避免 onefile 与 app 沙盒及签名机制冲突。

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${PYTHON:-python3}"
app_name="${APP_NAME:-编队仿真-裁剪版}"

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
runtime_hook="$project_root/scripts/pyinstaller_hooks/set_lite_feature_profile.py"
export PYINSTALLER_CONFIG_DIR="$project_root/build/pyinstaller-cache-macos"

"$python_bin" -m pip install --upgrade pip
"$python_bin" -m pip install -r requirements-gui.txt
rm -rf "$release_app_dir"

"$python_bin" -m PyInstaller \
    --noconfirm \
    --clean \
    --windowed \
    --onedir \
    --name "$app_name" \
    --paths "$project_root" \
    --workpath "$project_root/build/lite-release-macos" \
    --runtime-hook "$runtime_hook" \
    --hidden-import src.ui.gui.features.disabled.control_monitor \
    --hidden-import src.ui.gui.features.disabled.data_analysis \
    --hidden-import src.ui.gui.features.disabled.situation3d \
    --exclude-module src.ui.gui.features.full.control_monitor \
    --exclude-module src.ui.gui.features.full.data_analysis \
    --exclude-module src.ui.gui.features.full.situation3d \
    --exclude-module src.ui.gui.live_monitor \
    --exclude-module src.ui.gui.offline_plot \
    --exclude-module src.ui.gui.data_analysis_window \
    --exclude-module src.data.control_effect_analysis \
    --exclude-module src.ui.gui.situation3d \
    --exclude-module PySide6.QtCharts \
    --exclude-module PySide6.QtQml \
    --exclude-module PySide6.QtQuick \
    --exclude-module PySide6.QtQuick3D \
    src/ui/gui/main_window.py

echo "裁剪版 macOS app：$release_app_dir"
