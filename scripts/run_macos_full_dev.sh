#!/usr/bin/env bash
# 开发调试入口（全量档）：直接运行源码，不打包；改代码后重新执行即可。

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${PYTHON:-python3}"
install_dependencies=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --python)
            python_bin="$2"
            shift 2
            ;;
        --install-dependencies)
            install_dependencies=true
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

if [[ "$install_dependencies" == true ]]; then
    "$python_bin" -m pip install "$project_root"
fi

cd "$project_root"
export SIMU_GUI_FEATURE_PROFILE="full"
exec "$python_bin" src/ui/gui/main_window.py "$@"
