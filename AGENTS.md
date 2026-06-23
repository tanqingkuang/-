# AGENTS.md

## 项目约定

- **本项目所有内容统一使用中文**：代码注释与 docstring、与用户的沟通回复、提交信息、review 与记录文档等，一律用中文书写。
- 正式 GUI 技术栈是 PySide6。
- `docs/demo.html` 只作为布局、交互和视觉风格 demo，不作为正式运行时技术选型。
- `编队仿真.app/` 是本地构建产物，不纳入版本控制；需要双击调试时再由本地打包 / 同步流程生成。
- 不要删除或回退用户已有改动；提交前先检查 `git status --short`。

## 自测试要求

### 检视问题处理

处理代码检视、review 文档或缺陷反馈时，默认采用 TDD 手段：

1. 先为每个可执行的检视问题补充能失败的测试，证明问题存在或锁定预期行为。
2. 再修改实现，使新增测试和既有测试通过。
3. 若某条意见不适合用测试覆盖，必须在对应 review 文档或回复中说明原因。
4. 修复后在 review 文档中逐条回复“已修改”或“不修改原因”，并列出对应测试用例。

### 基础检查

每次修改 Python 代码后运行：

```bash
python -m compileall -q src
```

每次修改 `docs/demo.html` 后运行：

```bash
node - <<'NODE'
const fs = require('fs');
const html = fs.readFileSync('docs/demo.html', 'utf8');
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map((m) => m[1]);
for (const script of scripts) new Function(script);
console.log(`checked ${scripts.length} inline script(s)`);
NODE
```

提交或交付前运行：

```bash
git diff --check
```

### 注释覆盖率检查

每次 AI 修改 `src/` 下 Python 代码后，必须执行 `.claude/skills/comment-coverage` 对应检查：

```bash
python -X utf8 scripts/comment_coverage.py \
  --fail-under-module 100 \
  --fail-under-class 100 \
  --fail-under-func 100 \
  --fail-under-inline 15 \
  --worst 12
```

要求：

- 退出码为 `0` 才能交付。
- 若检查失败，必须先补充必要的模块、类、函数 docstring 或行内注释，再重新运行检查。
- 若本次修改不涉及 `src/` 下 Python 代码，可以跳过，但最终回复中必须说明“未运行 comment-coverage，因为未修改 src Python 代码”。
- 最终交付回复必须列出该检查的通过 / 跳过 / 失败状态。

### PySide6 GUI 自测试

PySide6 窗体至少要做 offscreen 构造测试：

```bash
QT_QPA_PLATFORM=offscreen python - <<'PY'
from PySide6.QtWidgets import QApplication
from src.ui.gui.main_window import MainWindow

app = QApplication([])
window = MainWindow()
window.show()
app.processEvents()

print(window.windowTitle())
print("node hscroll", window.node_table.horizontalScrollBar().maximum())
print("link hscroll", window.link_table.horizontalScrollBar().maximum())
print("progress", type(window.progress).__name__, window.progress.value())

window.close()
app.quit()
PY
```

期望：

- 窗口能构造，不抛异常。
- 节点表横向滚动条最大值为 `0`。
- 链路表横向滚动条最大值为 `0`。
- 底部进度条类型为 `QProgressBar`。

### GUI 交互检查

修改正式 GUI 后，至少手工或脚本检查以下行为：

- 开始、暂停、单步、重置按钮能更新运行状态。
- 倍率滑条有明确 handle，拖动后倍率文本变化。
- 浅色模式和深色模式能切换，且画布颜色同步变化。
- 风场脉冲、节点故障、链路丢包、清除扰动能更新回报和状态表。
- 俯视图支持滚轮缩放、拖动平移、重置视图。
- 自动居中只改变平移，不改变缩放。
- 侧视图与俯视图横向视野同步。
- 节点表和链路表不出现横向滚动条。
- 顶部工具栏、右侧表格、底部时间轴没有明显挤压或错位。

### 截图检查

offscreen 构造测试不能发现所有视觉问题。凡是修改 PySide6 布局、样式、表格、滑条、图例、时间轴或主题，都应该生成或人工查看一张真实窗口截图。

建议检查重点：

- 滑条是否像可拖动控件，而不是一整块色条。
- 表格列宽是否足够，表头和数据是否错位。
- 是否出现多余行号或横向滚动条。
- 图例颜色是否与画布元素一致。
- 全屏和退出全屏后，俯视图、侧视图、时间轴是否仍在一屏内。

## app 包本地调试

`编队仿真.app/` 不提交到 git。若本地需要双击 app 调试，可以重新生成或临时同步 app 内代码快照：

```bash
rm -rf '编队仿真.app/Contents/Resources/appsrc/src'
cp -R src '编队仿真.app/Contents/Resources/appsrc/src'
find '编队仿真.app/Contents/Resources/appsrc/src' -type d -name '__pycache__' -prune -exec rm -rf {} +
```

本地同步后可检查 app 内快照：

```bash
QT_QPA_PLATFORM=offscreen /Users/zhangmeng/miniforge3/bin/python - <<'PY'
import sys
sys.path.insert(0, '编队仿真.app/Contents/Resources/appsrc')

from PySide6.QtWidgets import QApplication
from src.ui.gui.main_window import MainWindow

app = QApplication([])
window = MainWindow()
window.show()
app.processEvents()

print("node hscroll", window.node_table.horizontalScrollBar().maximum())
print("link hscroll", window.link_table.horizontalScrollBar().maximum())
print(type(window.progress).__name__)

window.close()
app.quit()
PY
```

## Windows exe 打包

Windows x64 exe 需要在 Windows x64 环境构建，不要在 macOS 上直接用本机 PyInstaller 伪造产物。原因是 PySide6/Qt 需要目标平台的 Python wheel、Qt DLL 和 platform plugin。

本地 Windows 机器开发期快速打包：

```powershell
.\scripts\build_windows_exe_dev.ps1
```

首次打包或依赖变化后可加 `-InstallDependencies`。

开发期产物入口为 `dist\编队仿真.exe`，运行依赖在同级 `dist\编队仿真\` 目录。

发布期完整打包：

```powershell
.\scripts\build_windows_exe_release.ps1
```

GitHub Actions 打包：

- workflow 文件：`.github/workflows/build-windows-exe.yml`
- 触发方式：推送 `main` 相关文件，或手动 `workflow_dispatch`
- 产物：`formation-sim-windows-x64` artifact，内含 `编队仿真.exe`

## 清理生成物

测试后清理 Python 缓存：

```bash
find src tests '编队仿真.app/Contents/Resources/appsrc/src' -type d -name '__pycache__' -prune -exec rm -rf {} +
```
