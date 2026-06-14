# AGENTS.md

## 项目约定

- 正式 GUI 技术栈是 PySide6。
- `docs/demo.html` 只作为布局、交互和视觉风格 demo，不作为正式运行时技术选型。
- 修改 PySide6 GUI 后，需要同步 `编队仿真.app/Contents/Resources/appsrc/src`，否则双击 app 仍会运行旧快照。
- 不要删除或回退用户已有改动；提交前先检查 `git status --short`。

## 自测试要求

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

## app 包同步

如果修改了 `src/` 下正式 GUI 代码，需要同步 app 内代码快照：

```bash
rm -rf '编队仿真.app/Contents/Resources/appsrc/src'
cp -R src '编队仿真.app/Contents/Resources/appsrc/src'
find '编队仿真.app/Contents/Resources/appsrc/src' -type d -name '__pycache__' -prune -exec rm -rf {} +
```

同步后检查 app 内快照：

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

## 清理生成物

测试后清理 Python 缓存：

```bash
find src '编队仿真.app/Contents/Resources/appsrc/src' -type d -name '__pycache__' -prune -exec rm -rf {} +
```
