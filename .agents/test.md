# 最终收口测试

本文件只定义"最终收口测什么"；具体执行过程、失败分层语义、输出格式以对应 Skill / 脚本为准。
只要发生行为相关代码改动，收口不得因"风险低、改动小"而跳过；免测规则见 `.agents/workflow.md`。

## 0. 一键入口（首选）

```bash
python -X utf8 scripts/final_check.py
```

脚本自动收集改动文件（工作区未提交 + 相对 `origin/main` 已提交），按下文各节的触发条件裁剪执行：命中才真跑，未命中输出 `SKIP(no matching changes)` 并视为该门禁已完成。末行 `final_check=PASS` 且退出码 `0` 即收口通过。

- 需要强制全量：`python -X utf8 scripts/final_check.py --all`。
- 某个门禁 FAIL 时，按下文对应小节 / Skill 的失败处置规则修复后重跑。
- ST 失败涉及三层语义与刷基线判断，必须回到 `.claude/skills/st-check/SKILL.md` 处置，不得自行刷基线。
- 第 1~6 节保留单项命令，供定位单个门禁失败时使用。

另有机制性兜底：`.claude/settings.json` 的 PostToolUse hook 会在每次 Edit/Write 落盘 `.py` 后自动跑 `py_compile` + `ruff` 单文件检查（`.claude/hooks/check_py_file.py`），失败会立即反馈。它只覆盖单文件即时检查，不替代本文件的最终收口。

## 1. 基础检查（改了对应文件就必须跑）

| 触发条件 | 命令 | 通过标准 |
|----------|------|----------|
| 修改任意 Python 代码 | `python -m compileall -q src` | 退出码 0 |
| 修改 `docs/demo.html` | `node scripts/check_demo_html.js` | 输出 `demo_html=PASS` |
| 提交或交付前 | `git diff --check` | 无输出 |

## 2. 静态门禁（与 CI `test-gate.yml` 口径一致）

修改 `src/`、`tests/`、`scripts/` 下 Python 代码后运行：

```bash
python -m ruff check src tests scripts
python -m mypy
```

两者都必须零错误；禁止为过门禁而扩大 `pyproject.toml` 中的豁免清单，收紧豁免需用户确认。

## 3. LLT

修改 `src/` 下 Python 代码后运行完整 LLT：

```bash
pytest tests/llt -q
```

CI 额外统计覆盖率（`--cov --cov-fail-under=90`），本地默认不必带覆盖率参数。

## 4. 注释覆盖率

修改 `src/` 下 Python 代码后，按 Skill 执行：

- `.claude/skills/comment-coverage/SKILL.md`

## 5. 自动化 ST

修改 `src/` 下仿真、算法或 runner 代码后，按 Skill 执行（三层失败语义、刷基线流程、禁改阈值等规则以该 Skill 为准）：

- `.claude/skills/st-check/SKILL.md`

## 6. PySide6 GUI 离屏冒烟

修改 PySide6 窗体、布局或主题后运行：

```bash
python -X utf8 scripts/check_gui_offscreen.py
```

输出 `gui_offscreen=PASS` 即通过；失败时按脚本输出的 `[FAIL]` 条目排查。

## 7. GUI 交互与截图检查（人工/半自动）

离屏冒烟发现不了视觉问题。凡修改 PySide6 布局、样式、表格、滑条、图例、时间轴或主题，还应：

**交互行为检查**（手工或脚本至少覆盖）：

- 开始、暂停、单步、重置按钮能更新运行状态。
- 倍率滑条有明确 handle，拖动后倍率文本变化。
- 浅色 / 深色模式能切换，画布颜色同步变化。
- 风场脉冲、节点故障、链路丢包、清除扰动能更新回报和状态表。
- 俯视图支持滚轮缩放、拖动平移、重置视图；自动居中只改平移不改缩放。
- 侧视图与俯视图横向视野同步。
- 节点表和链路表不出现横向滚动条。
- 顶部工具栏、右侧表格、底部时间轴没有明显挤压或错位。

**截图检查重点**：

- 滑条像可拖动控件，而不是一整块色条。
- 表格列宽足够，表头和数据不错位；无多余行号或横向滚动条。
- 图例颜色与画布元素一致。
- 全屏和退出全屏后，俯视图、侧视图、时间轴仍在一屏内。

## 8. 收口后清理

```bash
find src tests -type d -name '__pycache__' -prune -exec rm -rf {} +
```
