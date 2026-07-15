---
name: comment-coverage
description: 统计本项目 src 目录的注释覆盖率并按验收标准判定成败。当 AI 修改了 src 下 Python 代码需要收口，或用户要检查/验收注释覆盖率、docstring 覆盖率、行内注释比例，或问"注释够不够""能不能过注释门禁"时使用。
---

# 注释覆盖率验收

调用 `scripts/comment_coverage.py` 统计 `src/` 的 docstring 与行内注释覆盖率，并据**验收标准**判定通过与否。

## 触发边界

- **必须执行**：AI 修改了 `src/` 下任何 Python 代码后的最终收口（见 `.agents/test.md`）；或用户明确要求检查注释覆盖率。
- **可跳过**：本次改动不涉及 `src/` 下 Python 代码时可跳过，但最终回复必须说明"未运行 comment-coverage，因为未修改 src Python 代码"。
- **禁止**：只做统计与判定，不要自动修改源码补注释，除非用户明确要求补齐。

## 验收标准（全部满足才算成功）

| 指标 | 要求 |
|------|------|
| 模块 module docstring 覆盖率 | = 100% |
| 类 class docstring 覆盖率 | = 100% |
| 函数 function docstring 覆盖率 | = 100% |
| 行内注释比例（注释行 / 代码行） | > 15% |

## 执行步骤

1. 在项目根目录运行（`-X utf8` 避免 Windows 控制台中文乱码）：

   ```bash
   python -X utf8 scripts/comment_coverage.py --fail-under-module 100 --fail-under-class 100 --fail-under-func 100 --fail-under-inline 15 --worst 12
   ```

2. 看**退出码**判定结果：
   - 退出码 `0` → 验收通过，四项指标全部达标。
   - 退出码 `1` → 验收未通过，stderr 逐条打印 `[FAIL] ...`，指出哪个指标差多少。
   - 退出码 `2` → 参数错误或 `src/` 不存在。

## 失败处理

- 退出码 `1`：先补齐必要的模块 / 类 / 函数 docstring 或行内注释（优先 `--worst` 列出的覆盖最低文件），再重新运行检查，直到退出码为 `0` 才能交付。
- 退出码 `2`：属环境或参数问题，先报告错误信息，不要盲目重试或改脚本。

## 输出契约

- 过程输出：不要复述脚本的整份报告；仅在未通过时引用 `[FAIL]` 行和最需要补注释的文件。
- 最终汇报固定包含：
  - 结论：`comment_coverage=PASS` 或 `comment_coverage=FAIL`（或跳过及原因）。
  - 四项分层覆盖率数字（模块 / 类 / 函数 / 行内）。
  - 未通过时给出优先补哪些文件的建议。

## 说明

- 仅看报告不卡门禁时，去掉 `--fail-under-*` 参数即可：`python -X utf8 scripts/comment_coverage.py --worst 12`。
- 脚本统计口径与更多参数见 `scripts/comment_coverage.py` 顶部文档字符串。
