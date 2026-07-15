# 项目约定

- **本项目所有内容统一使用中文**：代码注释与 docstring、与用户的沟通回复、提交信息、review 与记录文档等，一律用中文书写。
- 正式 GUI 技术栈是 PySide6。
- `docs/demo.html` 只作为布局、交互和视觉风格 demo，不作为正式运行时技术选型。
- `编队仿真.app/` 是本地构建产物，不纳入版本控制；需要双击调试时再由本地打包 / 同步流程生成（见 `.agents/build-and-debug.md`）。
- 当用户要求"打 Vx.y.z tag 并 release"或类似正式发版动作时，按 `.claude/skills/release-tag/SKILL.md` 执行发布编排，实际打包与 Release 上传交给 GitHub Actions。
- 不要删除或回退用户已有改动；提交前先检查 `git status --short`。
- 新增 GUI 逻辑一律先写零 Qt 依赖的 ViewModel，再接 Qt 视图层。
