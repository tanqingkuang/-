# CLAUDE.md

所有交流、代码注释、提交信息和文档统一使用中文。

开发入口索引：

- 项目约定：`.agents/conventions.md`
- 工作流与风险分级：`.agents/workflow.md`
- 代码修改后测试（最终收口）：`.agents/test.md`
- 运行调试与打包：`.agents/build-and-debug.md`

项目共享 Skills 位于 `.claude/skills/**`（comment-coverage / st-check / release-tag），由 Claude Code 按 skill 描述自动触发或经 Skill 工具调用。

最终收口首选一键入口 `python -X utf8 scripts/final_check.py`：脚本按改动文件自动裁剪各门禁（详见 `.agents/test.md`）。收口类检查一律走脚本入口，不要展开等价的裸命令组合：脚本会隐藏中间过程，只输出结构化结论，失败时才给关键日志。
