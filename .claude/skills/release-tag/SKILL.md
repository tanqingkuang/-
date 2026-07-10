---
name: release-tag
description: 按本项目发布约定推动版本 tag 和 GitHub Release 流程。当用户要求“打 Vx.y.z tag”“打 Vx.y.z tag 并 release”“发布 Vx.y.z”或类似正式发版动作时使用；本地只负责编排、校验和推送 tag，实际 Windows 打包与 Release 上传交给 GitHub Actions。
---

# 版本 Tag 与 Release 编排

本 skill 负责把正式发布完整推进到结束：同步主干、补齐必要的 GitHub Actions 发布流程、校验版本、创建并推送 tag、等待构建并发布 Release。不要在本地临时打包正式交付物，除非用户明确要求本地诊断。

## 基本原则

- tag 是源码快照，Release 是该 tag 对应的交付物；两者流程绑定，但语义独立。
- 正式发布必须从最新 `origin/main` 产生，不从功能分支、PR 分支或本地未同步提交发版。
- 同名 tag 一旦推到远端，不要删除重打；若已经对外发布，修复后使用下一个版本号。
- GitHub Actions 负责 Windows x64 构建、压缩包组织和 Release 资产上传；本地只做流程守门。
- 若仓库没有 tag 触发的 release workflow，应先补齐 workflow，并通过仓库要求的 PR / 检查合入主干，再继续发版；不能只报告缺失后停止，也不能改为本地发包。
- Release 说明必须使用中文、内容精简，不使用 GitHub 自动生成的英文变更列表、MR/PR 链接或比较链接。
- Release 说明固定包含“版本说明”“主要更新”“下载说明”三个部分；主要更新概括 2 至 4 条，下载说明只描述资产用途。

## 执行步骤

1. 解析版本号，要求形如 `V0.3.1`。如果用户没有给出明确版本号，先询问。
2. 检查工具与认证：
   - `gh auth status`
   - `gh repo view --json nameWithOwner,defaultBranchRef`
3. 检查工作区：
   - `git status --short`
   - 不删除或回退用户已有改动；若改动与发布无关，保留并在最终汇报中说明。
   - 若改动会改变发布源码或阻止切换到主干，暂停并说明具体冲突。
4. 切到主干并同步：
   - `git switch main`
   - `git fetch origin --prune`
   - `git pull --ff-only`
   - 若本地 `main` 与 `origin/main` 分叉，禁止 reset，先向用户说明本地 main 需要人工整理。
5. 校验 tag / release 不存在：
   - `git ls-remote --tags origin <tag>`
   - `gh release view <tag>`
   - 任一已存在都停止，避免覆盖正式发布。
6. 校验并准备 release workflow：
   - 查找 `.github/workflows/` 中是否有 tag 触发并创建 GitHub Release 的 workflow。
   - 若没有，按照现有构建脚本和项目交付约定新增 workflow：Windows 构建使用仓库已有脚本，资产整理为客户可直接使用的压缩包，Release job 使用 `gh release create` 或等效官方能力上传资产。
   - workflow 的 Release 步骤必须使用中文精简说明文件，不得使用 `--generate-notes`，不得自动附加 MR/PR 或比较链接。
   - 若仓库要求 PR 合入，创建分支和 PR，等待必需检查通过后合入；合入后重新同步 `origin/main`，再继续后续步骤。
7. 版本文件处理：
   - 若仓库已有明确版本文件或版本常量，先更新并走 PR 合入主干后再继续发 tag。
   - 若没有版本文件，本项目可以只用 tag 作为版本来源。
8. 创建 annotated tag 并推送：
   - `git tag -a <tag> -m "<tag>"`
   - `git push origin <tag>`
9. 等待并核验 GitHub Actions：
   - 找到该 tag 触发的 release workflow run。
   - 等待构建、打包、Release 上传全部成功。
   - 若失败，报告失败 job、日志摘要和 release/tag 当前状态，不要自动删除 tag。
10. 核验 Release：
    - `gh release view <tag> --json url,tagName,isDraft,isPrerelease,assets`
    - 确认资产包含完整客户交付包；如果是 Windows 包，应包含 exe、配置、外置 3D 模型等约定内容。
    - 确认 Release body 全部为中文，内容精简，且不包含 MR/PR 链接、英文变更列表或比较链接。

## 失败处置

- tag 未推送前失败：修复前置条件后可重跑。
- 缺少 workflow：先补 workflow 并通过仓库保护规则要求的 PR / 检查，不得把它当作终止条件。
- tag 已推送但 CI 失败：保留 tag，先定位 CI；是否删除远端 tag 必须由用户明确决定。
- Release 已发布后发现问题：不要覆盖同名版本，默认建议发下一个 patch 版本。

## 汇报格式

最终回复包含：

- tag 名称与目标 commit SHA。
- GitHub Actions run URL。
- Release URL。
- 上传资产清单。
- Release 说明是否为精简中文且不含 MR/PR 链接。
- 本地是否修改了文件、是否有未提交改动。
