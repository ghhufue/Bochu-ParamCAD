# 贡献指南与文档维护

仓库的文档与发布流程以 `main` 为目标分支，代码、测试和文档在同一工作树内维护。提交前请确认改动范围、构建开关和页面注册都已同步；文档站点只构建 `docs/site`，不会读取其他目录的 Markdown。

## 分支与 PR

- 功能、修复和文档改动都从 `main` 拉出短分支，通过 Pull Request 合并回 `main`。
- 文档工作流 `.github/workflows/docs.yml` 在 PR 和推送到 `main` 时都会运行，但只有 `push` 到 `main` 才执行部署任务。
- PR 阶段只执行 `npm ci` 和 `npm run build`，用于发现链接、语法和构建错误；不会写入 GitHub Pages。
- 推送到 `main` 后，构建产物 `docs/.vitepress/dist` 由 `upload-pages-artifact` 上传，再由 `deploy` 任务发布到 `github-pages` 环境。
- 工作流使用 Node.js 24 和 `docs/package-lock.json` 缓存依赖；本地也应使用 `npm ci` 或保持锁文件同步。
- 并发组按 `github.ref` 区分，且 `cancel-in-progress: false`，同一分支的文档构建不会互相取消。
- 工作流支持 `workflow_dispatch` 手动触发，用于重新构建或补发站点。
- 代码与对应文档尽量放在同一个 PR：评审者需要同时看到行为变化和页面描述的变化。
- PR 描述中说明改了什么、为什么改、如何验证；涉及行为变化时附上对应测试或验证命令。

## 代码、测试与文档同步

改动算法阈值、默认值或命令行参数时，至少同步以下位置：

| 改动类型 | 需要同步 |
| --- | --- |
| 新增或重命名配置键 | `apps/ParamCAD Studio/config/*.conf`、加载与校验代码和配置页 |
| 改变识别、装配或投影行为 | 对应核心源码、回归测试、模块 README 和 `docs/site/modules` 页面 |
| 改变 CLI 参数或报告字段 | `apps/ParamCAD Studio/src/cli`、工作区命令桥、`docs/site/cli`、CLI 测试及能力清单 |
| 改变工程格式或修订语义 | 持久化代码、兼容性测试、架构页和设计决策页 |
| 新增或删除公开能力 | 根 `README.md` 进度表、路线图页面和相关模块页 |

模块页允许“测试与诊断”小节暂时留空，但不能省略或写错源码位置。页面中的路径、类型名和命令必须能在当前工作树中定位；无法确认的数字或阈值宁可删除，也不要补全推测。

文档统一使用本项目术语：观测快照、源点 ID、曲面证据、有限面片、关系、候选、建模补丁、特征历史、修订、原子提交、B-Rep。站内引用写绝对路径，例如 `/development/configuration`；文件路径使用行内代码。

## 本地预览与 GitHub Pages

站点源目录是 `docs/site`，配置在 `docs/.vitepress/config.mts`，`base` 为 `/Bochu-ParamCAD/`。本地命令均在 `docs` 目录执行：

```powershell
cd docs
npm install
npm run dev
npm run build
npm run preview
```

`npm run dev` 启动本地开发服务器，`npm run build` 生成静态站点，`npm run preview` 预览构建结果。新增页面必须在 `docs/.vitepress/config.mts` 的 `sections` 中注册，否则不会出现在导航和侧边栏；修改导航结构时保持现有分组顺序。

提交前注意：

- 不要提交 `docs/node_modules` 或 `docs/.vitepress/dist`，两者已被根 `.gitignore` 忽略。
- 工作流只在 `docs/.vitepress/**`、`docs/site/**`、`docs/package*.json` 和 `.github/workflows/docs.yml` 变化时触发；只改其他目录不会更新站点。
- 部署使用 GitHub Pages 官方 action，`permissions` 需要 `pages: write` 与 `id-token: write`，由工作流文件维护，不要在页面中另行配置。
- 预览时优先检查站内绝对路径、代码块语言标记和表格列数，这些错误在构建阶段最容易暴露。
