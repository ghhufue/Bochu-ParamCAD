# CLI 使用指南

所有正式命令以 `pcad` 开头，例如 `pcad studio`、`pcad preprocess`、`pcad reconstruct cad`。模块不再以独立的 `pcad preprocess.exe` 等作为用户入口。

## 安装与发现

开发环境先构建 Studio，`CAD_CLIENT_BUILD_MODULE_CLIS=ON` 时会同时生成匹配的 `pcad.exe`，复制模块配置，并更新 `artifacts/cli-runtime.json`。仓库中的 `tools/pcad.cmd` / `python tools/pcad.py` 读取该记录，不猜旧构建路径。`PCAD_EXECUTABLE` 可显式覆盖。

```powershell
cmake --build artifacts/build-structured-light --config Release --target CadPointCloudClient
python tools/pcad.py --help
python tools/pcad.py capabilities
python tools/pcad.py studio
```

将 `tools` 加入 PATH 后可直接输入 `pcad`。正式离线包使用 `app/pcad.exe`，与 Studio、DLL 和 config 保持相对位置，无需另装 Python 来运行 C++ 命令。

## 版本与构建身份

```powershell
pcad --version
```

输出 JSON 包含 `application_version`、`build_configuration`、`build_utc`、`git_commit`、`source_state` 和 `worktree_fingerprint`。产品版本统一来自 `apps/ParamCAD Studio/VERSION`；构建时 CMake 记录 Git 提交、工作区是 `clean` 还是 `dirty`，以及未提交改动的 SHA-256 指纹。同一构建目录的 Studio“帮助 → 关于”显示相同信息。

排查“代码与运行结果不一致”时，应使用目标构建目录中的 `pcad`，核对其 `application_version` 和 `git_commit`；工作区为 `dirty` 时，指纹只说明构建时的未提交状态，不能与任意远端提交直接对应。

## 在线与离线

在线：把请求发送给已经运行的 Studio，复用界面按钮对应的工作区控制器。进度、模板、关系、结果提交、集合、模型和视口沿用界面逻辑。可能出现与手动操作相同的尺度确认或覆盖确认。默认等待结果应用后返回。

离线：由控制台程序调用 C++ 后端，不创建窗口，也不会启动 Studio。图片重建由相同 C++ 重建控制器编排既有 Python 图像后端。

同时支持两种模式的命令，默认 `auto`：有一个存活的 Studio 会话则在线，否则离线。只支持一种模式的命令按其能力执行。多个 Studio 同时运行时必须指定 `--session`。选定在线后，连接失败、计算失败、超时都不会转到离线重跑。

```powershell
pcad reconstruct cad                         # 有 Studio 时操作当前点云
pcad reconstruct cad --offline --in cloud.ply --unit mm --output outputs/run-001
pcad detect --online
pcad sessions
pcad status --session SESSION_ID
```

完整说明见[模式与命令能力](/cli/modes)、[命令参考](/cli/commands)、[AI 工作流](/cli/workflows)。算法参数独立维护于[超参数配置](/development/configuration)。

## AI 调用约定

先通过 `capabilities` 确认命令支持的模式，再核对单位和当前会话。离线使用新的输出目录；在线先读 `status`，确认当前文档。按 JSON 字段判定成功，不能只看进程启动或候选数量。点坐标和长 ID 列表只在明确需要时有界读取。

`pcad inspect` 永远离线，专门查询已导出的 `.pcad-analysis`，不依赖 Studio 是否启动。`pcad studio` 是唯一显式启动 Studio 的命令。
