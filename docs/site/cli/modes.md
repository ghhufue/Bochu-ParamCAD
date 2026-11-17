# 模式与命令能力

## 能力表

| 命令 | 在线 | 离线 | 默认行为 |
| --- | --- | --- | --- |
| studio | — | ✓ | 显式启动 GUI |
| sessions | — | ✓ | 列出响应中的会话 |
| status / cancel | ✓ | — | 查询 / 请求取消当前工作 |
| cloud open | ✓ | — | 在工作区载入 PLY |
| cloud info | ✓ | ✓ | 有会话时查询当前点云，否则要求 --in |
| project open / project save | ✓ | — | 打开 / 保存当前工程 |
| export step / export analysis | ✓ | — | 导出当前模型 / 识别快照 |
| capture views / capture structured-light | ✓ | — | 通过 Studio 视口拍摄 STEP / 编码结构光 |
| preprocess | — | ✓ | C++ 点云预处理 |
| detect | ✓ | ✓ | 有会话时触发识别表面，否则独立检测 |
| analyze / recognize / assemble / project | — | ✓ | 独立算法分析 |
| reconstruct cad | ✓ | ✓ | 当前文档 / 输入 PLY 的 CAD 重建 |
| reconstruct cloud | ✓ | ✓ | 工作区重建并加载 / 无窗口重建 |
| inspect | — | ✓ | 导出包的只读诊断 |

`pcad capabilities` 输出机器可读清单，含 schema、version、command、online、offline 和 description。

## 全局参数

```text
--mode auto|online|offline
--online / --offline          --mode 的便捷写法，只能选一种
--session ID                  指定在线会话
--timeout SECONDS             在线等待期限，默认 300 秒
--no-wait                     在线仅等待接收，不等待计算完成
--report FILE                 写入最终 JSON，而非标准输出
--config-dir DIR              离线配置目录
```

`--no-wait` 返回 accepted 只代表请求已接收；用 `pcad status` 的 last_command 核对最终结果。超时或 CLI 断开不会自动取消正在运行的 Studio 工作，使用 `pcad cancel` 请求取消。忙碌时新的修改命令被拒绝，状态、点云信息和取消仍可用。

在线会话通过当前用户可访问的本地 socket 通信，注册信息位于用户目录 `.paramcad/cli-sessions`。发现时会 ping；Windows 上还用进程创建时间和程序身份确认暂时无响应的 Studio 仍然存活，避免忙碌时误转离线。sessions 的 responsive 标记是否可立即响应。旧进程残留文件和复用 PID 不视为原会话。`PCAD_SESSION_DIR` 可用于隔离测试或不同工作环境。旧版 Studio 没有该接口，更新后需重新启动才可接收命令。

## 两种模式的数据语义

在线 CAD 重建使用当前工作点云、删除标记、模板、人工关系和 UI 设置；离线使用文件快照和配置，没有隐式继承 GUI 中未保存的编辑。在线 detect 运行完整“识别表面”流程，包含客户端最终集合发布和关系更新；离线 detect 输出核心基元证据。

在线不接受离线的 --unit、--synthetic、--set 等参数来偷偷改变当前文档或配置。需要改设置时按[超参数配置](/development/configuration)操作，或明确执行离线任务。
