# AI 工作流与诊断

## 在线重建当前工作区

```powershell
pcad sessions
pcad status
pcad cloud info --limit 10
pcad reconstruct cad --online
pcad project save --out outputs/result.pcad
pcad export step --out outputs/result.step
pcad export analysis --out outputs/result.pcad-analysis
```

只有在需要新开 Studio 时执行 `pcad studio`。重建前确认当前点云和 busy 状态；未知单位或手动修改可能弹出与按钮操作相同的确认。CLI 完成回应表示结果已通过原有控制器应用或明确拒绝。后续自动表面分析也可能短暂占用工作区，用 status 确认空闲后再保存/导出。

## 完全离线

```powershell
pcad cloud info --offline --in input/cloud.ply
pcad detect --offline --in input/cloud.ply --unit mm --report outputs/detect.json
pcad reconstruct cad --offline --in input/cloud.ply --unit mm --output outputs/cad-run-new
```

即使 Studio 已启动，--offline 也不连接它、不修改当前文档。真实单位是 m 时传 --unit m；已知单位通过观测快照转换为毫米。不能为了让命令通过而把 arbitrary 宣称为 mm。

## 识别结果排查

```powershell
pcad inspect --bundle outputs/result.pcad-analysis manifest
pcad inspect --bundle outputs/result.pcad-analysis list --limit 10
pcad inspect --bundle outputs/result.pcad-analysis show --id 118 --group
pcad inspect --bundle outputs/result.pcad-analysis analyze --id 118
```

先确认 bundle_id、源指纹和单位，再按集合查询。源点索引在分析包中是 0-based；核心及许多验证报告使用 PLY 行号 + 1，不要混用。下采样、预处理输出与不同快照之间先核对身份。

confidence、点数覆盖率和显示网格面积是不同指标。analyze 只复算平面残差，不代表已经对所有曲面执行完整几何有效性证明。

## 失败定位

| 情况 | 处理 |
| --- | --- |
| 找不到 pcad.exe | 构建 Studio，并核对 artifacts/cli-runtime.json 或 PCAD_EXECUTABLE |
| 在线没有会话 | 运行 pcad studio；旧客户端需重新启动 |
| 多个会话 | 使用 pcad sessions 返回的 --session ID |
| Studio 忙碌 | 读 status，等待或 cancel，不另跑一份离线任务 |
| 在线超时/断开 | 任务可能仍运行；核对 status.last_command 与界面 |
| 未知配置键 | 确认 EXE 与配置来自同一构建；构建保留已有运行配置 |
| 无单位 | 提供真实 mm/m 元数据，未知尺度先标定 |
| CAD unsupported/partial | 阅读 report.error、coverage_basis 和分离诊断，不按候选数认定完整恢复 |
| 分析包校验失败 | 核对是否混用了不同快照文件 |

## 开发验证工具

`cad_reconstruction_validation`、阶段 contract-stage、STEP 采样/逐面验收和专项 trace 脚本继续用于回归与定位。它们不是控制当前 Studio 的在线接口。相关入口见[阶段契约](/validation/contract-tests)、[识别结果导出](/validation/analysis-export)。

修改 CLI 时同步命令能力清单、参数契约、测试和本章节。在线流程必须接到真实控制器，离线测试应覆盖无窗口执行；不要新增与 GUI 重复的第二套结果提交逻辑。
