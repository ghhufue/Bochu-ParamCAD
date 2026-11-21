# 命令参考

## 会话、文档与导出

```powershell
pcad --version
pcad studio [--exe PATH] [--dry-run]
pcad sessions
pcad status
pcad cancel
pcad cloud open --in cloud.ply
pcad cloud info                         # 在线当前点云
pcad cloud info --offline --in cloud.ply --unit mm
pcad project open --in model.pcad
pcad project save --out model.pcad
pcad export step --out model.step
pcad export analysis --out cloud.ply.pcad-analysis
```

在线 cloud info 支持 `--limit 1..100 --offset N`，默认 20 条集合摘要，不输出完整点列表。离线 cloud info 读取 PLY 的点数、有限点数、范围、法向/颜色存在性及单位。export analysis 保留后台快照和不可覆盖规则。

## 六个算法模块

```powershell
pcad preprocess --in cloud.ply --unit mm --out working.ply --report preprocess.json
pcad detect --offline --in cloud.ply --unit mm --report detection.json
pcad analyze --in cloud.ply --unit mm --report analysis.json
pcad recognize --in cloud.ply --unit mm --report recognition.json
pcad assemble --in cloud.ply --unit mm --report assembly.json
pcad project --in cloud.ply --unit mm --path auto --step model.step --report projection.json
```

公共离线参数：`--in PLY` 或 `--synthetic box`、`--unit mm|m|arbitrary`、`--config FILE`、可重复的 `--set key=value`、`--print-config`。`--out` 只用于 preprocess；`--step` 和 `--path` 只用于 project。使用分开的参数值写法，如 `--in FILE`。

| 模块 | 输出 |
| --- | --- |
| preprocess | PLY、同名单位 JSON、处理统计；输出不保留任意自定义属性/原始点 ID |
| detect | 方程、证据支持点数、coverage、confidence、未归属数量 |
| analyze | 曲面摘要、面片、关系；没有完整支持点映射 |
| recognize | 候选数量、覆盖率和置信度；不导出 CAD |
| assemble | 部件状态、closed、开放边、候选统计；parts 非空不等于全部闭合 |
| project | 投影质量与操作数量，可选 STEP；不导出 PCAD |

这些模块从 PLY 重新执行各自需要的上游处理，不读取上一条模块报告。recognized / ok 不代表完成实体导出。需要 PCAD 时用 reconstruct cad。

## 视口拍摄

```powershell
pcad capture views --in model.step --output outputs/capture-new --texture texture.png
pcad capture structured-light --in model.step --output outputs/scan-new --scan-stations 8 --scan-size 640x480 --projector-size 256x256 --scan-reconstruct
```

两者只支持在线，不隐式启动 Studio。复用现有批量拍摄和结构光控制器。结构光还支持 --scan-renderer vtk/mitsuba、--scan-samples（32）、--scan-max-depth（4）、--scan-metallic（0.9）、--scan-roughness（0.4）、--scan-light-intensity（2）、--scan-baseline（0 自动）、--scan-no-phase-shift、--scan-multi-exposure；相移默认开启。更多物理与采集限制见[结构光模块](/modules/structured-light)。

## CAD 重建

```powershell
pcad reconstruct cad --online
pcad reconstruct cad --online --in cloud.ply --no-projection-fallback
pcad reconstruct cad --offline --in cloud.ply --unit mm --output outputs/cad-new
```

在线可省略 --in，以当前文档为输入。使用真实 UI 控制器，可在界面看到进度和结果，必要确认仍由客户端显示。在线参数只有 `--in`、`--no-projection-fallback`。

离线必须给 --in 和新的 --output 目录。支持 --unit mm/m、--no-projection-fallback、--no-specialized-recovery、--config、重复 --set、--path。后面三项针对投影回退配置；其他模块通过配置发现机制加载。

离线成功输出 input 快照、report.json、model.pcad、model.features.json、model.step；装配路径可能另有分离诊断。检查 cad_constructed、project_round_trip_verified、step_round_trip_verified 和实际文件。cad_exported_partial 表示部分结果。

Studio 和原生 pcad 共享本地应用缓存。报告的 `cache` 对象列出本次查询阶段及其命中、未命中或绕过情况。缓存键绑定实际观测和生效设置，包含阶段实现与数据格式版本。装配按提取分组、边界准备、闭合和投影保存，单独修改投影参数可以复用前面的结果。开放部件及其诊断也可以复用。

测试时可在启动进程前设置 `PCAD_CACHE_DIR`，把缓存放到独立目录。设置 `PCAD_DISABLE_CACHE=1` 可绕过缓存进行对照。Studio 的 **设置 → 清理点云缓存…** 可整体删除缓存；缓存的阶段、键和限制见[本地重建缓存](/modules/persistence#本地重建缓存)。打开 PLY 时只载入点云。主动执行表面识别或 CAD 重建时可复用匹配的计算缓存。保存并重开 PCAD 工程可恢复工程中的集合、关系和 CAD 模型。

## 图片重建

```powershell
pcad reconstruct cloud --images capture --output outputs/reconstructions --engine auto
pcad reconstruct cloud --offline --images photos --output outputs/reconstructions --engine colmap --dense
```

参数在两种模式下一致：--images、--output 必需；--engine auto/colmap/structured_light，--matcher exhaustive/sequential，--dense，--voxel-size-mm（默认 0.2）、--min-contrast（默认 8）、--no-camera-info。

复用 ReconstructionController，自动建立独立运行子目录。在线完成后加载点云、刷新相机和面板；离线只写数据。图片后端继续使用随包 Python；COLMAP 稠密路径还需兼容 GPU。普通 SfM 没有已知位姿尺度时单位仍为 arbitrary。

## 导出包诊断

```powershell
pcad inspect --bundle cloud.ply.pcad-analysis manifest
pcad inspect --bundle cloud.ply.pcad-analysis list --filter Plane --kind plane --limit 10 --offset 0
pcad inspect --bundle cloud.ply.pcad-analysis show --id 118 --group
pcad inspect --bundle cloud.ply.pcad-analysis analyze --name "Plane 1"
pcad inspect --bundle cloud.ply.pcad-analysis points --id 118 --limit 5
pcad inspect --bundle cloud.ply.pcad-analysis verify --full
```

只支持离线，C++ 直接读取快照。list/show 不读取二进制坐标；analyze 只读目标面。list 默认 20 条、points 默认 10 点，两者单次上限 100。名称重名必须使用 ID。verify 默认元数据，full 追加全部资产哈希。

## 输出与退出码

普通命令输出 JSON，`--report` 将结果写入文件。配置打印与 help 是文本。`pcad --version` 输出产品版本、构建配置、构建时间、Git 提交、工作区状态和改动指纹。模块报告带 schema/version/mode；在线响应另带请求 id、status、exit_code。计算日志可能写入 stderr。父目录需预先存在，CAD 输出目录必须新建。

- 0：操作成功，或在线请求已 accepted（使用 --no-wait 时）。
- 1：已执行但不支持、被取消、过期或结果未应用等业务失败。
- 2：参数、模式、会话、配置、读写或通信错误。

以报告 status/error 和命令特定字段为准。在线超时不代表计算停止，不要自动换离线重试。
