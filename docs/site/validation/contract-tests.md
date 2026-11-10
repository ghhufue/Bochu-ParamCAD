# 阶段契约与回归测试

阶段契约把点云到 CAD 的链路切成可独立验收的 34 个模块、6 组。当前清单为 `data/reconstruction-contracts/v1/manifest.json`，`schema` 为 `paramcad.reconstruction-contracts`，版本为 1；共有 **81 个用例、74 个固定夹具、9 个数据集**。每次运行只准备一份 PLY，通过源点 ID 把点、证据和参考产物串起来。

## 阶段编号与用例清单

| 组 | 模块 | 验证重点 |
| --- | --- | --- |
| A 点云输入与预处理 | 01–04 | 单位换算、活动点与源点 ID、体素降采样、离群过滤、法向估计 |
| B 曲面分析与修复 | 05–11 | 基元检测、证据汇集、有限面片、关系、优化、局部恢复、修复复验 |
| C 常规实体识别与引导 | 12–17 | 箱体、贯穿孔、独立曲面体、组合体、混合特征、模板引导 |
| D 装配与闭合 | 18–21 | 面片提取、连接分组与全量回填、有限边界、共边与壳闭合 |
| E 投影拉伸拟合 | 22–27 | 候选轴、区间与截面、二维轮廓、提案、评分、操作序列搜索 |
| F 建模、持久化与编排 | 28–34 | 建模计划、草图、B-Rep 重建、原子提交、PCAD、STEP、生产任务 |

依赖关系由 manifest 的 `stages` 固定。例如 05 从 04 取工作点，12 从 07 取面片，13 从 12 取宿主，14 从 07 识别曲面体，15 从 04 识别组合体，16 从 08 识别混合特征，17 从 04 取工作点和引导种子；28–33 依次读取 27 选中的投影操作，34 启动生产异步任务。`bridges` 只声明了 21 从 19 的 `backfill` 阶段取得全量归属这一条桥。

代表性用例：

| 用例 | 断言要点 |
| --- | --- |
| `m01.surface_box_m.units` | 1536 点、`scale_to_mm=1000`、跨度 6 / 8 mm |
| `m13.blind_hole.reject_short_wall` | 宿主箱体识别为真，`hole_count=0`，`wall_count≥1` |
| `m13.mixed_plate.through_hole` | 1 个通孔，半径 6.975–7.025 mm，壁厚 4.98–5.02 mm |
| `m18.two_boxes.extract` → `m21.two_boxes.closure` | 12 面片、2 分组、12 环、2 闭合体，体积 375 ± 1e-5 mm³ |
| `m22.u_channel.axes` → `m27.u_channel.search` | 候选轴、6 mm 区间、面积 88–96 mm²、体积 528–576 mm³、接受 1 次操作 |
| `m28.blind_hole.plan` → `m33.blind_hole.step_roundtrip` | 4 条命令、2 个草图面、1 个实体、材料 IoU ≥0.99999 |
| `m34.surface_box.production` | 生产分支、取消投递、提交修订 1、1 个特征、体积 239.95–240.05 mm³ |

## 固定夹具与点云串联

`--mode fixture` 直接读取参考上游 JSON，只执行目标阶段，不重新运行上游算法。夹具条目同时绑定数据集和期望模块号；回填用例的期望上游固定为模块 20。参考上游写入 `datasets/<dataset>/<fixture>.json`，目标阶段的产物写入 `fixture/<dataset>/<case-id>/artifact.json`，适合验证单个阶段的输入输出契约。

`--mode cloud` 按 manifest 的依赖递归运行当前实现，并在同一次运行中缓存、共享上游结果。下游只读取上游的 `artifact.json`；上游未通过验收时，下游状态记录为 `blocked_upstream`，同时带 `blocked_by_module`、`blocked_by_phase` 和原始错误。不会用旧产物顶替失败阶段。

装配串联顺序固定为 **18 面片 → 19 分组 → 20 边界 → 19 全量回填 → 21 闭合**。回填在报告中作为独立用例执行，目录标签为 `19-backfill`；模块 21 通过桥读取 `cloud/<dataset>/19-backfill/artifact.json`。同一数据集在同一阶段配置多个断言用例时，清单用 `upstream_gate` 指定唯一供下游判断的用例。

`--cloud` 用于外部产物引用不同 PLY 的场景，必须只选择一个数据集，并同时提供 `--upstream` 或处于 `cloud` 模式。驱动记录点云、上游与清单的哈希；对按清单生成的点云，哈希与清单不符时拒绝运行，`--upstream` 的哈希写入结果行。`--upstream` 只允许用于单个 `fixture` 用例。投影串联按配置选一条候选轴传递，整件的多轴竞争仍由 `fit-projection` 回归覆盖。

## 中间产物重放

```powershell
cmake --build "apps/ParamCAD Studio/build/msvc-debug" --config Release --target cad_reconstruction_validation
python -m tools.contracts.run_reconstruction_contracts --output outputs/contracts-all
python -m tools.contracts.run_reconstruction_contracts --mode cloud --case m21.two_boxes.closure --output outputs/contracts-closure
python -m tools.contracts.run_reconstruction_contracts --mode fixture --case m24.u_channel.profile --output outputs/contracts-profile
python -m tools.contracts.run_reconstruction_contracts --mode fixture --case m20.two_boxes.boundary --upstream outputs/contracts-all/cloud/two_boxes/19/artifact.json --output outputs/contracts-boundary-replay
python -m tools.contracts.run_reconstruction_contracts --mode fixture --case m33.blind_hole.step_roundtrip --upstream outputs/contracts-all/cloud/blind_hole/32/artifact.json --output outputs/contracts-step-replay
```

重放时输出目录必须不存在。`fixture` 模式不会重新计算上游，因此当前实现变化而参考产物未变时，失败会集中在目标阶段。`cloud` 模式会重新运行依赖链，适合确认错误来自上游状态还是目标阶段本身。模块 33 的报告目录同时保留 `roundtrip.pcad` 与 `roundtrip.step`，可直接检查保存重开和 STEP 回读。

## 失败定位

`summary.json` 是运行级索引，记录 `manifest_sha256`、`executable_sha256`、`fixture_generator_sha256`、`results[]` 与 `executions[]`。每个结果行包含用例、数据集、模块、模式、状态、点云哈希、上游哈希（有上游时）、`artifact_sha256`、指标、错误和耗时。

| 字段或状态 | 含义 |
| --- | --- |
| `passed` | 引擎返回 `contract_stage_completed`，且所有断言通过 |
| `failed` | 指标不符、点 ID 或分区不符、引擎非零、超时或报告缺失 |
| `blocked_upstream` | 上游未通过，本阶段未执行；`blocked_by_module` / `blocked_by_phase` 指向阻断源 |
| `metrics` | 断言标量、区间或布尔值的实测结果 |
| `point_ids` | 必需或排除的源点／工作点 ID |
| `partitions` | 面片、部件或回填分区的点归属是否与参考一致 |

定位顺序建议为：先看 `summary.json` 中首个非 `passed` 行；再读同目录 `report.json` 的 `error` 与 `metrics`；随后对照 `request.json` 确认模块、单位、选项和上游输入；只有需要引擎细节时才查看 `engine.log`。任何非通过结果都会使驱动返回非零退出码，适合直接接入回归脚本。
