# 实体推断实现状态与差距

> 本页对应当前产品版本 **0.3.35**（2026-11-22）。实体推断主体来自提交 `d307f00`（`Infer CAD solids from point-cloud surface scenes`），之后的工作树补充了实时预览与进度回调。
> 方案原文与目标设计见[实体推断实施方案（参考）](/development/solid-inference-plan)。本页只记录可以从当前工作树验证的实现、偏差与未完成项。

## 结论摘要

当前工作树已经把**默认自动重建**从“旧装配闭合链”切换到“SurfaceScene + 实体推断”：

- 上游曲面分析结果会导出为带指纹、有限边界和关系的 `SurfaceScene`；
- 下游实现观测闭合壳与平行端盖拉伸两类候选，支持通孔、盲孔、评分、Beam 组合搜索和材料歧义判定；
- 旧装配链路保留为离线 CLI 的显式 `--legacy-assembly` 选项，主流程不会自动回退；
- 生成器覆盖、部分重建、曲面实体和客户端建模适配仍未达到方案完成定义。

## 完成度总表

| 阶段 | 方案要求 | 当前状态 | 结论 |
| --- | --- | --- | --- |
| P0 数据契约 | 场景导出、边界/孔/岛、契约校验、单次上游分析 | 基本完成 | 缺独立验证头与 `UnsupportedSurfaceDomain`；导出有复制开销 |
| P1 多候选 | 8 类生成器迁移为候选 | 2/8 | 观测壳与平行端盖已实现；缺盖面、凸台、面交线、曲面实体等未实现 |
| P2 评分/搜索 | 公共评分、Beam、可导出诊断 | 骨架完成 | 缺未支持面/自由空间项，权重与阈值未配置化 |
| P3 主流程 | Qt/CLI 切换、legacy 隔离、适配器接线 | 部分完成 | 自动路径已切换；模板路径未迁移；新建模适配器未接入客户端 |
| P4 空间单元 | 平面单元闭合 | 未开始 | 与方案建议一致，后置 |

## 已实现内容

### 1. SurfaceScene v1 契约

实现位置：

- `core/geometric_reconstruction/include/reconstruction/solid_inference/SurfaceScene.h`
- `core/geometric_reconstruction/src/solid_inference/SurfaceSceneAdapter.cpp`

已落地：

- `SceneHeader` 保存 schema 版本、单位、分析指纹、源观测指纹、设置指纹与距离/角度容差；
- `ScenePatch` 保存支持点、质心、材料法向、方向是否已知、边界状态、边界环、支撑置信度、残差与间距；
- `SceneRelation` 保存面片端点、关系类型、误差、置信度和人工确认标记；
- `exportSurfaceScene()` 要求分析快照匹配、毫米单位，并校验面片曲面引用与支持点归属；
- 边界环从已验证有限域的三角网格外边界提取，按嵌套深度标记外环、孔和岛，并统一环方向；
- 只有“一个外环 + 若干孔”的简单域标记为 `ObservedClosed`，其余保持 `Ambiguous`；
- 优先使用同源全分辨率支撑决定，并以点集子集匹配和唯一匹配建立面片关联；
- `validateSurfaceScene()` 覆盖 `WrongUnit`、`SnapshotMismatch`、`InvalidSurfaceReference`、`MissingBoundary`、`InvalidLoop`、`MappingAmbiguous`。

### 2. 默认自动路径

实现位置：

- `core/geometric_reconstruction/src/pipeline/guided/GuidedSolidRecognizer.cpp`
- `apps/ParamCAD Studio/src/app/CadReconstructionTask.cpp`

模板列表为空时，`reconstructGuidedSolid()` 现在执行：

1. 复用或新建一次曲面分析；
2. 按需物化修复合并后的工作点；
3. 应用人工关系；
4. `exportSurfaceScene()`；
5. `inferSolids()`；
6. 先用 256 个抽样点评分筛选，再对前 3 名用原始全量观测复评；
7. 选择候选，写入 `CompoundCandidate`。

每个重建阶段都用 `StageProfile` 记录耗时。默认路径没有“识别失败后再进入旧装配”的隐式回退。

### 3. 候选生成器

| 生成器 | 实现 | 已支持 |
| --- | --- | --- |
| 观测闭合壳 | `ObservedShellClosure.cpp` | 至少 4 个闭合平面面片、共享边连通、边界图每条边使用两次、直接构造并试建边界体 |
| 平行端盖拉伸 | `ParallelCapExtrusion.cpp` | 平行平面端盖、深度取平面间距、外轮廓匹配、通孔、盲孔、外壁推断、端盖矩形吸附 |

孔相关判据已实现：

- 通孔要求两端内环匹配、内环可拟合为圆、存在同轴观测圆柱壁且角度与轴向覆盖足够；
- 盲孔要求单侧开口、匹配的孔底环和同轴圆柱壁；
- 外壁始终标记为推断面，并记录深度证据。

### 4. 评分、搜索与选择

实现位置：`core/geometric_reconstruction/src/solid_inference/SolidInference.cpp`。

已实现：

- 距离项使用 `p50` 与 `p95`，`distanceError = (p50 + 2·p95) / (3·容差)`；
- 法向项只在点方向已知且落在模型面内时统计；
- 占据冲突只对距离大于 10 倍容差且被实体分类器判为内部的点计数；
- 关系项覆盖平行、共面、垂直，并按置信度或人工确认加权；
- 复杂度项与推断侧壁惩罚；
- 接受门槛为覆盖率 0.90、P95 不超过 4 倍容差、占据冲突不超过 `max(2, 0.5% × 点数)`；
- 评分前先执行 OCCT 重建，失败即拒绝；
- Beam Search 默认宽度 4、最多 3 个操作、最多 64 个候选；组合尝试 `NewBody`、`Join`、`Cut`，并校验增材/减材方向和独立实体不重叠；
- 选择时对 0.05 分差内的候选比较材料；材料不同则返回歧义，不强行选择。

### 5. 输出与建模接口

- 特征程序写入 `CompoundCandidate::projectionPrograms`，轮廓体写入 `profileBodies`，未解释点写入 `unresolvedPointIds`；
- `ModelingAdapter` 新增 `planSolidCandidate()` / `commitSolidCandidate()`，支持 `expectedRevision` 与特征程序原子规划；
- 生产客户端仍通过 `commitCompoundCandidate()` 提交，新适配器目前只在测试中使用。

### 6. 旧路径隔离

- 离线 CLI：`pcad reconstruct cad --offline ... --legacy-assembly` 才会加载装配设置并进入 `reconstructAssembly()`；
- 离线报告的 `legacy_assembly`、`specialized_recovery_enabled`、`projection_fallback_enabled` 字段已按该语义输出；
- Studio 默认「重建 CAD」不启用旧装配；代码中存在 `reconstructCadLegacy()` 方法，但当前没有绑定到可见菜单；
- `pcad assemble` 仍然直接运行装配模块，用于独立分析，不等同于生产重建分支。

### 7. 实时预览与进度回调

- `SurfaceAnalysisProgress` 在支撑验证过程中逐片上报已接受面片，客户端用有限三角域或支持点网格生成半透明预览；
- `GuidedRecognitionProgress::solid` 在候选 OCCT 试建后上报临时形状，空形状表示撤回；
- 工作线程通过互斥待发送缓冲聚合预览，主线程以 50 ms 定时器批量刷新；
- 预览应用前再次校验点云、模型与视口实体修订；取消、结束、重新同步或修订变化都会清除预览；
- 预览不修改文档或模型修订，显示网格构造失败不影响识别结果；
- legacy 装配路径不发送预览。

### 8. 缓存与版本

- 识别缓存版本纳入 `src/solid_inference/*` 与 `apps/ParamCAD Studio/VERSION`；
- 产品版本当前为 `0.3.35`（实体推断主体提交 `d307f00` 时为 `0.3.34`）；
- 新增回归目标 `surface_scene_test`，并更新客户端重建、参数编辑和工作区交互测试。

## 与方案的偏差

| 方案项 | 当前实现 | 影响 |
| --- | --- | --- |
| 独立 `SurfaceSceneValidator.h` | 校验函数合并在 `SurfaceScene.h` | 功能等价，接口发现性略低 |
| `ScenePatch.kind` | 通过 `surfaceId` 引用 `PrimitiveEvidence` 变体 | 设计合理，但与草案字段不一致 |
| `UnsupportedSurfaceDomain` | 未实现 | 非平面面片直接不参与推断，不报特定错误 |
| 场景只读/不可变 | 场景按值复制，筛选阶段再复制一份 | 大点云有内存与耗时风险 |
| 8 类生成器 | 已实现 2 类 | 缺盖面、凸台、面交线、曲面实体等没有候选来源 |
| 推断面面积惩罚 | 只有固定 `0.2` 侧壁惩罚 | 未按推断面积或风险加权 |
| 自由空间/相机射线冲突 | 未实现 | 只用实体分类器判断内部冲突 |
| 权重标定 | 权重与阈值硬编码 | 无法按验证集调参 |
| 部分重建 | 候选必须达到全场景覆盖率 0.90 | 缺盖面、多个独立部件可能直接失败 |
| 全局空间单元 | 未实现 | 与方案后置建议一致 |

## 已知缺口与风险

1. **生成器覆盖不足**：只有完整闭合壳和双端盖拉伸。缺一个端盖时既不能闭合，也不能拉伸，与方案“缺盖面应可推断并标记置信度”的目标不一致。
2. **全局覆盖率门槛**：评分分母是整个场景的观测点。多个独立部件必须由 Beam 组合覆盖；部件数超过 `maximumFeatureOperations`（默认 3）时无法达到 0.90，整轮失败。
3. **平行端盖匹配过严**：要求两端轮廓顶点数完全一致；独立有限域的采样数不同会漏配。
4. **候选预算按顺序截断**：平面两两配对只消耗前 `maximumCandidates/2` 个名额，有效组合可能被跳过。
5. **曲面实体退化**：实体推断只处理平面。圆柱等曲面若没有两个可匹配的平面端盖，可能不再产生候选；旧识别器的解析曲面能力没有迁移。
6. **模板路径未迁移**：存在人工模板时仍走旧的模板拟合与 `recognizeParametricSolidFromAnalysis()`，不会使用 SurfaceScene。
7. **建模适配器未接线**：`planSolidCandidate()` / `commitSolidCandidate()` 没有生产调用方；客户端仍走 `CompoundCandidate` 兼容路径。
8. **性能风险**：
   - 最终评分对每个候选遍历全量观测并逐点执行 `BRepExtrema`；
   - 场景复制会重复携带曲面与面片的支持点 ID；
   - 支撑决定到面片的匹配使用集合构造，规模大时成本明显；
   - 观测闭合壳的顶点焊接与共享边检查存在二次复杂度。
9. **部署后能力回归风险**：默认路径不会自动回退到旧装配；对依赖旧专用闭合的零件，需要显式使用 `--legacy-assembly` 或继续使用 `pcad assemble` 做对照。

## 测试覆盖

新增 `core/geometric_reconstruction/tests/surface_scene_test.cpp`，目标名为 `surface_scene_test`，覆盖：

- 快照指纹失配与单位错误；
- 歧义边界不进入闭合；
- 六面箱体的观测闭合壳；
- L 形凹端盖保持多边形棱柱；
- 两级阶梯产生 Join 候选；
- 多体组合与材料不重叠检查；
- 通孔与盲孔匹配；
- 真实检测器与支撑验证链路。

客户端测试同步更新：`cad_reconstruction_task_test`、`parameter_editing_test`、`workspace_interaction_test`。工作区测试在关闭缓存时还会断言重建期间确实收到曲面与实体预览，并且预览不改变点云或模型修订。方案矩阵中的“缺盖面部分成功”“法向未知/噪声内点”“自动重建后手工修改”仍缺少专门的新增用例。

## 建议的下一步

按对目标差距的影响排序：

1. 把 `planSolidCandidate()` / `commitSolidCandidate()` 接入 Studio 与 CLI，完成修订绑定和手工修改保护的闭环。
2. 增加缺盖面推断（平面交线闭合），并为推断端盖记录来源与置信度。
3. 放宽平行端盖匹配（轮廓重采样或双向距离），同时把候选预算改为按兼容性排序。
4. 为部分重建设计独立口径，避免全场景覆盖率一票否决。
5. 补齐评分项与配置化权重，至少形成可回归的基线。
6. 治理场景复制与全量距离查询的内存/耗时，再扩大候选数量。
7. 决定模板路径是否迁移到 SurfaceScene；迁移前明确标注为既有识别器路径。
