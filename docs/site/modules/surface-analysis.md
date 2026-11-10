# 曲面分析与修复

## 职责与入口

本模块把曲面检测得到的解析证据整理成有限连通面片，推断并优化面片之间的几何关系，并在识别失败时提供局部观测恢复与修复事务的公共接口。它不决定最终 CAD 实体，但识别器读取的 `SurfaceAnalysisResult` 由这里生成。

主要入口：

- `analyzeSurfaces(points, options, result, error, seeds, cancelled)`：组合检测、面片索引和关系推断；
- `collectSurfaceEvidence()`：分族或多尺度汇集证据，合并人工种子，输出未分配点；
- `indexSurfacePatches()`：把证据支持点拆成 `SurfacePatch`；
- `indexSurfaceEvidence()`：索引面片并立即推断关系；
- `inferPatchRelations()` / `inferSurfaceRelations()`：对已有面片或自定义关系节点推断关系；
- `applySurfaceRelationHints()`：校验并确认人工关系；
- `optimizeSurfaceRelations()`：对可接受的关系子集做观测约束下的联合优化；
- `retrieveLocalObservations()` / `recoverLocalSurfaces()`：在失败区域取回同源观测并局部重检；
- `recognizeAndRepairParametricSolid()`：识别侧的修复事务，负责合并证据、复验和提交或回退。

## 输入与输出

输入：

- `PointStore`：同一观测快照中的点，支持 ID 必须存在；
- `SurfaceAnalysisOptions`：检测选项、关系容差、连通距离、是否分族、是否恢复残余曲面、局部恢复参数；
- 可选的 `seeds` 人工证据与取消回调。

关键默认值：`connectivityDistance = 0` 表示按支持密度自动估计连通半径；`independentFamilies = true` 让五类基元分别检测；`recoverResidualSurfaces = false` 默认关闭多尺度残余恢复。局部恢复最少区域点数 20，halo 容差倍率 2、关系倍率 16，稳定平面重叠比例 0.1、法向余弦 0.95，局部连通容差倍率 2、自适应上限倍率 4。

输出 `SurfaceAnalysisResult`：

- `pointFingerprint`、`unit`、`options`：本分析快照的身份；
- `surfaces`、`patches`、`relations`、`unassignedPointIds`：重新编号的证据、有限面片、关系假设和未解释点；
- `detectionPasses`、`localFailures`：检测轮数与识别返回的局部失败诊断；
- `baseFingerprint`、`recoverySourceFingerprint`、`supplementalPoints`：修复合并后的基线与补点；
- `repairMappings`、`repairPatchMappings`：局部基元/面片到全局 ID 的映射。

基元 ID 和面片 ID 都只在当前分析快照内有效，换点集或换单位后必须重新索引。

## 核心数据结构与依赖

- `PatchId`：面片 ID，只在一次分析内唯一。
- `SurfaceRelationKind`：`Adjacent`、`Parallel`、`Perpendicular`、`Coplanar`、`Coaxial`、`SameRadius`、`Intersect`、`Tangent`、`AxisNormalToPlane`、`AxisParallelToPlane`。
- `SurfaceRelation`：`first`、`second`、`kind`、`error`、`confidence` 与 `automatic`；`automatic = false` 表示人工确认。
- `SurfacePatch`：`surfaceId`、`supportPointIds`、包围范围 `lower`/`upper`、`centroid`、平均 `materialNormal` 与 `orientationKnown`。
- `RelationSurface`：关系推断节点，可带可选证据；证据为空时仍可参与邻接判断。
- `LocalAnalysisFailure`：失败原因、区域点 ID、候选、修复候选、拒绝/修复关系、补点和指纹。
- `LocalObservationRecovery`：局部取点结果，含局部点、补点 ID、源指纹与搜索范围。
- `RelationOptimizationReport`：接受关系、拒绝关系与优化前后误差。

依赖检测模块的 `PrimitiveEvidence` 与五类解析方程、`PointStore` 的源点 ID、Eigen3 的阻尼最小二乘，以及项目 `KdTree` / `Grid` 的面片连通、邻接与局部恢复。它不依赖 Qt 或 VTK。

## 面片索引、关系优化、局部恢复与复验

**面片索引。** 每个基元的支持点先按 ID 去重，再由广度优先搜索拆成连通分量。连通半径取 `connectivityDistance`；为 0 时取 `max(关系距离容差 × 4, 2.5 × sqrt(2 × (dx·dy + dy·dz + dz·dx) / n))`，其中包围范围来自支持点。每个面片记录质心、范围、支持点平均法向；只有全部支持点都带已知定向法向时 `orientationKnown` 才为真。

**关系推断。** 两个面片先做包围盒距离剪枝，再用网格邻域确认是否在距离容差内相邻。随后按基元类型追加关系：

- 平面与平面：法向对齐达到平行门限时给 `Parallel`，偏移相等时追加 `Coplanar`；接近垂直时给 `Perpendicular`，相邻时追加 `Intersect`；
- 轴向基元与轴向基元：轴线平行且横向偏差在容差内给 `Coaxial`；
- 平面与轴向基元：按轴与平面法向的对齐/垂直给 `AxisNormalToPlane` 或 `AxisParallelToPlane`；
- 同类且半径差在容差内给 `SameRadius`；
- 平面与球/圆柱相切、或两球内外相切且相邻时给 `Tangent`。

置信度取两个证据置信度与 `maximumConfidence` 的最小值，默认上限 0.95；派生关系再乘 `derivedConfidenceFraction`，默认 0.8。相邻关系默认置信度 0.75。关系是几何假设，不会自动改写模型。

**人工关系校验。** 每个 `SurfaceRelationHint` 携带两组点 ID。实现按支持点重叠比例找到对应面片，要求重叠不少于两侧较小支持集的 `manualPatchOverlapFraction`（默认 0.6）。随后只在已有自动关系中查找同类型、同端点的关系；找不到即返回冲突错误，不会凭空添加几何约束。确认后把该关系标为 `automatic = false`。

**关系优化。** `optimizeSurfaceRelations()` 只处理平面、圆柱和圆锥，并且只接受 `Parallel`、`Perpendicular`、`Coaxial`、`Coplanar`、`SameRadius`、`AxisNormalToPlane` 六类提议。默认预算为最多 32 次尝试、12 个活跃曲面、256 个残差采样目标、15 次迭代、初始阻尼 `1e-3`、收敛步长 `1e-7`。每次尝试要求支持点内点率不低于 0.9、平均误差增加不超过 `0.15 × 距离容差`，并按 `validationToleranceFraction = 0.25` 复验：距离关系用 0.25 × 距离容差，角度关系用 0.25 × 角容差；同时保持圆柱半径和圆锥张角有效。不满足的提议进入 `report.rejected`，整体误差写入 `beforeError` / `afterError`。

**局部观测恢复。** `retrieveLocalObservations()` 要求工作快照与分析指纹匹配，且工作点与源点单位一致。失败区域 ID 必须存在于工作快照，源快照也必须包含这些 ID，否则报“不同坐标帧或点身份”。搜索 halo 默认为 `距离容差 × 2`；区域小于 20 点时放大为 `max(距离容差 × 2, 关系距离容差 × 16)`。恢复只取源快照中实际存在、方向已知、且未被稳定平面解释的观测；区域过小时再裁剪到与失败区域连通的连通分量。它不会生成未观测的点。

**局部重检与修复事务。** `recoverLocalSurfaces()` 保留失败区域内的原候选，再以两个有界检测轮次补充新候选：平面上限 24、圆柱上限 8、平面/圆柱/圆锥最低支持 20，不放松距离和边界完整性阈值。随后重建面片并优化关系，结果写入 `LocalAnalysisFailure`，但 `repaired` 仍为假，只有实体识别器可以确认修复成功。

识别侧的 `recognizeAndRepairParametricSolid()` 按源点 ID 去重合并观测与修复候选，重建全局索引，再用原始点 ID 重新绑定人工关系。提交前要求长方体、棱柱、贯穿孔和曲面实体数量不变，每个修复区域的局部边界覆盖率不下降，曲面实体平均残差不增加超过 `0.15 × 距离容差`，几何签名漂移不超过 `0.25 × 距离容差`，全局边界覆盖率不下降且 `surfaceCoverage ≥ 0.98`。任一项失败即拒绝合并，保留原分析和基线候选。成功时写入 `baseFingerprint`、`recoverySourceFingerprint`、补点和局部到全局映射；已有 `baseFingerprint` 的快照若再次出现待修复区域，会要求新的恢复事务，而不是无限叠加。

**快照指纹与缓存复用。** `pointStoreFingerprint()` 混合单位、ID、位置、法向存在性、定向状态、法向分量和置信度。`analysisMatches()` 要求单位与指纹同时一致；`analysisInputMatches()` 在存在 `baseFingerprint` 时改比基线指纹；`analysisSettingsMatch()` 比较检测种子、启用的基元族、阈值、关系容差等选项。客户端只有在输入与设置都匹配时才复用 `sharedAnalysis`，并标记 `analysisReused = true`。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 分析入口与公共类型 | `core/geometric_reconstruction/include/reconstruction/analysis/SurfaceAnalysis.h` |
| 证据汇集与分析组合 | `core/geometric_reconstruction/src/analysis/SurfaceAnalysis.cpp` |
| 面片索引与关系推断入口 | `core/geometric_reconstruction/src/analysis/SurfacePatches.cpp` |
| 关系计算 | `core/geometric_reconstruction/src/analysis/SurfaceRelations.cpp` |
| 关系优化 | `core/geometric_reconstruction/src/analysis/RelationOptimization.cpp` |
| 局部观测恢复 | `core/geometric_reconstruction/src/analysis/LocalRecovery.cpp` |
| 快照指纹与复用判定 | `core/geometric_reconstruction/src/analysis/AnalysisSnapshot.cpp` |
| 修复合并与复验 | `core/geometric_reconstruction/src/recognition/RepairTransaction.cpp` |

## 测试与诊断

## 已知限制

- 关系只是几何假设，默认不强制模型满足；人工关系只能在已有自动关系中确认同类型、同端点项，冲突时直接报错。
- 关系优化只覆盖平面、圆柱和圆锥，且只接受六类关系；球、圆环以及 `Tangent`、`Intersect` 等关系不会参与联合拟合。
- 优化默认预算为 32 次尝试、12 个活跃曲面和 15 次迭代，超出预算的提议会被拒绝；预算可在 `analysis.conf` 调整，但结果不是全局最优保证。
- 局部恢复要求工作点与源点同单位、同坐标帧，且源快照必须包含失败区域的全部观测 ID；不能补出未观测的端面、壁厚或遮挡区域。
- 修复必须保持特征数量不变并满足覆盖率、残差和签名复验；任一条件不满足就保留原分析，缺失面仍然缺失。
- 分析结果与点快照和选项绑定。点集、单位或检测/关系设置变化后缓存失效，必须重新分析；修改毫米容差也不会自动换算到任意单位点云。
