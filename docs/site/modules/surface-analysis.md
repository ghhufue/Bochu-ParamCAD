# 曲面拓展、验证与局部修复

所属主线：[模块2：曲面拓展与验证](/modules/point-cloud-to-cad)。本页集中说明面范围、支撑、关系和补证；检测器细节见[曲面分割](/modules/surface-detection)，孔、凸台和实体分类见[特征与实体识别](/modules/feature-recognition)。

## 职责与入口

本模块组织自动平面拓展，把解析证据验证为有观测支撑的有限连通面片，再建立几何关系；识别失败时提供局部观测恢复与修复事务。面片通过只证明该面的支撑足够，最终孔、凸台及实体分类仍由识别器完成。

主要入口：

- `analyzeSurfaces(points, options, result, error, seeds, cancelled)`：组合检测、拓展、有限支撑验证和关系推断；
- `collectSurfaceEvidence()`：分族或多尺度汇集证据，拓展自动平面、合并人工种子；
- `recoverPlaneSupport()`：自动平面连通生长与重拟合；
- `indexSurfacePatches()`：仅把证据支持点拆成初步 `SurfacePatch`，不代表通过支撑验证；
- `validateSurfaceSupport()`：按有限连通域检查面支撑，输出接受集与诊断；
- `indexSurfaceEvidence()`：验证后重建面片与关系，保留原始提案；
- `revalidateSurfaceSupportOnSource()`：在同源原始观测上独立补证、验证，再更新工作分析；
- `resolveSurfaceOwnership()`：将源点分析发布为唯一点归属，执行大平面优先、冻结参考复验和有界重新分配；
- `inferPatchRelations()` / `inferSurfaceRelations()`：对已有面片或自定义关系节点推断关系；
- `applySurfaceRelationHints()`：校验并确认人工关系；
- `optimizeSurfaceRelations()`：对可接受的关系子集做观测约束下的联合优化；
- `retrieveLocalObservations()` / `recoverLocalSurfaces()`：在失败区域取回同源观测并局部重检；
- `recognizeAndRepairParametricSolid()`：识别侧的修复事务，负责合并证据、复验和提交或回退。

## 输入与输出

输入：

- `PointStore`：同一观测快照中的点，支持 ID 必须存在；
- `SurfaceAnalysisOptions`：检测选项、支撑验证与有限域设置、关系容差、连通距离和局部恢复参数；
- 可选的 `seeds` 人工证据与取消回调。

公开分析选项的关键初值：`connectivityDistance = 0` 表示按支持密度自动估计连通半径；`independentFamilies = true` 让五类基元分别检测；`recoverResidualSurfaces = false` 关闭多尺度残余恢复；`expandPlaneSupport = true` 对自动平面运行连通支持扩展。局部恢复最少区域点数20，halo容差倍率2、关系倍率16，稳定平面重叠比例0.1、法向余弦0.95，局部连通容差倍率2、自适应上限倍率4。生产入口会加载配置并按任务覆盖选项；例如客户端整景“识别表面”会启用残余恢复，不能仅由结构体初值判断某次任务走了哪条路径。

输出 `SurfaceAnalysisResult`：

- `pointFingerprint`、`unit`、`options`：本分析快照的身份；
- `surfaces`、`patches`、`relations`、`unassignedPointIds`：重新编号的证据、有限面片、关系假设和未解释点；
- `supportProposals`、`supportProposalIds`：验证前提案和已接受方程到原提案的映射；
- `supportValidation`、`sourceSupportValidation`：工作点与原始点各自的参考密度、设置及判定；
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

**自动平面拓展。** `collectSurfaceEvidence()` 调用 `recoverPlaneSupport()`，合并相容种子，按局部间距生长，累计重拟合并重访先前拒绝的边界点。距离容差保持不变，方向漂移与全部支持点继续受约束。该通用生长流程针对自动平面；曲面模板补全和识别失败后的局部重检属于各自的有界路径。方法见[平面支持连通扩展](/algorithms/primitive-fitting#plane-support-recovery)。

**初步面片索引。** 每个基元的支持点先按 ID 去重，再由广度优先搜索拆成连通分量。连通半径取 `connectivityDistance`；为0时按包围范围和不同坐标位置数估计，重复位置不人为缩小连接尺度。每个面片记录质心、范围和平均法向；只有全部支持点都带已知定向法向时 `orientationKnown` 才为真。这一步仅拆片，最终面域还须通过下面的支撑验证。

**有限支撑验证。** 默认使用 `enforce`。从高置信度大平面建立固定参考密度，按连通域检查面密度与内部占用；参考不足或不稳定时明确回退到局部支撑。球面另检查立体角与较小方向展开角，圆柱检查连续绕轴展开角，圆锥按连续面片检查二维覆盖和弓高；三者都要求弓高超过噪声且优于稳健平面替代。可靠点建立有限域后，域内全部有效观测计入覆盖，孔洞和域外线尾保留。具体阈值与面积口径统一放在[面密度与曲面展开验证](/algorithms/surface-support-validation)。

人工种子按基元类型分别保护：`ProtectedSurfaceSeed` 记录基元类型与支持点，验证时只有同类型的种子点才把候选标记为 `protectedManual`，避免一个平面的手动点误保护圆柱或其他类型候选。保护候选未通过验证时，错误信息会附带类型、面片 ID、点数、有效点、覆盖点、面积与保护标记。

**凹球面片段恢复。** 主验证完成后，分析还会在相邻平面碎片包围的未解释区域局部重检球面和圆锥，用于补回被平面截断的凹球面或球冠。面积达到参考中位数一定比例的大平面和人工保护面片不参与区域构造；候选必须跨越至少两个面片，并再次通过同一固定参考的有限支撑验证。同一球心的候选按支持集重叠合并，必要时在原始分辨率继续扩展。该路径不放宽原距离和法向阈值，细节见[凹球面与曲面片段恢复](/algorithms/surface-support-validation#凹球面与曲面片段恢复)。

**原始点与最终归属。** 工作点、原始观测分别建立统计上下文，不能混用密度。核心 `resolveSurfaceOwnership()`在源点补证后处理独占归属；通过验证的大平面优先保留有限域和原容差内的点。归属后按冻结参考再验证，释放点最多重分配一轮并复验。该接口只使用源点ID和显式设置，检查快照及验证签名，取消或失败时保持输出不变。客户端只映射文档行和描述结果。未解释点保留，显示网格不能重新跨接拒绝域；用于识别的原分析仍可保留重叠证据。CAD 任务未闭合时，客户端可用 `indexValidatedSourceSurfaces()` 把原始点上已验证的面片重新索引并显示，而不是退回未经归属的工作点结果。

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

**快照指纹与缓存复用。** `pointStoreFingerprint()` 混合单位、ID、位置、法向存在性、定向状态、法向分量和置信度。`analysisMatches()` 要求单位与指纹同时一致；`analysisInputMatches()` 在存在 `baseFingerprint` 时改比基线指纹；`analysisSettingsMatch()` 比较检测、关系、拓展和支撑验证设置。影响有限域的设置参与签名，纯显示采样预算不改变支撑统计。原始点补证还检查源指纹；只有身份与设置匹配时才复用缓存。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 分析入口与公共类型 | `core/geometric_reconstruction/include/reconstruction/surface/SurfaceAnalysis.h` |
| 证据汇集与分析组合 | `core/geometric_reconstruction/src/surface/SurfaceAnalysis.cpp` |
| 自动平面拓展 | `core/geometric_reconstruction/src/surface/expansion/PlaneSupportRecovery.cpp` |
| 有限面域与支撑验证 | `core/geometric_reconstruction/src/surface/SurfaceFootprint.cpp`、`SurfaceSupportValidation.cpp` |
| 凹球面与曲面片段恢复 | `core/geometric_reconstruction/src/surface/CurvedFragmentRecovery.cpp` |
| 原始点集合归属与复验 | `core/geometric_reconstruction/src/surface/SurfaceOwnership.cpp` |
| 客户端数据与结果适配 | `apps/ParamCAD Studio/src/io/PointCloudAnalysis.cpp` |
| 面片索引与关系推断入口 | `core/geometric_reconstruction/src/surface/SurfacePatches.cpp` |
| 关系计算 | `core/geometric_reconstruction/src/surface/SurfaceRelations.cpp` |
| 关系优化 | `core/geometric_reconstruction/src/surface/RelationOptimization.cpp` |
| 局部观测恢复 | `core/geometric_reconstruction/src/surface/LocalRecovery.cpp` |
| 快照指纹与复用判定 | `core/geometric_reconstruction/src/surface/AnalysisSnapshot.cpp` |
| 修复合并与复验 | `core/geometric_reconstruction/src/pipeline/recovery/RepairTransaction.cpp` |

## 测试与诊断

`plane_support_recovery_test` 验证平面拓展；`surface_support_validation_test` 覆盖密度参考、浅曲面、圆锥、孔洞、窄条、分离域和凹球面片段恢复；`surface_analysis_test` 验证分析与修复；`point_cloud_analysis_test` 验证客户端集合发布。分析包的 `support-validation.json` 保存归属前后判定，见[分析结果导出与查询](/validation/analysis-export)。

## 已知限制

- 自动平面生长不等于所有解析曲面都有通用拓展器；球面、圆柱的展开验证也不等于孔或实体识别。
- 凹球面片段恢复要求至少两个已通过验证的相邻平面碎片，并把局部窗口限制在 8192 点以内；没有平面邻域或孤立的小球冠仍可能漏检。
- 关系只是几何假设，默认不强制模型满足；人工关系只能在已有自动关系中确认同类型、同端点项，冲突时直接报错。
- 关系优化只覆盖平面、圆柱和圆锥，且只接受六类关系；球、圆环以及 `Tangent`、`Intersect` 等关系不会参与联合拟合。
- 优化默认预算为 32 次尝试、12 个活跃曲面和 15 次迭代，超出预算的提议会被拒绝；预算可在 `analysis.conf` 调整，但结果不是全局最优保证。
- 局部恢复要求工作点与源点同单位、同坐标帧，且源快照必须包含失败区域的全部观测 ID；不能补出未观测的端面、壁厚或遮挡区域。
- 修复必须保持特征数量不变并满足覆盖率、残差和签名复验；任一条件不满足就保留原分析，缺失面仍然缺失。
- 分析结果与点快照和选项绑定。点集、单位或检测/关系设置变化后缓存失效，必须重新分析；修改毫米容差也不会自动换算到任意单位点云。
