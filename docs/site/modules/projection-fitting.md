# 二维轮廓与投影回退

所属主线：[模块4：多体分组与闭合](/modules/point-cloud-to-cad)。装配路径将开放部件交给这里尝试拉伸/布尔程序；本模块还可从调试入口独立调用。孔槽的特征意义见[模块3](/modules/feature-recognition)，最终提交见[模块5](/modules/parametric-modeling)。

## 职责与入口

本模块把单个部件的观测投影到一个候选轴上，恢复二维截面轮廓，并搜索“基体拉伸 + 台阶/槽/孔切除或合并”的操作序列。它输出的是可回放的投影程序，只有当临时试建出的实体满足评分门槛时才发布；失败时保留候选轴、拒绝程序和质量诊断。

对外入口：

- `fitProjectionExtrusion(source, options, result, evidence, cancelled)`：完整拟合；
- `inspectProjectionSketches(...)`：只准备截面输入，不建体，用于调试；
- `buildProjectionProgram(operations, model, error)`：把操作序列独立重放到 `ModelingCore`，成功后原子接管模型；
- `fitProjectionSketchBoundary(input)`：返回 CAD 草图拓扑与壁面检查之前的原始截面区域。

装配回退通过 `fitOpenParts()` 调用本模块，生产任务也可从投影调试入口直接进入。调试入口把 `inspectOnly` 设为 `true` 时只调用 `inspectProjectionSketches()` 输出候选截面，并把被拒程序或预览实体保留在诊断结果中，不提交到模型。

## 输入与输出

输入：

- 单件 `PointStore`，必须为毫米制，至少 `2 × minimumCapPoints`（默认 24）个点；
- 可选 `PrimitiveEvidence`：必须与点集同单位、同坐标系、同源点 ID；
- `ProjectionExtrusionOptions`：路径、距离容差、轴/拟合/评分点预算、截面与孔槽门槛、搜索预算；
- 可选取消回调。

输出 `ProjectionExtrusionResult`：

- `distanceTolerance`：实际使用的容差；
- `axes`：候选轴及来源、凸投影面积、包络体积、状态、最佳基体质量；
- `operations`：通过验收的操作序列；`rejectedOperations`：被拒但保留的最佳程序；
- `quality`：评分点数、中位距离、P95 距离、表面内点率、无支持面面积比例、平均法向误差和总分；
- `residualPointIds`：距离超过容差的源点 ID；
- `bestAxisIndex`、`reason`、`sketchInputs`：最佳轴、失败原因和调试截面。

距离容差为零时按部件包围对角线乘 `0.0025` 估计；拟合点最多 24000，评分点最多 1800，体素化尺度为 `2`。

## 核心数据结构与依赖

| 类型 | 关键内容 |
| --- | --- |
| `ProjectionPath` | `Automatic`、`SingleAxis`、`TwoAxisIntersection`、`Segmented` |
| `ProjectionExtrusionOptions` | 轴上限 12、操作上限 3、端面点数 12、草图边上限 80、评分门槛 |
| `ProjectionSketchInput` | 草图平面、截面类别、二维点、alpha 半径、首点 ID、区域支持 |
| `ProjectionOperation` | 草图特征、`NewBody/Join/Cut/Intersect`、深度、方向、支持点 |
| `ProjectionFitQuality` | 内点率、P95、无支持面比例、法向误差、总分 |
| `ProjectionAxisReport` | 轴向、来源字符串、凸投影面积、包络体积、状态 |

依赖：

- **Eigen3**：PCA 主轴、侧壁法向矩、圆拟合；
- **CGAL 二维 Delaunay**：通过 `BoundaryGeometry` 提取 alpha 轮廓；
- **OpenCASCADE**：临时建体、距离查询、面面积和体积；
- `ModelingCore`：操作序列的独立试建与原子接管；
- 项目预处理：缺失法向估计、评分点体素采样。

## 候选轴、截面轮廓、操作序列与回退

候选轴按证据优先级生成：圆柱轴线、平面法向、PCA 三个主轴、侧壁平面法向两两叉积、侧壁法向矩的最小特征向量，以及可选的“小凸投影面积”方向。重复方向按 `0.9986` 余弦合并，总数不超过 12。

区间与端面：

- 轴向深度去掉两端各 0.2% 的极值后作为拉伸区间；
- 端面要求深度位于 `3 × 容差` 带内、法向与轴夹角余弦不低于 0.9，且至少 12 个支持点；不满足时该端面为空。
- 拉伸深度小于 `3 × 容差` 时该轴不产生提案。

二维截面：

- 对外环做 alpha 轮廓并按面积过滤，最大环为外轮廓；孔环必须有多个深度带的侧壁支持。
- 轮廓可拟合为圆：至少 8 个顶点、半径不小于 `5 × 容差`、角度缺口不超过 60°；否则退化为线段轮廓。
- 草图边上限为 80。一次拉伸只接受一个连通区域；多个外区域直接判为无效。

提案类型：

- 基体：由首端面或末端面截面生成 `NewBody` 拉伸。
- 台阶：由内部平行平面把区间切成上下两段，形成可合并的分段或 `Join` 提案；最多使用 2 个内部截面。
- 直槽：底面轮廓须为四条线段，至少两条壁被支持（每条壁至少 6 点，深度达槽深的 0.6），生成带过切的 `Cut`。
- 盲孔：圆柱角度覆盖不低于 0.65，孔底和开口环带各有至少 6 点、深度大于 `3 × 容差`，生成圆形 `Cut`。
- 贯穿孔：要求两端都有开口点、内壁点足够、无材料阻塞，半径大于 `2 × 特征容差`，残差不大于 1 倍特征容差。

评分与验收：

- 总分 = 平均截断距离损失 + 0.5 × 平均法向损失 + 无支持面面积比例 + 0.015 × 操作数 + 0.001 × 曲线数；
- 接受条件为表面内点率不低于 0.90、P95 距离不超过 `3 × 容差`、无支持面面积比例不超过 0.04；
- 搜索使用宽度 4 的束搜索，最多 3 个操作；新程序必须带来超过 0.01 的分数改进、达到接受条件，或补足规定的最小操作数；布尔体积变化必须大于 `1 × 容差³`；
- 完整程序生成后会尝试删除冗余操作，只有在仍可接受且分数不劣于原程序时才剪枝；
- 终态分为 `Accepted`、`InsufficientSupport`、`NoValidBase` 和 `Cancelled`，只有 `Accepted` 才发布操作序列。

装配回退中，`fitOpenParts()` 只处理未闭合、至少 2 个面片、至少 60 个全量点的部件，按点数降序最多处理 6 个；成功部件标记为 `closed_projection`，置信度取表面内点率。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 公开接口与选项 | `core/geometric_reconstruction/include/reconstruction/recognition/ProjectionExtrusion.h` |
| 拟合入口、选轴、区间与试建 | `core/geometric_reconstruction/src/recognition/projection/ProjectionExtrusion.cpp` |
| 二维截面恢复 | `core/geometric_reconstruction/src/recognition/projection/ProjectionProfile.cpp` |
| 基体、台阶、槽与孔提案 | `core/geometric_reconstruction/src/recognition/projection/ProjectionProposals.cpp` |
| 固定点评分 | `core/geometric_reconstruction/src/recognition/projection/ProjectionScoring.cpp` |
| 操作序列搜索 | `core/geometric_reconstruction/src/recognition/projection/ProjectionSearch.cpp` |
| 投影参数读写 | `core/geometric_reconstruction/src/recognition/projection/ProjectionSettings.cpp` |
| 默认投影参数 | `apps/ParamCAD Studio/config/projection.conf` |

## 已知限制

- 一次拉伸只接受一个连通外轮廓。凹形轮廓、多个分离区域或复杂内腔需要拆分为多次操作，当前搜索最多只允许 3 个操作。
- 端面必须由真实观测支持。缺失端面时该轴被跳过，不会用凸包或对称假设补出端面。
- 截面轮廓由 alpha 形状和线段/圆拟合得到，细小圆角、样条边界和小于采样间距的特征会被简化或丢失。
- 评分建立在固定采样视图上，内点率只反映该视图的解释程度；不能证明未观测区域，也不能替代真值几何比较。
- 投影回退最多处理 6 个开放部件，且每个部件至少 60 个全量点；更小的开放部件不会进入拟合。
- 操作序列的几何正确性依赖临时 `ModelingCore` 建体成功；布尔失败或多实体结果会被直接丢弃，没有通用修复。
