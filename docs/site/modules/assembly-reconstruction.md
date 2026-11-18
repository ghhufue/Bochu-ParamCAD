# 多体分组与有限边界闭合

所属主线：[模块4：多体分组与闭合](/modules/point-cloud-to-cad)。这里将面片组织成部件，与[模块1的曲面分割](/modules/surface-detection)区分；本路径会独立提取面片。闭合失败后的程序拟合见[投影回退](/modules/projection-fitting)。

## 职责与入口

本模块在常规实体识别无法给出单一可信候选时，把同一份观测分解为有限面片、连接关系、部件分组和闭合实体。它只读取观测快照，不接受源模型、文件路径或参考标签；分解出的部件也不被断言为原始装配实例。

唯一入口是 `reconstructAssembly(points, options, output, error, cancelled)`，要求输入为毫米制 `PointStore`。生产任务在满足以下条件时调用它：没有模板、没有人工关系，并且常规识别失败、候选实体数大于一，或“单箱体但表面覆盖低于 0.99”需要拆分细化。细化模式下若分离结果只有一个解析轮廓体，并且其通孔数不少于原候选，任务会采用分离结果；否则保留原候选并丢弃装配结果。未启用投影回退或处于细化模式时，生产任务把 `maximumProjectionParts` 置零。

装配内部固定按 `extractPatches → groupPatches → recoverBoundaries → assignSourceObservations → closeParts → 专用闭合 → recoverLocalProfiles → fitOpenParts` 执行。边界恢复使用工作点，全量回填在其后发生；这一顺序决定了两种点归属的语义。

## 输入与输出

输入：

- 全量观测快照：毫米坐标、可选法向、稳定源点 ID；
- `AssemblyOptions`：距离容差、工作点上限、邻域、最小面片点数、法向角、连通与 alpha 系数、闭合预算、投影回退开关；默认工作点上限 200000、邻域 32、最小面片 24、法向角 10°、连通系数 2.5、alpha 系数 2.5、部件面片 4–32、投影回退 6 个；
- 可选取消回调。

输出 `AssemblyReconstructionResult`：

- `patches`：有限面片，含方位、范围、平面性、局部间距、残差、有限边界环和两种点归属；
- `connections`：面片之间的连接证据，含凸连接计数、观测数、距离和是否接受；
- `parts`：部件分组、包围范围、闭合状态、开边/非流形计数、置信度、状态与原因；
- `candidate`：通过验证的长方体、棱柱、轮廓体、曲面实体或投影程序；
- `unassignedPointIds` 与 `unassignedWorkingPointIds`：未归属的全量点与工作点；
- `inputPoints`、`workingPoints`、`distanceTolerance`：本次分解的规模与实测容差。

默认容差为零时按观测自动估计：包围盒对角线乘 `1e-6`，或噪声下四分位数的三倍，取较大者；随后再以对角线 `1e-9` 兜底。

## 核心数据结构与依赖

`FiniteSurfacePatch` 是本模块的核心记录：

- `id`、`origin`、`normal`、`xDirection`、`lower`、`upper`、`spacing`、`residual`；
- `planar` 与 `orientationKnown`；
- `loops`：世界坐标有限轮廓；第一环围成最大面积，后续环表示孔或分离岛；
- `workingPointIds`：支撑拟合与观测边界的装配采样点；
- `pointIds`：全量观测回填后的源点 ID，与 `workingPointIds` 独立；
- `boundaryBeforeClosure`：闭合重写轮廓前保存的观测边界与修订来源。

`AssemblyPart` 记录 `patchIds`、两种点归属、包围范围、`closed`、`openEdges`、`nonmanifoldEdges`、`confidence`、`status`、`reason` 和可选的 `projection`。

`BoundaryRevisionSource` 区分两种闭合修订：`ConvexSupportPlanes`（由凸支撑平面重算）与 `MergedCoplanarPatch`（合并共面面片时清空轮廓）。

依赖：Eigen3 用于协方差、平面交点和最小二乘；CGAL 二维 Delaunay 用于 alpha 轮廓；OpenCASCADE 用于 `ProfileBodyParams` 建体、曲面实体试建和实体分类；下游建模核心用于原子试建。模块本身不依赖 Qt 与 VTK。

## 面片分组、全量回填、边界与闭合

面片提取：

1. 工作点按坐标哈希截断到 200000 点，按位置排序，缺失法向时用 32 邻域重新估计。
2. 以法向一致性和局部间距生长面片，跨过观测到的垂直壁时切断连通；支持点达到 12 后周期性用全部支持点重拟合平面并限制方向漂移，重访此前被拒的边界点；点数少于 24 的种子被丢弃。
3. 未分配点先尝试并入邻近平面，再按局部平滑性生长为曲面面片。
4. 相邻共面平面碎片在双方全部支持点都满足对方平面时合并，合并后保留种子平面方程；每个面片记录中位间距、残差和平面性，残差不超过容差时标记为平面。

分组与回填：

- 相邻面片法向点积低于 0.995（约 5.7°）、距离在连通范围内、且不跨越观测边界时建立连接证据。
- 已知方向时要求凸连接比例不低于 0.7 且至少 3 条连接；方向未知时按总连接数接受。接受后并入同一部件。
- 全量回填对每个源点查找最近 4 个工作面片点，检查邻近距离与法向一致性；找不到时进入未分配集合。

有限边界：

- 每个平面面片用 alpha 形状提取外环，alpha 半径取 `max(2×容差, 间距×2.5)`，简化容差取 `max(容差, 间距×0.45)`。
- 轮廓按面积排序，最大环为外轮廓；分割出的环可作为孔或分离岛。
- 与相邻面片的交线作为约束线吸附边界顶点，吸附后拓扑不一致时回退原始环。
- 诊断计数器记录非简单环、分割环、被拒环、简化/约束/吸附/拓扑回退，只统计，不改变几何。

闭合：

- 闭合默认使用观测边界；当部件有 4–32 个平面面片时，另尝试由凸支撑平面重算轮廓，要求至少 4 个支撑平面、无衰退方向、可行顶点至少 4 个。
- 支撑平面重算时，顶点由三个拟合平面的交点定义；闭合阶段再用相邻面法向修正共享顶点位置。
- 每条边只被使用一次计为开边；被使用三次以上或方向不一致计为非流形边。存在任一情况即不闭合。
- 失败状态包括 `open`、`ambiguous`、`orientation_unknown`、`closed_boundary`、`invalid_shell` 和 `material_conflict`；成功状态为 `closed_solid` 或 `closed_primitive`。
- 内部观测与实体内部分类冲突超过 `max(2, 点数/200)` 时拒绝该实体。
- 曲面闭合复用独立曲面识别，样本数须在 80–20000 之间，覆盖率门槛 0.95，最多尝试 32 个部件。

专用闭合与回退：

- 解析轮廓闭合要求拟合支持点覆盖可支持观测的 98%，部分拟合还要求可支持点至少占部件点的 80%；成功状态为 `closed_analytic_profile`。
- 解析轮廓中的圆孔按开口、内壁、同轴关系和分段观测进一步判断，可形成贯穿孔、沉孔/阶梯孔及带圆锥钻尖的盲孔。相关分段写入 `ProfileBodyParams::cavities`，不统一写成 `ThroughHoleCandidate`；不能只统计 `throughHoles` 字段判断是否恢复了孔。该路径仍要求宿主轮廓、端面与对应孔壁受到观测支持。
- 环形管闭合要求内外壁与两个观测端面同时成立，状态为 `closed_annular_tube`。
- 圆盘闭合要求带符号端面、弦边和圆弧壁；推断接触时状态为 `closed_circular_slab_inferred_contact`。
- 局部轮廓恢复另用 8–64 个平面、父子分割点数门槛、保留比例 0.98 和父支持比例 0.90 等预算，用于拆分过大的开放组。
- 开放部件若至少 2 个面片、至少 60 个全量点，按点数从大到小最多试拟合 6 个；成功后状态为 `closed_projection`，置信度取投影内点率。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 公开入口与选项 | `core/geometric_reconstruction/include/reconstruction/parts/AssemblyReconstruction.h` |
| 面片、分组、回填与调度 | `core/geometric_reconstruction/src/parts/assembly/AssemblyReconstruction.cpp` |
| 有限边界恢复 | `core/geometric_reconstruction/src/parts/assembly/FiniteBoundaries.cpp` |
| 轮廓几何与拓扑 | `core/geometric_reconstruction/src/parts/assembly/BoundaryGeometry.cpp` |
| 壳闭合与实体验证 | `core/geometric_reconstruction/src/parts/assembly/PartClosure.cpp` |
| 解析轮廓闭合 | `core/geometric_reconstruction/src/parts/assembly/AnalyticProfileClosure.cpp` |
| 环形管与圆盘闭合 | `core/geometric_reconstruction/src/parts/assembly/AnnularTubeClosure.cpp`、`CircularSlabClosure.cpp` |
| 默认装配参数 | `apps/ParamCAD Studio/config/assembly.conf` |

## 已知限制

- 分解是纯观测的凸面片方法，凹部件、接触面和被遮挡的内部结构可能保持歧义，不能保证恢复原始装配实例。
- 闭合要求每个参与面片有闭合的有限轮廓，且共享边被相邻面一致使用；缺一个端面通常表现为开边，不会被观测凸包自动补成实体。
- 工作点上限为 200000，超过后按坐标哈希截断；极小特征可能在采样阶段丢失，分解结果依赖截断后的采样。
- 开边、非流形边、方向未知和材料冲突都会阻止闭合；当前没有自动修补这些拓扑缺陷的通用流程。
- 投影回退最多处理 6 个开放部件，且只接受至少 2 个面片、60 个全量点的部件；更小的开放部件不会生成实体。
- 专用闭合只覆盖解析轮廓、环形管和圆盘三类，其他回转体或自由曲面仍需局部恢复或人工模板。
