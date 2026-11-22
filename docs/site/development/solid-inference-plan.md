# 实体推断实施方案（参考）

> **性质**：架构迁移方案与分阶段计划，不是当前功能说明。文中带“方案要求”“计划”字样的内容表示目标，不代表已经实现；当前实现范围、已落地接口和未完成项见[实体推断实现状态与差距](/development/solid-inference-status)。
>
> 方案核对基线为 2026-11-22 的 `main` 结构；实现状态页对应当前产品版本。

## 背景与目标

系统原有两条存在交叠的链路：

- **曲面分析链**：`surface/` 完成解析曲面检测、支撑扩展与验证、面片索引、点归属、关系推断与优化；`SurfaceAnalysisResult` 包含 `surfaces`、`patches`、`relations`、支撑验证结果和快照指纹。
- **装配与闭合链**：`parts/assembly/` 重新执行面片提取、分组和有限边界恢复，然后进入壳闭合与各类专用闭合。

方案要消除的是第二条链对第一条链已完成工作的重复，而不是删除 `core/parametric_modeling`。后者仍然负责草图、拉伸、布尔、B-Rep、事务提交和 STEP 导出。

目标：

1. 定义唯一、可验证、带版本的上游到实体推断数据契约。
2. 下游只消费已验证的曲面、有限边界、关系和源点证据，不静默重做 RANSAC、区域生长、点归属或 Alpha Shape。
3. 用“多候选生成 → 统一评分 → 组合搜索 → OCCT 校验 → 特征计划”替代单条启发式规则独占决策。
4. 对未闭合、遮挡或证据矛盾的区域允许部分重建，明确标记推断面与不确定性，并保留人工纠错入口。
5. 分阶段迁移，保留旧链路的隔离诊断与对照能力。

明确不在本轮范围：自由曲面全面恢复、从任意 B-Rep 唯一恢复设计历史、复杂装配接触关系的自动确定、修改建模核心的文档状态管理方式。

## 目标架构与模块权责

```text
PointStore / ObservationSnapshot
             │
             ▼
[上游 Surface Analysis —— 唯一真源]
检测/重拟合 → 有限支撑验证 → 面片索引/点归属 → 关系优化
             │
     生成 SurfaceScene v1：方程 + 有限域/轮廓 + 关系 + 证据/可信度
             │
             ▼
[下游 Solid Inference]
契约检查 → 几何/拓扑图 → 多策略候选 → 统一评分/组合搜索
             │
     SolidCandidate / FeatureProgram / UnresolvedRegions
             │
             ▼
[Parametric Modeling —— 保持现有职责]
建模适配器 → ModelPatch → ModelingCore
草图/拉伸/布尔/边界体 → B-Rep 验证 → .pcad / STEP
```

| 职责 | 唯一责任模块 | 下游是否允许重做 |
| --- | --- | --- |
| 点云预处理、法向估计、解析曲面检测与重拟合 | `preprocessing/`、`segmentation/`、`surface/` | 禁止 |
| 有限面域、孔边界、分离岛、支撑与独占归属 | `surface/` | 禁止；只能消费或声明不够用 |
| 平行、共面、垂直、共轴等关系拟合与优化 | `surface/` | 禁止；可派生候选拓扑关系 |
| 面组合、部件归属假设、缺失壁/盖面推断 | `solid_inference/` | 负责 |
| 拉伸、切除、壳闭合、原始候选提案 | `parts/assembly`、`parts/projection` 的可复用算法 | 负责，但必须消费统一输入 |
| 候选合法性、观测评分、组合排序与诊断 | `solid_inference/` | 负责 |
| 特征历史、草图/布尔、OpenCASCADE、原子提交、STEP | `core/parametric_modeling` | 保留 |

关键区分：`Adjacent` 是上游的“邻近观测”关系，不代表两个面必然属于同一零件；“共同围成一个实体”“孔/凸台/切除”“共享拓扑边”属于下游假设。

## SurfaceScene v1 数据契约

### 交付原则

- 不直接把 `SurfaceAnalysisResult` 的内部结构当作长期协议。`SurfacePatch` 目前只有曲面引用、支持点、质心/范围与材料法向，不直接携带完整闭合轮廓环。
- 在 `surface/` 侧增加明确的 `SurfaceScene` 导出器，把支撑决定与已验证面片关联，转换为有限面的拓扑边界。下游不得再执行一次 Alpha Shape 补齐协议。
- 支持共面多片、每片多个边界环、圆柱等非平面解析曲面；不能把“没有观测到孔内壁”当作已证实通孔。
- 交付的是观测事实。推断闭合形成的边/面需要另存于下游候选，不得写回覆盖真实边界。

### 建议类型

```cpp
struct SceneHeader {
    uint32_t schemaVersion = 1;
    LengthUnit unit = LengthUnit::Millimeter;
    uint64_t analysisFingerprint = 0;
    uint64_t sourceFingerprint = 0;
    uint64_t settingsFingerprint = 0;
    double distanceToleranceMm = 0;
    double angularToleranceRad = 0;
};

enum class BoundaryStatus { ObservedClosed, ObservedOpen, Ambiguous };
enum class LoopRole { Outer, Hole, Island, Unknown };

struct BoundaryLoop3d {
    LoopRole role = LoopRole::Unknown;
    std::vector<Vec3> vertices;
    bool closed = false;
    double confidence = 0;
};

struct ScenePatch {
    PatchId id = 0;
    SurfaceId surfaceId = 0;
    PrimitiveKind kind;
    std::vector<PointId> supportPointIds;
    Vec3 centroid, materialNormal;
    bool orientationKnown = false;
    BoundaryStatus boundaryStatus = BoundaryStatus::Ambiguous;
    std::vector<BoundaryLoop3d> loops;
    double supportConfidence = 0;
    double residualMm = 0;
    double spacingMm = 0;
};

struct SceneRelation {
    PatchId first = 0, second = 0;
    SurfaceRelationKind kind;
    double error = 0, confidence = 0;
    bool manualConfirmed = false;
};

struct SurfaceScene {
    SceneHeader header;
    std::vector<PrimitiveEvidence> surfaces;
    std::vector<ScenePatch> patches;
    std::vector<SceneRelation> relations;
    std::vector<PointId> unassignedPointIds;
};
```

上述类型是接口草案，实际实现应使用只读引用或不可变值避免重复复制大点云；若需缓存或序列化，应使用值类型与稳定 ID，不能把裸指针写入磁盘。

### 坐标与标识规则

1. 坐标统一为右手系、毫米；距离容差单位为毫米，角容差单位为弧度，下游不得自行缩放。
2. `PointId` 保持上游源行号语义；`SurfaceId`/`PatchId` 只在同一个分析快照内有效，下游引用面片时必须绑定 `analysisFingerprint`。
3. `analysisFingerprint` 必须对应用所使用的工作点与修复版本；`sourceFingerprint` 绑定原始观测。修复合并时显式处理基线指纹与修复源指纹。
4. 同一解析平面的多个不连通片段保持“一个 `SurfaceId`、多个 `PatchId`”，不能为闭合自动合并被上游判为分离的区域。
5. 面方程方向与材料外向法向分别声明；方向未知时不得强行推断材料内外，可生成候选但降级为歧义。
6. `BoundaryLoop3d` 统一规定闭合性、环朝向、外环/孔/分离岛语义；上游无法确定孔/岛归属时使用 `Unknown`，禁止隐式当作孔。
7. 关系端点必须引用存在的面片；不存在、快照不匹配、轮廓无效等应报可定位的契约错误，不得静默重跑检测。
8. 工作点用于快速生成候选，原始点用于最终验证；不可混用计数分母或假设采样密度相同。

### 上游导出要求

现有支撑决定包含三角有限域、孔和覆盖点，但 `SurfacePatch` 没有直接引用它；支撑验证还可能在 不同采样阶段生成。方案要求：

- 建立有依据的“面片 ID ↔ 支撑决定 ↔ 有限域”显式关联；
- 优先从已验证三角域提取边界边、组装环，并复用已识别的孔与岛；
- 不能按下标一一对应，不能因为曲面 ID 一致就把全部域视作一个面片；
- 优先使用同源全分辨率验证结果，但必须检查源指纹、支持点和坐标系匹配；
- 匹配失败时报告 `MissingBoundary` 或 `MappingAmbiguous`，不得擅自重新执行 Alpha Shape；
- 圆柱、圆锥等曲面保留解析方程与实际观测覆盖；缺少可靠 trim 环时允许 `ObservedOpen`。

### 错误策略

建议 `validateSurfaceScene()` 覆盖 `WrongUnit`、`SnapshotMismatch`、`InvalidSurfaceReference`、`MissingBoundary`、`InvalidLoop`、`UnknownOrientation`、`MappingAmbiguous`、`UnsupportedSurfaceDomain`。

单片数据不足时该片降级为开放/歧义并进入未解决区域；整个快照、单位或 ID 破坏契约时整轮拒绝。校验函数不得修改上游对象。

## 下游算法

### 只读 Surface Graph

- 节点是 `ScenePatch`，引用解析方程、有限域、观测支持和可信度。
- 边是已有几何关系，加上下游独立建立的“可能共享交线/可能是相对端盖/可能属于同一特征”假设边。
- 用真实轮廓、平面交线、边界投影重叠生成候选，不修改原始关系图。
- 输出一个或多个 `PartHypothesis`；接近或接触不等同于同一实体，允许多体与竞争分组。

### 候选生成器矩阵

| 生成器 | 输入证据 | 输出假设 | 必须防范 |
| --- | --- | --- | --- |
| `ObservedShellClosure` | 已观测闭合边界、共享边/顶点 | 直接闭合 B-Rep / 边界体 | 开边、非流形、法向未知 |
| `ParallelCapExtrusion` | 一对平行端面、投影轮廓匹配 | 草图 + 定深拉伸 | 平行但属于不同部件、深度猜测 |
| `PlaneJunctionClosure` | 多平面交线和有限边界 | 缺失公共边/面候选 | 误将无限平面交线作为真实边 |
| `SeedPlaneExtrusion` | 高质量主面 + 深度层集合 | 多种拉伸起始/结束位置 | 只按面积选取、连续盲搜 |
| `HoleAndRecess` | 内环、同轴圆柱壁、孔底/第二开口 | 盲孔、通孔、凹槽 Cut | “平面有孔”不等于孔已贯通 |
| `AttachedBoss` | 主体面、局部侧壁与顶面 | Join / 凸台 | 内外法向、宿主归属不确定 |
| `AnalyticCurvedSolid` | 圆柱/球/圆锥/圆环及端面 | 解析实体或与主体布尔 | 仅见一部分曲面时强行补全 |
| `PlanarCellInference`（后期） | 多个已验证支撑平面、材料方向 | 材料单元选择、壳闭合 | 单元数爆炸、无法表达曲面 |

拉伸深度策略：从其他平行面、关联边界、几何层次和包围盒中提出离散步长，每个主平面保留 Top-K；不以“体积随深度变化 + 内部点数量比例”作为唯一判据。遮挡造成的缺盖面应标记 `inferredCap=true` 与置信度。

孔洞策略：两个对应孔边界或“开口 + 孔底”只产生 `Cut` 假设；必须先有被证实或已选中的宿主材料体才能提交。通孔需要两端开口或可靠内壁与材料判据；盲孔需要开口、孔底和深度支持。

### 统一候选与组合搜索

```cpp
struct SolidCandidate {
    uint64_t analysisFingerprint = 0;
    std::vector<PatchId> supportingPatches;
    std::vector<PointId> supportingPoints;
    std::vector<PatchId> unresolvedPatches;
    std::vector<FeatureOperation> featureProgram; // NewBody/Join/Cut/Intersect
    std::optional<BoundaryBody> fallbackBoundary;
    std::vector<InferredSurfaceRecord> inferredSurfaces;
    CandidateScore score;
    CandidateStatus status; // Proposed / Accepted / Ambiguous / Rejected
    std::vector<Diagnostic> diagnostics;
};
```

生成器只提案，不得自行发布最终模型。搜索状态包括当前材料体、可切除宿主、已解释观测面、冲突关系及累计操作复杂度。可沿用现有投影搜索的 Beam Search 机制，但把候选来源从单一投影操作扩展到多策略 `FeatureOperation`，并通过共享评估器判断。候选按被解释面/材料体区域去重，防止多个路径重复计分。

### 统一评分

建议初版损失：

$$
E(M)=w_dE_{distance}+w_nE_{normal}+w_oE_{occupancy}
+w_uE_{unsupported}+w_rE_{relation}+w_cE_{complexity}
$$

- **距离项**：点到模型 B-Rep 外表面的稳健距离（median/P95/截断均值），不能只算全部点的均值。
- **法向项**：观测方向已知时检查实体外法向；未知时单独记不可判定。
- **占据冲突**：观测点明显深入候选实心区域，或与可靠相机自由空间射线冲突时强惩罚；边界附近噪声不算冲突。
- **未支持面项**：区分“可见却无点”的矛盾与“背面未观测”的不确定；对推断面分别计算面积和风险。
- **关系项**：模型是否保留上游支持的平行/正交/共轴与端点边界约束。
- **复杂度项**：不必要的特征数、额外切除、几何自由度与碎片化体数。

硬校验优先于总分：OCCT 建体成功、实体有效、非退化、法向/拓扑闭合正确、布尔结果合法。使用统一的局部点采样快速筛选，最终对原始全量观测独立复核。`BRepCheck_Analyzer` 只验证 CAD 拓扑几何有效性，不负责证据真实性。

## 冗余代码清理与保留清单

| 位置 / 现有逻辑 | 决策 | 迁移后责任 |
| --- | --- | --- |
| `surface/SurfaceAnalysis.cpp`、`SurfacePatches.cpp`、`SurfaceOwnership.cpp`、`SurfaceRelations.cpp` | 保留并成为唯一真源 | 产出可靠的方程、分片、关系、归属 |
| `surface/SurfaceFootprint.cpp`、`SurfaceSupportValidation.cpp` | 保留/扩展 | 产出可对外使用的有限域与边界语义 |
| `AssemblyReconstruction.cpp::extractPatches()` | 从新路径移除调用 | 不再重新估法向、区域生长、拟合或合并共面碎片 |
| `AssemblyReconstruction.cpp::groupPatches()` | 替换 | 使用场景面片与关系派生竞争部件假设 |
| `FiniteBoundaries.cpp::recoverBoundaries()` | 从新路径移除调用 | 使用上游边界；下游只推断缺失连接/修补候选 |
| `assignSourceObservations()` | 从新路径移除调用 | 上游提供支持点归属；下游维护候选解释与残余点 |
| `parts/assembly/PartClosure.cpp` | 保留并改为消费场景 | 共享边、壳封闭、B-Rep 试建与材料冲突检查 |
| `PlanarContourRecovery.cpp`、`AnalyticProfileClosure.cpp`、`AnnularTubeClosure.cpp` | 保留为候选生成器，拆除重复预处理 | 生成拉伸/盲孔/曲面假设，不直接宣布最优结果 |
| `parts/projection/*` | 保留评分/搜索思想，抽象公共接口 | 不重新推导已提供的轴/面证据 |
| `recognition/ParametricSolidRecognizer.cpp` | 逐步统一入口 | 旧箱体/曲面识别成为候选生成器 |
| `core/parametric_modeling/*`、`ModelingAdapter.cpp` | 保留 | 特征历史、OCCT、事务提交；扩展候选到补丁的映射 |
| `apps/ParamCAD Studio/src/app/CadReconstructionTask.cpp` | 重接流水线 | 所有自动路径先准备同一场景，再做实体推断 |

保留独立检测的唯一合法情况：离线 legacy 基准/回归模式，或用户明确发起新的上游局部重分析。它必须产生新的上游快照与 ID 映射后重新进入下游；下游不能内部偷偷重算并覆盖原场景。

## 阶段计划与验收

### P0 数据契约落地

- 新增 `SurfaceScene` 类型、验证器和场景导出器。
- 上游显式导出已验证边界、孔、分离岛，建立面片与有限域的映射；没有真实数据时输出开放或歧义。
- 对法向、毫米、指纹、跨快照引用、缺失支持点做契约校验。
- 验收：同一测试点云只运行一次上游检测；新接口不调用 `extractPatches`、`recoverBoundaries`、`assignSourceObservations`；同一分析快照重复导出等价；所有 ID 可追溯。

### P1 多候选主路径

- 新增 `SolidInference.h` 与候选生成器，把观测壳闭合、平行端面拉伸、面交线、孔/孔底匹配、解析曲面识别改为候选生成器。
- 不修改上游场景；每个候选记录支持面、推断面、深度来源、开放/歧义原因。
- 验收：箱体、两级台阶、L 形体、通孔、盲孔有可追踪候选；不匹配的平行面组合不会直接提交；存在未观测端面时结果注明推断。

### P2 共享评分与组合搜索

- 抽象公共的点到 B-Rep 距离/法向接口，补充自由空间/占据冲突、关系、复杂度和推断面惩罚。
- 实施有限 Beam Search；初期可用 3 步/宽 4 作为性能基线，不承诺准确率阈值。
- 使用相同点集评分不同生成器的候选，最终用原始全量点复验。
- 验收：基体加切除、基体加凸台可组合；候选来源不同但比较口径一致；评分与每项误差可导出。

### P3 主流程接入与旧代码隔离

- `CadReconstructionTask.cpp`：分析结果有效时直接转换为场景；没有或失效时由上游统一分析一次。禁止 fallback 重新分割。
- 将建模适配器接入已排序特征程序，生成带 `expectedRevision` 的原子补丁；保护手工编辑特征与现有修订语义。
- 旧装配路径保留在显式 legacy/诊断入口做 A/B 对照，主流程不能自动回退。
- 验收：Qt 与 CLI 使用同一套主要几何证据；失败时保留开放面与诊断、不改模型；重建成功后工程可再编辑，STEP 可导出。

### P4 可选全局空间单元闭合

- 对平面主导部件，按支持平面划分有限空间单元，以已定向面、边界证据和可用自由空间观察决定材料占据；按单元图优化生成候选壳或 CSG 组合。
- 单元数和优化预算必须受限；圆柱孔由解析特征切除，不强制退化成小体素。仅在 P0–P3 测试稳定后考虑。

## 回归测试矩阵

| 用例 | 必查内容 | 失败应如何呈现 |
| --- | --- | --- |
| 六面箱体 | 唯一闭合实体；尺寸/法向正确；简洁特征 | 不生成多个重复体 |
| 缺一个端盖的棱柱 | 推断候选及来源/置信度；不能伪装为已观测面 | 歧义或部分成功 |
| L 形与两级阶梯 | 不被强行拟合成大长方体；可 Join/Cut | 记录未解释的面 |
| 两个平行但不属于一体的面 | 不因平行就连接 | 拒绝配对并记录理由 |
| 通孔、盲孔、无孔底 | 区分贯通/定深/信息不足 | 不凭一个内环猜通孔 |
| 同平面分离岛、内孔、自交环 | 不混淆材料与空洞；拓扑检查 | 契约失败或区域歧义 |
| 圆柱侧壁/端盖 | 解析实体及材料方向 | 无端盖不强行封闭 |
| 部分法向未知/噪声内点 | 稳健阈值，不把全部内部近邻当矛盾 | 有条件接受或保留待确认 |
| 多体接触/相邻 | 不默认为单体；部分重建可用 | 每个体独立状态 |
| 旧快照或毫米不一致 | 显式拒绝，模型修订不变 | 快照错误或单位错误 |
| 自动重建后手工修改 | 语义引用、撤销、重建、STEP | 不破坏用户修改历史 |

建议额外记录：上游运行次数、候选数、被拒原因、有效 B-Rep 比例、观测解释率、P95 距离、占据冲突、推断面面积占比、耗时与峰值内存。不要只用点覆盖率评估：过大的实心块可以靠近部分点，却错误吞没凹槽或孔。

## 完成定义

- 产品主路径只进行一次上游曲面分析、有限边界生成和点归属，下游不私下重复这三类操作。
- 上游输出类型与下游输入类型一一对应；边界缺失能够显式表达，不被无声补造。
- 多个算法复用一个候选接口与一套评分器，候选来源、推断面、冲突和拒绝原因可追溯。
- 至少通过箱体、阶梯、带孔、缺盖面、分离岛、多体与快照失配等回归测试。
- 成功候选经 OpenCASCADE 校验，优先输出草图/拉伸/切除的可编辑特征程序；无法提取合理程序的复杂形状才使用显式边界体回退。
- 不破坏现有特征历史、手工修改保护、事务提交和 STEP 导出。

## 关键源码定位

| 内容 | 路径 |
| --- | --- |
| 上游分析接口 | `core/geometric_reconstruction/include/reconstruction/surface/SurfaceAnalysis.h` |
| 上游有限域 | `core/geometric_reconstruction/include/reconstruction/surface/SurfaceFootprint.h` |
| 上游支撑结果 | `core/geometric_reconstruction/include/reconstruction/surface/SurfaceSupportTypes.h` |
| 上游面片/关系/归属 | `core/geometric_reconstruction/src/surface/SurfacePatches.cpp`、`SurfaceRelations.cpp`、`SurfaceOwnership.cpp` |
| 旧装配提取/分组 | `core/geometric_reconstruction/src/parts/assembly/AssemblyReconstruction.cpp` |
| 旧有限边界 | `core/geometric_reconstruction/src/parts/assembly/FiniteBoundaries.cpp` |
| 旧平行轮廓与孔口匹配 | `core/geometric_reconstruction/src/parts/assembly/PlanarContourRecovery.cpp` |
| 旧闭合 | `core/geometric_reconstruction/src/parts/assembly/PartClosure.cpp` |
| 投影搜索与评分 | `core/geometric_reconstruction/src/parts/projection/ProjectionSearch.cpp`、`ProjectionScoring.cpp` |
| 生产入口 | `apps/ParamCAD Studio/src/app/CadReconstructionTask.cpp` |
| 参数化核心 | `core/parametric_modeling/include/modeling/ModelingCore.h` |
| 候选到建模补丁 | `core/geometric_reconstruction/src/modeling/ModelingAdapter.cpp` |

## 实施决策

先完成 P0、P1、P2，以最小改动让“上游验证后的面、关系、边界”成为唯一事实来源，并让已有的拉伸、切除、闭合算法在统一候选框架下工作；之后再考虑 P4 空间划分。这样比继续向装配路径堆特殊规则或提前重写 OCCT 核心更稳妥。
