# 数据契约与坐标约定

本页列出跨模块传递时必须成立的条件：单位如何归一、点 ID 如何保持稳定、各类 ID 的有效范围、以及修订号如何阻止过期结果写回。字段定义以头文件为准，调用顺序以生产任务为准。

## 单位、尺度与坐标系

几何核心只有三种长度单位（`core/geometric_reconstruction/include/reconstruction/data/PointStore.h`）：

| 单位 | `millimetersPerUnit` | 行为 |
| --- | --- | --- |
| `Millimeter` | 1 | 直接使用，工作坐标即毫米 |
| `Meter` | 1000 | 快照阶段统一乘以 1000 转为毫米 |
| `Arbitrary` | `std::nullopt` | 没有隐式换算，数值保持不变，单位标签仍为任意 |

`ObservationSnapshot` 是几何链路中统一尺度的位置，而且只做均匀缩放：已知单位转换后坐标与单位同时变为毫米，点 ID 和拓扑关系不变。`Arbitrary` 不会被悄悄当作毫米；客户端在启动重建前必须让用户明确确认“1 个源单位 = 1 毫米”，确认后以 `Millimeter` 传入，拒绝则整次重建不启动（`apps/ParamCAD Studio/src/app/PointCloudWorkspaceController.cpp`）。

容差的单位随坐标一起确定：

- 配置文件和 `*Options` 中的距离参数按毫米解释，例如 `AssemblyOptions::distanceTolerance` 明确标注为毫米，取 0 时由装配流程从观测中估计局部噪声；
- 关系推断的 `distanceTolerance` 同样以工作坐标（毫米）计量；
- 用户在关系面板输入的点云容差会先乘以预处理报告的 `appliedScaleToMillimeters`，再作为平面和圆柱检测阈值；
- `arbitrary` 点云在用户确认后按 1:1 处理，因此其容差也按“确认后的毫米”理解，而不是物理毫米；
- 跨模块接口中的长度值不携带单位标签，调用方必须按上述约定解释，几何核心不会做第二次隐式换算。

坐标系约定为右手系。COLMAP 点云位于其重建世界坐标系，记录在 `reconstruction.json`；结构光点云位于采集世界坐标系，单位为毫米，记录在 `result.json`。两者不能在未校验尺度和世界变换时直接混合。相机局部坐标为 +X 向右、+Y 向下、+Z 向前；`CameraInfoBuilder` 在写出外参前完成相机基到世界基的转换，结构光以 `rig_to_world × device_to_rig` 得到设备位姿。STEP 回读会把形状平移到质心位于原点（无封闭体积时用包围盒中心），只改变摆放位置，不改变距离与尺寸。

## 观测快照与源点 ID

`PointId` 是 `std::uint64_t`，0 为无效值。生产链路中的 ID 规则是：

- 客户端 `PointId = 原文档行号 + 1`，由 `PointCloudAdapter::toPointStore` 通过 `PlyCloud::sourceRows` 还原（`apps/ParamCAD Studio/src/io/PointCloudAdapter.cpp`）；
- 跳过非有限坐标、删除活动点、降采样与离群过滤都不会给剩余点重新编号，因此同一源点在所有后续阶段保持同一 ID；
- 模板选区点会加入预处理保护列表，保证过滤后仍可按原 ID 检索；
- `PointStore` 构造时强制 ID 非零且唯一，并拒绝非有限坐标、越界置信度与负曲率。

`ObservationSnapshot` 保存的是全分辨率观测，不做降采样；工作点是从中抽取、经预处理后仍保留原 ID 的子集。两者的关系靠 ID 维持，而不是靠数组下标或临时标签。

观测指纹 `pointStoreFingerprint` 是一个 64 位混合哈希，输入包括单位、每个点的 ID、位置、是否存在法向、方向是否可信、法向分量与置信度（`core/geometric_reconstruction/src/data/PointStore.cpp`）。分析结果同时保存 `pointFingerprint` 与 `unit`，复用缓存分析前必须通过 `analysisInputMatches`（单位与指纹一致）和 `analysisSettingsMatch`（检测与关系设置一致），否则重新分析。删除点、重新载入或改变单位都会改变指纹，旧分析因此不再满足复用条件。生产任务另外记录 `skippedInvalidObservations`，用于说明 PLY 中有多少行因坐标非有限被丢弃。

## 证据、部件与特征 ID

`SurfaceId` 是 `std::uint64_t`，0 为无效值，随 `PrimitiveEvidence` 一起传递。`indexSurfacePatches` 在建立面片索引时会把证据 ID 从头重编号为 1..N，因此**证据 ID 与面片 ID 的有效范围都是当前分析快照**，不能跨快照引用（`core/geometric_reconstruction/src/surface/SurfacePatches.cpp`）。

| ID | 类型 | 分配方式 | 有效范围 |
| --- | --- | --- | --- |
| `PointId` | `uint64_t` | 源行号 + 1 | 整个点云生命周期，编辑与预处理后不变 |
| `SurfaceId` | `uint64_t` | 索引时按顺序重编号 | 一个分析快照 |
| `PatchId` | `uint64_t` | 面片创建顺序，从 1 开始 | 一个分析快照 |
| 装配面片 / 部件 ID | `size_t` | 本次分解内顺序分配 | 一次装配结果 |
| `FeatureId` | `uint64_t` | `PartDocument` 单调分配，从 1 开始 | 一个建模文档 |

面片保存 `surfaceId` 与支持点 ID；关系只保存两个 `PatchId` 和关系类型、误差、置信度与是否为自动推断。索引时会检查每个支持点是否存在于当前快照，缺失即返回 `Surface support references a different point snapshot`。人工关系以点 ID 描述端点，进入识别前会先用当前观测面片校验，校验不通过不会静默改变几何。

装配阶段的 `FiniteSurfacePatch::id` 与 `AssemblyPart::id` 属于本次分解，不能当作 `FeatureId` 使用。每个部件同时保存 `workingPointIds`（装配采样归属）和 `pointIds`（全量观测回填归属），两组 ID 独立；候选实体只包含通过几何与 B-Rep 验证的部件，仍不代表原始装配实例。

`FeatureId` 与 `SketchEntityId` 由 `PartDocument` 分配，0 表示无效（`core/parametric_modeling/include/modeling/Id.h`）。建模计划在模型私有副本上提前分配草图和依赖 ID，`ModelingCore::apply` 成功后返回受影响特征列表；调用方不应自行拼接或假设特征 ID。修订检查贯穿计划与应用：计划阶段比对预期修订，应用阶段再次比对，任一不符即拒绝。

## SurfaceScene 契约

实体推断不直接读取 `SurfaceAnalysisResult` 的内部结构，而是消费 `SurfaceScene` v1。类型定义在 `core/geometric_reconstruction/include/reconstruction/solid_inference/SurfaceScene.h`，导出与校验在 `SurfaceSceneAdapter.cpp`。

### 场景头

| 字段 | 约束 |
| --- | --- |
| `schemaVersion` | 当前为 1；其他值拒绝 |
| `unit` | 必须是 `Millimeter` |
| `analysisFingerprint` | 必须对应本次使用的分析快照，不能为零 |
| `sourceFingerprint` | 绑定原始全量观测；没有源快照时取源支撑验证指纹 |
| `settingsFingerprint` | 由检测、关系与支撑验证参数规范化生成 |
| `distanceToleranceMm` / `angularToleranceRad` | 有限且为正 |

### 场景面片与关系

| 字段 | 约束 |
| --- | --- |
| `ScenePatch::id` | 非零、场景内唯一；与上游 `PatchId` 同义 |
| `surfaceId` | 必须指向 `scene.surfaces` 中的有效元素 |
| `supportPointIds` | 非零、唯一，且属于当前分析快照 |
| `boundaryStatus` | `ObservedClosed` 时 `loops` 不能为空 |
| `loops` | 闭合环至少 3 点，开放环至少 2 点；坐标必须有限 |
| `SceneRelation` | 两端引用存在的面片，端点不同，误差与置信度有限 |

边界环角色为 `Outer`、`Hole`、`Island` 或 `Unknown`。当前导出器只有“一个外环 + 若干孔”的简单域标记为 `ObservedClosed`；出现分离岛、多个外环或无法组环时保持 `Ambiguous`，不会在下游重新提取轮廓。

### 错误码与处理策略

| 错误码 | 含义 |
| --- | --- |
| `WrongUnit` | 场景不是毫米单位 |
| `SnapshotMismatch` | schema、分析指纹或容差无效 |
| `InvalidSurfaceReference` | 面片 ID、曲面引用、支撑点或关系端点无效 |
| `MissingBoundary` | 标记闭合但没有边界环 |
| `InvalidLoop` | 环开放、点数不足、坐标非有限或置信度越界 |
| `MappingAmbiguous` | 面片支持点不属于当前分析快照 |

单片边界不足时，该片降级为开放或歧义；快照、单位或 ID 破坏契约时整轮拒绝。校验函数不修改上游对象。

### 快照与观测口径

- 场景只引用同一分析快照中的 ID；跨快照复用会在导出或推断入口被拒绝。
- 工作点用于生成候选和快速筛选；原始全量观测用于最终评分复核。两者不能混用计数分母。
- 当前实现按值保存场景与曲面证据，包括支持点 ID；大点云会带来额外内存与复制开销。

## 修订与数据归属

客户端在启动后台任务前记录三类修订与源路径；结果交接时逐项比对三类修订，源路径主要用于诊断和投影调试归属。

| 状态 | 类型 | 递增或变化时机 |
| --- | --- | --- |
| 点云修订 | `quint64` | 载入点云、接管工程、每次编辑检查点（无论是否记录撤销） |
| 模型修订 | `ModelRevision`（`uint64_t`） | 命令成功执行、补丁成功应用、特征还原或接管工程 |
| 视口实体修订 | `quint64` | 设置或清空显示实体时 |
| 源文件路径 | `QString` | 载入点云或接管工程时 |

结果交接时若点云、模型或视口实体修订任一变化，本轮候选被丢弃并提示重新重建；取消、预处理失败和识别失败同样不改变模型。模型补丁失败时 `ModelingCore` 恢复执行前的文档副本，修订不递增，因此界面看到的模型与修订保持一致。

数据归属分为两种口径：

- **工作点归属**用于实体推断与模板拟合：预处理后的工作点携带原 ID，检测、面片索引、场景导出和候选生成都在它上面计算；
- **全量观测归属**用于最终评分与 legacy 装配：实体推断最终用原始全量观测复核候选；装配从全量观测重新采样出工作面片并分组，再按源点 ID 回填；投影拟合按部件点 ID 取全量观测。

实体推断把工作点分析导出为场景，场景中的面片支持点仍引用原 ID。最终评分时，若存在源观测则使用源观测，否则使用工作点；评分入口会校验观测指纹与场景源指纹一致。局部恢复只从全量快照中取失败区域及容差 halo 内的真实样本，返回补点 ID 与源指纹；合并后的分析另外记录基线指纹、修复源指纹、补充点以及局部到全局的曲面与面片映射。装配闭合若改写轮廓，`boundaryBeforeClosure` 保留闭合前的观测边界，`loops` 才是闭合处理后的版本，避免用修订后的轮廓反向评估观测质量。

修订校验分布在链路的多个位置，而不是只做一次：

| 校验位置 | 比较对象 | 不一致时的行为 |
| --- | --- | --- |
| 启动后台任务时 | 记录点云、模型、视口修订与源路径 | 仅建立比较基线 |
| 尺度确认返回后 | 三类修订是否仍等于启动值 | 放弃启动，不创建任务 |
| 建模计划生成 | 预期模型修订与当前模型修订 | 拒绝生成补丁并返回原因 |
| 补丁应用 | `ModelPatch::expectedRevision` 与当前修订 | 文档保持原状，修订不递增 |
| 结果交接 | 三类修订是否仍等于启动值 | 丢弃整轮结果并提示重新重建 |

因此，跨阶段引用必须同时携带“哪个快照”和“哪个 ID”：只带 ID 而不校验指纹或修订，是当前代码明确拒绝的用法。
