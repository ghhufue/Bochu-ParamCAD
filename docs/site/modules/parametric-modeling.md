# 参数化建模与 B-Rep

## 职责与入口

本模块是 Qt/VTK/JSON 无关的参数化建模核心。持久状态是“特征历史”：由 POD 参数组成的有序特征列表；OpenCASCADE 的 B-Rep 只是由特征历史派生出的当前结果，不在特征之间保存面编号或网格。

对外入口集中在 `ModelingCore`：

- `execute(ModelCommand)`：执行单条命令，成功后递增修订；
- `apply(ModelPatch)`：按预期修订校验后批量执行命令，并可选重建；
- `rebuild()`：从当前特征历史重新构造 B-Rep；
- `features()`、`bodyShape()`、`revision()`、`lastError()`：只读访问；
- `restoreFeatures()`：导入经过校验的特征历史并立即重建；
- `adoptPrepared()`：接管一个已经独立验证过的模型，并使待处理任务失效。

自动重建通过 `reconstruction::commitCompoundCandidate()` 生成建模补丁；客户端通过 `ModelingController` 提交编辑命令。

## 输入与输出

输入：

- `FeatureParams` 变体：草图、拉伸、切除、长方体、贯穿孔、球、圆柱、圆锥、圆环、轮廓体；
- `ModelCommand`：新增/编辑/删除特征、增删改草图实体、设置抑制；
- `ModelPatch`：`expectedRevision`、有序命令和是否重建；
- 草图平面引用：基准面、偏移基准面、语义面或显式世界平面。

输出：

- `ModelResult`：成功标志、受影响特征 ID、草图实体 ID 和错误文本；
- `ModelPatchResult`：成功标志、新修订、受影响特征 ID 和错误文本；
- 重建后的 `TopoDS_Shape`，以及每个特征的 `valid`、`errorText` 状态。

单位统一为毫米，坐标系为模型坐标；Object 显示变换不属于建模数据。

## 核心数据结构与依赖

| 类型 | 关键内容 |
| --- | --- |
| `Feature` | `id`、`name`、`type`、`params`、`suppressed`、`valid`、`manuallyEdited`、`resizeAnchor` |
| `FeatureParams` | 草图、拉伸、切除与六类直接语义实体的 `std::variant` |
| `BoxPrimitiveParams` | 位姿、`sizeX/Y/Z`、`join` |
| `ThroughHolePrimitiveParams` | 世界坐标轴点、轴向、半径；没有深度，由当前实体范围推导安全跨度 |
| `CurvedPrimitiveParams` | 球心/半径、圆柱底面/轴/半径/高度、圆锥双半径、圆环主/管半径 |
| `FaceReference` | 所属特征、`FaceRole`、法向/质心/面积、瞬态面索引提示 |
| `ModelPatch` | 预期修订、命令列表、是否重建 |

`FaceRole` 定义语义面：拉伸的 `StartFace`/`EndFace`；长方体的 `BoxXMin` 至 `BoxZMax` 六个面；圆柱的 `CylinderBase` 与 `CylinderTop`。

依赖：OpenCASCADE 8.0.1 负责草图构面、拉伸、布尔、曲面实体和 STEP；Eigen 仅出现在上层识别，不在建模核心内；核心不依赖 Qt、VTK 或 JSON 库。

## 草图、布尔运算、依赖重建与原子提交

草图与拉伸：

- 草图实体包括点、线、矩形、圆；参考点不参与轮廓构造。
- 轮廓支持无序闭合线段环，以及一个外环加若干内孔；未闭合草图是合法持久状态，`validateSketch()` 报告当前是否可拉伸或切除。
- 拉伸支持 `NewBody`、`Join`、`Cut`、`Intersect`；多个 `NewBody` 在 `bodyShape()` 中返回复合体。
- `targetBody` 把布尔限制到指定拉伸产生的实体；目标必须已存在且仍在当前模型中，运算后必须仍是单一有效实体。

基准面与语义面：

- 支持 `XY/YZ/XZ` 基准面、偏移基准面、显式 `Plane3d` 和语义面引用。
- 语义面从所属特征的当前参数解析，不依赖布尔结果的瞬态面编号；上游尺寸改变后，依赖草图和拉伸/切除会一起重定位。
- 显式 `Plane3d` 是世界坐标快照，不跟随上游面变化。
- 选择面时，`identifyPrimitiveFace()` 只在“恰好一个受支持基元面包含拾取点”时返回引用；共面归属歧义或导入 STEP 面保持为固定平面。

依赖与删除：

- `dependsOn` 覆盖三种边：草图的面引用指向上游特征、拉伸的 `sketchId` 与 `targetBody`、切除的 `sketchId`。
- `dependentsOf(id, false)` 返回直接依赖；`dependentsOf(id, true)` 返回传递闭包。
- `removeFeature(id, false)` 在存在直接依赖时拒绝删除；`cascade=true` 时递归删除所有下游特征。
- 被抑制特征在重建时跳过，但其引用关系仍参与导入校验。

原子提交：

- `apply()` 先比较 `expectedRevision`；不匹配时返回当前修订和“Model revision mismatch”。
- 任一命令失败，或 `rebuild=true` 时重建失败，文档恢复到调用前的副本，修订不前进。
- 只有命令列表非空且整体成功时才递增修订；受影响特征 ID 按命令顺序返回。
- `restoreFeatures()` 在独立副本上校验 ID 唯一、类型与参数匹配、固定参照合法、引用只指向上游，并在重建成功后才替换当前文档。
- `StepExporter::exportStep()` 使用 `AP214IS` 写出 B-Rep；核心路径接受文件路径或 `std::ostream`，后者供客户端实现 Unicode 路径和原子替换。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 建模入口与修订 | `core/parametric_modeling/include/modeling/ModelingCore.h` |
| 特征类型与参数 | `core/parametric_modeling/include/modeling/Feature.h`、`FeatureParams.h` |
| 直接语义基元 | `core/parametric_modeling/include/modeling/PrimitiveParams.h` |
| 草图实体 | `core/parametric_modeling/include/modeling/Sketch.h` |
| 语义面与草图平面解析 | `core/parametric_modeling/src/SemanticFaces.cpp` |
| 文档、依赖与删除 | `core/parametric_modeling/src/PartDocument.cpp` |
| 重建与布尔运算 | `core/parametric_modeling/src/RebuildEngine.cpp` |
| 原子补丁 | `core/parametric_modeling/src/ModelingCore.cpp` |

## 测试与诊断

## 已知限制

- 任意布尔结果面的稳定命名仍未实现。语义面指向源特征的支持平面，不保证该平面在下游布尔后仍有对应有界面。
- 显式 `Plane3d` 是世界坐标快照，不跟随上游特征；修改上游尺寸后，基于该平面的草图不会自动重定位。
- 未闭合草图可以保存，但消费它的拉伸或切除会失败并记录错误；核心不会自动封闭轮廓。
- 一个 `ModelPatch` 内的命令整体原子，但补丁之间没有自动合并；调用方必须提供正确的 `expectedRevision` 才能避免覆盖较新模型。
- 贯穿孔没有深度参数，跨度由当前实体范围推导；当宿主范围变化时孔会自动贯穿新的范围。
- STEP 导出只写出当前 B-Rep 几何，不包含特征历史、草图、显示变换、纹理或标注。
