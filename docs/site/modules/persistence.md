# 工程保存与 STEP 导出

所属主线：[模块5：参数化建模与交付](/modules/point-cloud-to-cad)。前置步骤是[候选提交与 B-Rep 重建](/modules/parametric-modeling)；本页说明模型生成后如何保存和交付。

## 职责与入口

本页覆盖两条独立的持久化路径：`.pcad` 工程保存与重开，以及当前 B-Rep 的 STEP 导出。工程文件保存可继续编辑的特征历史、点云和显示状态；STEP 只导出几何，不承载编辑历史。

入口：

- `ProjectStore::save(path, snapshot, error)`：保存工程；
- `ProjectStore::load(path, error)`：完整校验并重建，返回独立的 `PreparedProject`；
- `ProjectController::loadFrom()`：在 `load()` 成功后把工程替换到当前工作区；
- `StepModelExporter::write(shape, path, error)`：客户端 STEP 导出适配器；
- `modeling::exportStep()`：核心层的 AP214 序列化。

## 输入与输出

工程保存输入 `ProjectSnapshot`：

- `features`：特征历史，含稳定 ID、名称、抑制状态、草图平面/实体和全部基元参数；
- `currentBody`：当前 B-Rep；`parametricBody` 标记它是否应与特征历史一致；
- `cloud`：原始全量点云、删除掩码、集合、关系、选择、比例元数据；
- `view`：相机、投影、正交比例、原点锁定、对象变换/显示、云可见性/点大小/选择模式、基准面可见性、背景、光照、纹理投影和输出尺寸。

工程保存输出是单文件归档：8 字节 `PCADPRJ1` 签名、32 位大端 JSON 清单长度、紧凑 JSON 清单，随后按清单顺序排列的资产。

工程重开输出 `PreparedProject`：已重建的 `ModelingCore`、独立点云文档、当前 B-Rep、视图状态和纹理。它不接触当前活动状态。

STEP 输入是经过验证的 `TopoDS_Shape` 和输出路径；输出是 STEP 文本文件，失败时返回错误文本并保留原文件。

## 核心数据结构与依赖

| 类型 | 内容 |
| --- | --- |
| `ProjectSnapshot` | 特征列表、当前 B-Rep、点云指针、视图状态、参数化实体标志 |
| `PreparedProject` | 临时资产目录、建模核心、点云文档、当前实体、视图、纹理 |
| `ProjectViewState` | 相机、显示、对象变换、渲染和纹理设置 |
| `FeatureCodec` | 特征历史的 JSON 编解码 |
| `SaveFileBuffer` / `QSaveFile` | 流式写入与原子替换 |

工程清单字段包括 `schema`、`version`、`features`、`parametricBody`、`view`、`cloudName`、`cloudUnit`、`selection` 和 `assets`。资产白名单为 `body.brep`、`cloud.ply`、`annotations.json`、`texture.png`，每项记录文件名、字节数和 SHA-256。

版本规则：

- 普通工程写 `version: 2`；
- 任一拉伸带 `targetBody`（限定目标体的布尔）时写 `version: 3`；
- 任一草图包含三点圆弧（`Arc2D`）时写 `version: 4`，优先于版本 3；
- 任一拉伸切除使用 `throughAll` 时写 `version: 5`，优先于版本 3 和 4；
- 任一草图使用沿宿主面法向的偏移时写 `version: 6`；
- 任一草图按轮廓边 ID 引用拉伸侧面时写 `version: 7`；
- 读取接受 `version` 1 至 7；缺失 `resizeAnchor` 时按“圆柱默认底面、其他默认中心”恢复，缺失 `manuallyEdited` 时按 `false`。旧版本没有圆弧实体，读取时按原有直线、矩形和圆恢复。

依赖：Qt Core（`QSaveFile`、`QDataStream`、`QCryptographicHash`、JSON、图像）、OpenCASCADE（`BRepTools`、`BRepCheck_Analyzer`、布尔材料比较）、建模核心（`restoreFeatures` 重建）、`PlyReader`/`PlyWriter` 和点云标注编解码。

## PCAD 保存恢复、兼容性与 STEP 回读

保存流程：

1. 先在临时 `ModelingCore` 上调用 `restoreFeatures()`，拒绝无法加载的历史，避免覆盖旧工程。
2. 编码特征和视图，并立即解码、解析视图、校验资产格式，做一次自检。
3. 若存在当前实体，检查 B-Rep 有效性；参数化实体还必须与特征重建结果材料一致。
4. 用 `PlyWriter` 写出原始点云，写出标注 JSON，保存纹理为 PNG，并记录选择 ID。
5. 计算每个资产的大小与 SHA-256，清单上限为 16 MiB。
6. 用 `QSaveFile` 写入签名、长度、清单和资产，最后 `commit()`；任一步失败都保留旧文件。

重开流程与保护：

- 先检查签名、长度范围、JSON 可解析、`schema` 匹配和版本受支持。
- 资产名必须在白名单内且不重复；边读边计算 SHA-256，校验大小和字节末尾没有多余内容。
- 特征解码后在独立模型中 `restoreFeatures()`，要求 ID 唯一、类型匹配、引用只指向上游，并立即重建。
- 读取 `body.brep` 后检查非空和 B-Rep 有效性；若为参数化实体，还要与重建结果做双向材料比较。
- 点云必须同时存在 `cloud.ply` 与 `annotations.json`；单位只能是 `mm`、`m` 或 `arbitrary`；纹理解码失败即拒绝。
- 所有解码、校验和重建完成后，`ProjectController` 才替换模型、点云、实体和视图状态；失败时当前工作区保持不变。
- 打开工程期间禁止拍摄或照片重建；替换时取消待处理的 CAD/曲面任务并推进文档修订，旧结果不能覆盖新工程。

STEP 导出：

- 核心设置 `write.step.schema = AP214IS`，用 `STEPControl_AsIs` 转移当前形状。
- 客户端把核心的流式写出适配到 `QSaveFile`，因此支持 Unicode 路径和原子替换；写入失败、流错误或提交失败都会取消替换。
- 导出只包含 CAD 几何。相机/显示变换、纹理、点云标注和特征历史不会写入 STEP；工程持久化是独立操作。
- 扩展名接受 `.step` 与 `.stp`，缺省时补 `.step`；没有实体、路径为空或实体在选文件期间变化时拒绝导出。

STEP 回读由 `StepModelLoader` 完成：使用 Unicode 感知的输入流读取 B-Rep，保留原始几何和尺寸；模型按体积质心（无封闭体时按包围盒中心）平移到原点，文件中的原始放置不被保留。导入实体没有参数化特征历史，只能整体显示、导出或被新工程替换。

## 本地重建缓存

Studio 和原生 `pcad` 共用一份磁盘重建缓存。它不属于 `.pcad` 工程，也不改变点云、集合、关系或特征历史；打开 PLY 时仍只载入点云，只有主动执行表面识别或 CAD 重建时才读取和写入缓存。

- 默认目录为 Qt 缓存目录下的 `reconstruction/v4`；设置 `PCAD_CACHE_DIR` 可把根目录改到独立位置，设置 `PCAD_DISABLE_CACHE=1` 可完全绕过缓存做对照。
- 缓存按阶段保存：`preprocessing`、`surface_evidence`、`working_analysis`、`source_analysis`、`primitive_detection`、`recognition`、`assembly_grouping`、`assembly_preparation`、`assembly_closure`、`assembly_projection` 和 `source_surface_preview`。
- 键绑定实际观测、单位、生效设置、阶段实现和数据格式版本；写入时在文件头记录客户端版本、Git 提交和工作区指纹。单条载荷上限为 512 MiB，校验失败或版本不符都按未命中处理。
- 装配按提取分组、边界准备、闭合和投影分层保存，单独修改投影参数仍可复用前面的结果。CAD 未闭合后用于显示源点验证面片的预览也可复用。
- CAD 报告的 `cache` 对象列出每个阶段的 `hit`、`miss` 或 `bypass` 及原因。命令行可通过 `--report` 或标准输出读取该对象。
- **设置 → 清理点云缓存…** 整体删除缓存，正在识别或重建时拒绝执行。当前没有单条删除、容量上限或自动回收策略。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 工程容器保存与重开 | `apps/ParamCAD Studio/src/io/ProjectStore.cpp` |
| 本地重建缓存 | `apps/ParamCAD Studio/src/io/ReconstructionCache.cpp`、`ReconstructionCacheCodec.h` |
| 工程快照与结果类型 | `apps/ParamCAD Studio/src/io/ProjectStore.h` |
| 特征历史编解码 | `apps/ParamCAD Studio/src/io/FeatureCodec.cpp` |
| 工程动作与失败保护 | `apps/ParamCAD Studio/src/app/ProjectController.cpp` |
| STEP 客户端适配 | `apps/ParamCAD Studio/src/io/StepModelExporter.cpp` |
| 核心 STEP 序列化 | `core/parametric_modeling/src/StepExporter.cpp` |
| 特征历史恢复与重建 | `core/parametric_modeling/src/PartDocument.cpp` |
| PLY 写入 | `apps/ParamCAD Studio/src/io/PlyWriter.cpp` |

## 已知限制

- 工程是整体归档，没有增量保存、压缩或部分恢复；任一清单项或资产校验失败都会拒绝整个工程。
- 工程读取覆盖版本 1 至 7。更高版本会被直接拒绝。
- 重建缓存不进入工程，也不随工程复制；更换构建、设置或点云后旧条目只能按未命中处理，不会自动迁移。
- 撤销历史、待处理草图手势和拍摄镜头列表不进入工程，重开后只能从已提交状态继续。
- 纹理在保存时重新编码为 PNG；若原始纹理格式或色彩管理重要，工程内保存的副本不等同于源文件。
- `body.brep` 是 OpenCASCADE 二进制实体，不是可读交换格式；跨版本 OCCT 兼容性依赖 B-Rep 读写实现。
- STEP 只导出几何，不包含特征历史、纹理、点云标注或显示变换；需要继续编辑时应保存 `.pcad`。
