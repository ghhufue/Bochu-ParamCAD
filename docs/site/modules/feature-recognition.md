# 实体识别与人工引导

## 职责与入口

本模块把曲面证据、有限面片和关系转换为可提交的 CAD 候选，并接受用户模板与人工关系作为额外约束。它对 Qt/VTK 无依赖，输入是预处理后的工作点云，输出仍是建模提案，而不是最终的 B-Rep。

生产流程的统一入口是 `reconstructGuidedSolid()`。模板列表为空时，它复用或新建曲面分析，然后调用 `recognizeAndRepairParametricSolid()` 完成常规识别与修复；模板列表非空时，先逐选区拟合模板，再用剩余观测识别其余材料，最后对每个模板做整件边界复验。另有两个直接入口：`recognizeParametricSolidFromAnalysis()` 供阶段测试直接指定分析快照，`reconstructParametricSolid()` 供一次性“检测加识别”的调用方使用。

## 输入与输出

输入：

- `PointStore`：毫米制工作点，`PointId` 非零且唯一；实体识别依赖方向确定的法向，方向未知时拒绝自动建体；
- `PlaneDetectionOptions` 与 `CylinderDetectionOptions`：距离、支持点、随机种子和平面/圆柱上限；
- `std::vector<TemplateHint>`：选区编号、模板类型与选区源点 ID；
- `std::vector<SurfaceRelationHint>`：两组源点 ID 与关系类型；
- 可选的共享 `SurfaceAnalysisResult`、全量观测快照和取消回调。

输出：

- `CompoundCandidate`：长方体、棱柱、贯穿孔、独立曲面实体、轮廓体与投影程序，附 `surfaceCoverage`、`confidence`、`gridCells`、`searchNodes` 和 `minimumBoxCountProven`；
- `SurfaceAnalysisResult`：本次识别实际使用的分析快照，可能包含修复后的补充点；
- `TemplateFit` 列表与 `failedRegionId`：每个选区的拟合结果，以及失败选区编号。

`surfaceCoverage` 是“被候选实体解释的观测比例”，不是源模型完整度；`confidence` 由各识别器分别定义，不能跨特征类型直接比较。

## 核心数据结构与依赖

| 类型 | 关键内容 |
| --- | --- |
| `CompoundCandidate` | `boxes`、`prisms`、`throughHoles`、`curvedSolids`、`profileBodies`、`projectionPrograms` |
| `BoxCandidate` | 语义 `BoxPrimitiveParams`、六个源平面、置信度 |
| `ThroughHoleCandidate` | 孔轴、半径、源圆柱证据、宿主厚度、置信度 |
| `CurvedSolidCandidate` | 球/圆柱/圆锥/圆环参数、表面覆盖率、边界覆盖率、平均残差 |
| `TemplateHint` | `regionId`、`TemplateKind`（平面/圆柱/球/圆锥/圆环/长方体）与选区点 |
| `SurfaceRelationHint` | 两组点 ID 与 `SurfaceRelationKind` |

识别器按固定顺序竞争：先尝试混合特征（薄矩形宿主、附着体、凸台、圆形或矩形通槽）；若混合结果含曲面实体、贯穿孔或切除，直接采用；否则尝试组合体（最少箱体覆盖加倾斜棱柱）；组合体失败时回退混合候选；再尝试“单箱体加贯穿孔”；最后尝试独立曲面实体。该顺序由 `recognizeParametricSolidFromAnalysis()` 实现，错误文本会依次拼接各分支的拒绝原因。

主要依赖是项目自有的 `SurfaceAnalysisResult`、`ModelingCore`（模板复验与临时建体）和 OpenCASCADE 的面距离查询；不依赖 Qt、VTK 或 PCL。

## 常规识别、模板约束与候选复验

常规识别的接受条件：

- 箱体拼装函数要求恰好六个平面候选、三组互相平行的正交面对，且尺寸为正；拒绝原因包括平面数不为六、无法组成正交平行面对和退化箱体。
- 箱体置信度为六面平均证据置信度、被解释点比例和正交关系置信度的乘积，因此缺面或大量未解释观测会显著降低该值。
- 贯穿孔要求圆柱轴与宿主某一轴对齐（默认 5°），角度覆盖不低于 0.70、轴向覆盖不低于 0.85，两端与宿主外表面偏差不超过厚度的 10%，且完全位于横向边界内；短圆柱不会被误判为贯穿孔。
- 组合体要求达到 `minimumSurfaceCoverage` 0.90，并记录“最少箱体数是否已证明”；搜索预算为 512 个材料单元、200000 个搜索节点。
- 混合特征要求存在观测到的薄矩形宿主（厚宽比不超过 5、宿主置信度 0.95），整体表面覆盖率不低于 0.98。
- 独立曲面实体要求表面覆盖率与边界覆盖率均不低于 0.90，且必须观测到端面或满足角度分箱覆盖；缺面扫描不会被补成封闭实体。
- 旧“单箱体加贯穿孔”路径另做整体材料一致性检查，边界覆盖低于 0.80 时转入曲面实体分支。

模板引导的约束方式：

1. 选区少于 20 个有效点时直接拒绝；选区重叠时返回“Overlapping template groups”。
2. 长方体模板先在选区内拟合并用全景点云补全，要求至少 90% 的选区点落在 3 倍容差带内，否则返回“No observed box boundary supports this template patch”。
3. 曲面模板先做局部检测与精修；端面不完全时仅用真实观测扩展支持，扩展后仍不完整则失败。曲面模板还要求选区表面支持不低于 70%。
4. 所有模板通过后，把候选真正提交到临时 `ModelingCore`，按模板类型逐面检查：每个选区点必须落在对应解析面类型的 CAD 面或候选曲面边界 3 倍容差带内，匹配比例不得低于 90%，否则报出失败选区和“support m/n”。

人工关系在进入识别前校验：两组点各自匹配到当前分析中的有限面片，重叠比例至少为较小支持集的 60%；同类型关系必须已由自动推断给出，否则拒绝并提示“Manual relation conflicts with observed geometry”。人工关系只提升为确认关系，不移动几何。

修复事务在发布前复验：特征数量不得变化，局部边界覆盖不得下降，已识别几何位移不得超过 0.25 倍容差，整体表面覆盖不得低于 0.98。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 模板与引导入口 | `core/geometric_reconstruction/include/reconstruction/recognition/GuidedSolidRecognizer.h` |
| 常规候选竞争 | `core/geometric_reconstruction/src/recognition/ParametricSolidRecognizer.cpp` |
| 模板拟合 | `core/geometric_reconstruction/src/recognition/guided/GuidedSolidRecognizer.cpp` |
| 模板边界复验 | `core/geometric_reconstruction/src/recognition/guided/HintValidation.cpp` |
| 箱体识别 | `core/geometric_reconstruction/src/recognition/BoxRecognizer.cpp` |
| 贯穿孔识别 | `core/geometric_reconstruction/src/recognition/ThroughHoleRecognizer.cpp` |
| 独立曲面实体 | `core/geometric_reconstruction/src/recognition/curved/CurvedSolidRecognizer.cpp` |
| 人工关系校验 | `core/geometric_reconstruction/src/analysis/SurfaceRelations.cpp` |

## 测试与诊断

## 已知限制

- 识别覆盖范围有限：长方体组合、平面加强筋、单箱体贯穿孔，以及独立球、圆柱、圆锥/圆台和完整圆环体；盲孔、凸台、口袋和任意机械零件仍不能稳定恢复。
- 模板只支持平面、圆柱、球、圆锥、圆环和长方体六类；选区必须至少有 20 个有效点，且不同选区不能重叠。
- 曲面实体必须观测到端面或足够的边界角度覆盖。被夹具遮挡、只扫描到侧壁的零件会被拒绝，而不是补出虚构端面。
- 单箱体路径只取支持点最强的六个平面证据；若这六面不能组成三组正交平行面，就转入组合体、混合特征或装配分支。
- 人工关系只能确认自动推断已存在的关系，不能引入新的几何约束，也不会移动或重拟合曲面。
- 候选覆盖率只反映已观测点被解释的比例，不能证明未观测区域或源模型的完整度。
