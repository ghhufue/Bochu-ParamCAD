# 设计决策

本页记录当前实现中已经生效、且能从代码或配置验证的架构选择。仓库没有维护带编号的 ADR 历史，因此这里不编造编号或时间线，只提供统一的“决策记录格式”和当前决策清单。新决策应按同一格式追加，并在代码合并时同步。

## 决策记录格式

建议每条决策使用以下字段，标题用一句话描述选择，而不是描述问题：

```text
标题：<一句话决策>
状态：提议 / 已采纳 / 已废弃
背景：<为什么需要决定，约束来自哪里>
决策：<当前采用的方案>
替代方案：<考虑过或可行的其他方案，以及未采用的原因>
影响：<对分层、接口、性能、维护和兼容性的影响>
验证：<测试、命令、代码入口或可观察行为>
```

“验证”必须指向仓库内可执行或可检查的依据，例如头文件、CMake 目标、CTest 用例、`.conf` 键或 README 命令。仅凭经验或历史讨论的内容不写入“当前决策”。

## 当前决策

| 决策 | 内容 | 可验证依据 |
| --- | --- | --- |
| 分层与自有类型边界 | 几何重建层不依赖 Qt/VTK，也不持有 B-Rep；COLMAP、CGAL、OpenCASCADE、VTK 类型停留在各自适配边界内，跨层只传项目自有类型。 | `core/geometric_reconstruction/README.md`、`core/parametric_modeling/README.md` |
| 观测不可变与稳定源点 ID | `PointStore` 保存不可变源观测；降采样、过滤和修复合并不重写点，`PointId` 为 `std::uint64_t`，证据通过 ID 引用支持点。 | `core/geometric_reconstruction/include/reconstruction/data/PointStore.h` |
| 观测快照只做尺度转换 | `ObservationSnapshot` 只把已知单位统一到工作尺度，保留源单位和比例，并以指纹标识点序、单位、ID、位置和法向证据。 | `core/geometric_reconstruction/include/reconstruction/data/PointStore.h` 中的 `ObservationSnapshot` 与 `pointStoreFingerprint` |
| 识别候选不是 CAD | 识别输出仍是候选（如 `CompoundCandidate`）；只有转换为 `ModelPatch` 并由建模核心提交后，才产生特征历史与 B-Rep。 | `core/geometric_reconstruction/README.md`、`core/parametric_modeling/include/modeling/ModelPatch.h` |
| 默认实体推断、旧装配显式隔离 | 模板列表为空时，生产重建走 `SurfaceScene → inferSolids → 评分选择`；旧装配链只在离线 `--legacy-assembly` 下运行，失败不自动回退。 | `GuidedSolidRecognizer.cpp` 的空模板分支、`CadReconstructionTask.cpp` 的 legacy 分支、`OfflineCadCommands.cpp` 的参数解析 |
| 场景契约冻结观测 | 下游只消费带分析/源/设置指纹的 `SurfaceScene`；边界缺失显式标记为开放或歧义，不在下游重新提取轮廓。 | `solid_inference/SurfaceScene.h`、`SurfaceSceneAdapter.cpp` |
| 建模补丁原子提交 | `ModelPatch` 绑定 `expectedRevision`，命令与可选重建要么全部成功，要么文档保持原状；失败不会留下部分特征。 | `core/parametric_modeling/include/modeling/ModelPatch.h`、`ModelingCore::apply`、`ModelingCore::revision` |
| 语义特征优先于固定拓扑命名 | 长方体六个面、圆柱底面/顶面使用语义角色；草图可关联 `FeatureId + FaceRole`，而不是依赖布尔结果的面编号。任意布尔面与裁剪边界的稳定命名仍未实现。 | 根 `README.md` 第 2.4 节、`core/parametric_modeling/README.md` |
| 配置外置 | 拍摄、预处理、检测、分析、识别、装配和投影参数放在 7 份 `.conf` 中，运行时可覆盖，算法代码只保留纯结构常量。 | `apps/ParamCAD Studio/config/*.conf`、配置加载与校验代码 |
| VTK 只读显示 | B-Rep 是权威几何；VTK 只显示由 B-Rep 派生并三角化的网格，显示变换不改变模型尺寸。 | 根 `README.md`、`apps/ParamCAD Studio/README.md` |
| 工程格式向后兼容 | 普通工程写 `version: 2`，带目标体布尔操作写 `version: 3`，草图含圆弧写 `version: 4`，读取接受 1、2、3、4；缺失字段按既定默认值恢复。 | `apps/ParamCAD Studio/src/io/ProjectStore.cpp`、持久化模块页 |

这些决策共同约束失败语义：任一阶段的证据、候选或补丁失败时，上游观测和当前 B-Rep 保持不变；后台结果还必须通过点云、模型和视口实体修订检查，过期结果不得覆盖新状态。单位规则同样明确：建模核心使用毫米，未知尺度必须由用户确认，不能被静默当作毫米。

## 决策之间的关系

- 分层边界让观测层只产出证据和候选，建模层只接受命令或补丁，因此两侧可以独立测试。
- 不可变观测与修订检查共同保证结果可复现：输入或状态变化后，旧分析不能被误用。
- 语义特征命名支撑客户端参数编辑；固定拓扑命名缺失则限制了通用布尔结果上的约束和参照。
- 配置外置让阈值可现场调整，但也要求每次改键时同步校验、文档和回归用例。
- 工程格式兼容与核心层解耦：客户端负责归档版本，建模核心不依赖客户端 JSON 或工程文件。

## 替代方案与影响

- **不引入 PCL 作为第二套点云栈。** 当前预处理、法向和证据类型由项目自有代码与 CGAL 提供，避免跨库点类型转换和额外构建依赖；代价是降采样、离群过滤和精修逻辑需要自行维护。
- **不做以网格为最终产物的重建。** 泊松重建一类方法能生成三角网格，但无法直接提供可编辑的尺寸、位置和特征语义，与“参数化 CAD”目标冲突；因此当前必须从观测中恢复解析曲面证据，并对缺面、遮挡和不完整实体显式拒绝。
- **不把 COLMAP、CGAL 或 OCCT 类型跨层传递。** 跨层传递会把这些库的版本和 ABI 固化到公共接口，也会让几何层意外依赖建模层；当前选择在适配器内完成映射，代价是需要维护转换代码和证据归一化。
- **暂不承诺任意布尔面的稳定命名。** 现有语义面只描述源特征的支持面，不保证在布尔运算后仍对应某个有界面；因此下游约束和参照面选择受限，通用拓扑命名仍待实现。
- **用不可变快照加修订失效替代增量修改。** 点、单位、分析结果或模型一旦变化，旧候选按指纹和修订号丢弃并重新计算；这牺牲了增量分析的开销优势，换取可复现和不会误用旧证据的确定性。
- **配置外置而非编译期常量。** 外置让现场调参与回归复现不需要重编译，但引入了配置校验、优先级和版本迁移问题；当前 `.conf` 没有 `schema_version`，键重命名会按未知键报错，迁移机制仍待补充。
