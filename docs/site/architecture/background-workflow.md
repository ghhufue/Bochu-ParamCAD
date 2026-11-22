# 后台任务与结果提交

本页描述从点击“重建 CAD”到候选写回模型的完整后台路径：入口有哪些、任务如何取得不可变快照、取消在什么位置生效、结果如何校验修订，以及部分成功与失败分别留下什么状态。几何算法本身不在这里展开。

## 生产入口与分支

生产重建共有三个入口，菜单文本与代码行为一致（`apps/ParamCAD Studio/src/app/MainWindow.cpp`）：

| 入口 | 参数 | 用途 |
| --- | --- | --- |
| 工具 → 重建 CAD | `projectionFallback = true` | 默认实体推断；该开关只在显式 legacy 装配时生效 |
| 工具 → 重建 CAD（不使用投影回退） | `projectionFallback = false` | 同样运行默认实体推断；默认路径不使用投影回退开关 |
| 调试 → 在导入的点云上拟合投影… | `startProjection` | 直接对单件点云运行投影拟合，结果进入投影调试页 |

启动前的检查顺序是固定的（`apps/ParamCAD Studio/src/app/PointCloudWorkspaceController.cpp`）：

1. 已有后台任务、分析线程或正在启动时直接返回；
2. 源路径为空（未载入点云）时给出提示，不创建任务；
3. 若当前重建包含手动修改过的重建特征，先询问是否允许替换，拒绝则退出；
4. 取当前工作点云；工作点为空时提示并返回；
5. 尺度未知时要求用户确认“1 个源单位 = 1 毫米”，取消则退出；
6. 尺度确认会打开嵌套事件循环，返回后重新比对点云、模型与视口实体修订，防止期间发生导入或编辑。

分支选择只取决于是否存在模板约束：`hints.empty() ? Conventional : Guided`。两条分支最终都调用同一个 `reconstructGuidedSolid`：空模板分支导出 `SurfaceScene` 并运行实体推断；模板分支先拟合并固定模板，再用既有识别器处理剩余材料。人工关系随快照一起传入，不因为分支不同而被忽略。

旧装配链不再由识别结果自动触发。只有调用方显式传入 `legacyAssembly = true` 时，`runCadWork` 才跳过实体推断并调用 `reconstructAssembly()`；当前可见入口是离线 CLI 的 `--legacy-assembly`。Studio 的默认任务始终传入 `false`，实体推断失败也不会回退到装配。

`CadReconstructionTask` 把输入打包成值语义的 `CadWork`：点云、单位、模板、源容差、共享分析、人工关系、投影与专用恢复开关、投影选项、`legacyAssembly` 标记。`start()` 只在本对象未运行时接受请求，随后创建独立 `QThread` 执行 `runCadWork`。工作线程不访问模型文档、视口或工作区状态，只读快照并产出 `CadReconstructionResult`。

线程与信号的关键约定：

- `stageChanged` 报告预处理、模板拟合、识别、分离部件、投影与取消阶段；
- `finished` 在任务对象所属线程发出，发出期间 `isRunning()` 仍为 `true`，因此结果处理不会被新任务插入；
- 工作线程的异常统一转为 `Failed`，异常文本写入 `result.error`；
- 析构时先置取消标志、断开信号并 `wait()`，避免线程仍在运行时销毁对象。

取消以共享的 `std::atomic_bool` 作为判定依据；析构时还会调用 `requestInterruption()` 帮助线程尽快退出。

- `cancel()` 仅在运行中且标志尚未置位时动作，并把阶段切换为 `Cancelling`；
- `isCancellationRequested()` 同时要求任务仍在运行且标志已置位；
- 取消检查发生在阶段边界：取快照后、预处理后、识别后、装配后；已经进入的 CGAL、OCCT 或装配计算不会中途强制打断；
- 若计算已完成但结果尚未投递，取消仍然获胜，结果被改写为 `Cancelled`。

阶段回调本身也带一致性检查：只有任务仍在运行、取消标志仍是同一个对象且未被置位时，阶段变更才会被转发。

## 实时预览通道

默认实体推断在计算过程中通过回调上报临时显示数据，不等待最终提交：

- `SurfaceAnalysisProgress` 在支撑验证的 `Enforce` 模式中逐片上报已接受的面片；`Report` 模式在分析完成后统一上报；
- `GuidedRecognitionProgress::solid` 接收候选试建得到的临时 `TopoDS_Shape`；空形状表示撤回上一个预览；
- 工作线程把预览写入互斥保护的待发送缓冲，主线程用 50 ms 定时器批量投递 `previewChanged`，任务结束时再补一次；
- 工作区在应用预览前再次比对点云、模型与视口实体修订；不一致、取消、任务结束或点云重新同步都会清除预览；
- 预览网格或形状构造失败被视为可选显示失败，不影响识别与评分结果；
- 预览不修改点云文档、模型修订或特征历史。legacy 装配路径不发送实时预览。

## 结果交接与修订校验

结果回到工作区线程后进入 `onCadReconstructionFinished`。投影调试结果先分流到投影调试页，只更新预览与诊断，不修改 CAD。

其余结果按以下顺序处理：

1. 比对点云修订、模型修订与视口实体修订；任一变化则丢弃整轮结果并提示重新重建，模型保持原状；
2. 通过修订校验且带点云指纹的分析才缓存为下一次可复用的表面分析；
3. 取消时只报告“模型保持原状”；
4. 预处理完成时把点数、降采样移除数、离群移除数与法向统计写入日志；
5. 有 legacy 装配结果时先把部件候选写回点云集合、更新闭合与开放边界显示并记录数量，即使最终结果是 `Unsupported`；
6. 非成功结果按类别提示；成功结果才提交候选。

模型提交由 `ModelingController::commitReconstructedCompound` 完成：保存当前模型作为撤销快照，以当前修订作为预期修订、`replacePrevious = true` 调用 `commitCompoundCandidate`。补丁失败时只返回错误并弹窗，文档由建模核心回滚，不会留下半成品。提交成功后记录撤销历史、发出 `modelRebuilt`，工作区重新三角化并刷新显示。

模板拟合结果以派生数据方式回填到点云选区，不额外写入撤销步骤；随后重新推断关系，保证界面关系仍指向原始点。最终日志与状态栏按长方体、棱柱、通孔、覆盖率、曲面实体和有限壳体统计。若本轮不是装配结果且未归属点数达到门槛，会再触发一次表面分析。

## 部分结果与失败处理

`CadReconstructionResult::Outcome` 只有五种取值：`Success`、`PreprocessingFailed`、`Unsupported`、`Failed`、`Cancelled`；分支由 `Branch` 单独记录为 `None`、`Conventional`、`Guided` 或 `Assembly`。

装配结果能否提交取决于是否产生了通过验证的候选：

| 场景 | 结果 | 模型状态 | 点云与界面 |
| --- | --- | --- | --- |
| 预处理失败 | `PreprocessingFailed` | 不变 | 记录错误，不生成候选 |
| 实体推断失败 | `Unsupported`，`branch = Conventional` | 不变 | 可显示失败选区；有检测面片时保留并显示面片；不进入装配 |
| legacy 装配只建立分组、没有闭合实体 | `Unsupported` | 不变 | 显示部件分组、边界与开放状态 |
| legacy 装配部分成功 | `Success`，`partialAssembly = true` | 提交通过验证的实体 | 保留开放部件与诊断 |
| legacy 装配调用失败 | `Unsupported` | 不变 | 不保留旧候选，因为没有先运行实体推断 |
| 投影未通过 | 独立入口为 `Unsupported`；legacy 装配回退中该部件保持开放 | 不变 | 调试页保留拒绝原因与质量诊断 |
| 建模补丁失败 | `Success` 但提交失败 | 整体回滚 | 弹窗显示建模错误 |
| 计算期间点云或模型被修改 | 结果丢弃 | 不变 | 提示重新重建 |
| 用户取消 | `Cancelled` | 不变 | 在阶段边界停止 |
| 未捕获异常 | `Failed` | 不变 | 记录异常文本 |

`partialAssembly` 的判定随分支不同：默认实体推断下，只要选中候选存在未解释观测点就标记部分结果；legacy 装配下，还会考虑未归属全量观测、未闭合部件，以及 `closed_circular_slab_inferred_contact` 这类推断接触状态。这样界面不会把“显示了一部分实体”误报为完整重建。

需要强调的两条边界：识别成功不等于建模一定成功，候选必须经过 `ModelingCore::apply`；后台取消只在阶段边界生效，长耗时几何计算不会因为点击取消而立即返回。
