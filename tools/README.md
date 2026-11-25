# ParamCAD 工具

`pcad` 是统一客户端命令入口。`tools/pcad.cmd` / `python tools/pcad.py` 仅负责定位 CMake 最近构建登记的原生 `pcad.exe` 并透传参数、工作目录、输出和退出码。C++ 后端执行离线任务，在线请求交给正在运行的 Studio。

使用方式、能力矩阵和 AI 操作契约见 [CLI 专章](../docs/site/cli/index.md)。`pcad reconstruct cad --offline --in cloud.ply --unit mm --output outputs/cad-new` 已提供正式完整重建入口；下文验证程序保留为开发诊断用途。

结构光固定场景验收使用 `validation/run_structured_light_validation.py`：通过 `--client` 指定实际客户端、`--output` 指定新目录，比较 Gray Code、相移、多曝光和均匀材质；所有拍摄与重建仍在客户端内完成。结果口径和开发环境依赖见[验证记录](../docs/site/development/structured-light-validation.md)。

## 点云划分人工审核桌面工具

双击检查包目录中的 `点云划分审核.lnk` 或 `启动点云审核.cmd`，也可执行：

```powershell
python -m tools.component_review.review_component_clouds --dataset outputs/component-cloud-review-20261101
```

依赖 Python 自带的 Tkinter 和 Pillow，离线运行。按原编号依次加载拼图，三个按钮分别记录
“点云划分有问题”“点云划分正确（可以作为拟合的测试集）”“暂不做决定”，保存后自动下一张。
快捷键 `1/2/3`；左右键翻页；滚轮缩放、拖动查看；支持整件定位图、工件/状态筛选、备注及回看改判。
备注框/搜索框输入数字不会触发判定快捷键。

检查包的 `review/` 保存：

- `decisions.json`：按点云名称记录编号、判断、备注、时间、点云哈希和续审位置，原子写入。
- `decisions.csv`：可用 Excel 查看全部编号的当前判断。
- `fit_test_set.json`：仅收录人工选择“正确”的点云，改判后同步移除。
- `decisions.json.bak`：上一次保存的记录。

关闭重开继续上次位置；单个检查包的同一记录目录只允许一个审核窗口写入。
记录绑定点云内容，改变点云后旧判断不会自动沿用。默认四份 `000` 未归属点也在列表中。
`--reviews <目录>` 可为同一检查包创建独立的审核记录（也用于隔离 UI 测试）。

```powershell
python -m unittest discover -s tools/tests -p test_component_review.py -v
```

## 批量导出独立候选点云与多视角检查图

已有点云重建的四工件输出可批量拆成独立 PLY，并生成每份 12 视角拼图及整件编号定位图：

```powershell
python -m tools.component_review.export_component_clouds `
  --reconstruction outputs/boundary-repair-20261101/final `
  --output outputs/component-cloud-review-new --workers 4
```

依赖 `numpy`、`Pillow`。输出目录必须新建；脚本核对原始点云及算法执行文件 SHA-256，
算法更新后需先重新执行重建。闭合和未闭合候选全部导出，不按 CAD 是否生成过滤。
文件夹使用 `<工件>_<原候选编号>`，每份包含 `cloud.ply`、单位/来源 `cloud.json`、`views.png`。
四份 `000` 文件夹保存未归属点。根目录包含四张 `<工件>_locator.png`、`index.csv` 和总清单。

PLY 保留原始坐标、法向及属性，不降采样、不居中；`source_point_id` 保存源 PLY 的 1-based 点行号。
图片才使用独立 PCA 主轴和显示采样，12 视图比例一致；每件定位图分栏标注全部编号，
引线端点及其原始点 ID 同时保存到总清单，便于核对对应关系。
后续测试独立点云时，将其 `cloud.ply` 与同目录 `cloud.json` 一起保留。

本轮结果为 329 个候选 + 4 份未归属点，共 333 个文件夹；导出测试：

```powershell
python -m unittest discover -s tools/tests -p test_component_cloud_export.py -v
```

## STEP 点云与逐面识别验证

已有验证工具现在支持直接从 STEP 表面采样，无需图片或 COLMAP：

```powershell
cmake --build "apps/ParamCAD Studio/build/msvc-debug" --config Release --target cad_reconstruction_validation
python -m tools.validation.run_step_surface_validation data/real-workpieces-step/cover-plate/cover-plate.step --output outputs/cover-plate-validation
```

输出目录必须新建或为空。生成带法向的毫米点云、独立 STEP 参考清单、核心和客户端逐面报告、
着色分区 PLY 及 `summary.json`。每个参考面要求同类型候选支持率和实际显示标签覆盖率均达到 98%；
任一面失败，脚本返回非零。`--noise 0.005 --seed 719` 可重复验证位置噪声，
`--skip-client` 只跑核心。算法不接收参考标签；参考清单仅在检测完成后用于核对。
使用相同参数加 `--replay` 可重用已保存点云，之前的报告和着色点云会保留在 `history/`。

这是曲面识别验证，不代表已经恢复封闭、可编辑的 CAD 实体或导出重建 STEP。
实体统计表示该参考实体的所有面均通过检查，不表示自动恢复了装配实例的身份。

底层命令也可单独重放：

```powershell
& "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe" sample source.step reference.json cloud.ply 0.5
& "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe" analyze cloud.ply analysis.json reference.json 0.05
& "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe" analyze-client cloud.ply client.json reference.json 0.05
```

`sample` 按 `0.5 mm` 目标间距为每个 STEP 面分配面积加权样本，在三角网格的 UV 区域中采样并核对实际裁剪边界，
再计算解析位置和有向法向。逐面实际密度会记录在参考清单中。正确性验证不启用 `--preserve-edges`。
检测读取的 PLY 只有位置和法向，单位 sidecar 也不含面或实体标签。

## 仅点云的 CAD 重建

如果部件只观测到一张平面、圆柱面或圆锥面，可以单独拟合有限曲面并导出 STEP：

```powershell
& "apps/ParamCAD Studio/build/msvc-debug/cloud-only/Release/cad_reconstruction_validation.exe" `
  fit-observed-surface cloud.ply surface.json surface.step 0.05
```

点云须有毫米或米单位信息，末尾参数为毫米距离容差。曲面 STEP 使用新的输出路径。平面保留观测到的内边界，圆柱和圆锥按观测范围裁切。多面样本可将各自有观测支持的平面、圆柱和圆锥导出为开放曲面组，报告的 `faces` 列出各面的拟合参数。`surface_fitted` 记录曲面拟合，`cad_constructed` 区分闭合实体。

边界杂线与壳闭合问题可用 `python -m tools.diagnostics.diagnose_assembly_boundaries --baseline <批处理输出> --output <新目录> --probe`
复测。默认、alpha 半径增大、全输入点三个探针直接调用同一部件分离实现，保留输入校验和、
参数及开放边/轮廓统计；不会读取参考 STEP 或将实验参数改成生产默认。
此前的四工件边界分析、首轮轮廓修复及验收记录已归档。 [历史边界诊断](../docs/Outdated/Assembly_Boundary_Investigation_20261101.md) · [历史修复报告](../docs/Outdated/Assembly_Boundary_Repair_20261101.md)

客户端与批处理均只使用点云坐标、法向、单位及用户手动指定的模板。
原 STEP 的自动发现、读取、逐面对应、拓扑引导和相关重建命令已删除。
已有工程的参数重建、普通 STEP 导入／导出继续保留。

```powershell
python -m tools.reconstruction.run_point_cloud_cad --input scan=path/to/scan.ply --unit mm --output outputs/cloud-cad-new
```

脚本只复制 PLY 和单位信息到隔离输入目录，再调用客户端同一个后台重建任务。
支持重复 `--input NAME=FILE.ply`，每个案例单独记录结果；没有对应表或原模型输入参数。
输出目录必须新建或为空，`--timeout 300` 控制每个案例的计算时限。

成功时输出可编辑 PCAD、参数 JSON 和 STEP，并验证保存重开及导出回读。
成功仅表示当前识别出的几何能形成有效 CAD，不证明恢复了未知原模型的全部部件。
多部件重建还输出 `segmented.ply` 与 `parts.json`，记录候选归属、有限边界、连接和未闭合原因。
仅部分闭合时报告 `cad_exported_partial`；没有可验证实体时报告 `unsupported`，仍保留分离诊断。
分离流程的 `surface_coverage` 是归属于闭合候选的观测点比例，不是原模型恢复率；`coverage_basis` 明确语义。
不会读取旁边的模型文件来补齐。此前的四份案例结果和客户端用法已归档。 [历史纯点云 CAD 说明](../docs/Outdated/Real_Workpieces_Editable_CAD.md)

前一节的 STEP 采样／逐面评测仅用于离线制作与评估测试数据；
参考拓扑只在曲面检测完成后用于评测，不进入本节的重建任务。

## 投影引导的草图、拉伸与切除

独立部件拟合可以直接使用审核工具选出的 PLY，需保留相邻的单位 JSON 或用 `--unit` 声明单位。

```powershell
python -m tools.reconstruction.run_point_cloud_cad --mode projection `
  --input part=path/to/cloud.ply --output outputs/projection-fit
```

批处理可重复传入 `--input`。加 `--area-directions` 可额外尝试凸包投影面积较小的方向，
与默认的面片、圆柱轴、PCA 和侧面法向候选作对照。
单件报告的点 ID 为输入 PLY 的 1-based 行号，审核 PLY 中的 `source_point_id` 可映射回工件点云。
通过时输出 `model.pcad`、特征 JSON 与 STEP。其余案例保留候选轴和拒绝原因。

单件拟合从有观测的端面恢复折线及整圆草图，检查内环的侧壁或圆柱支持。
有孔壁与孔底时可尝试盲孔切除，有槽底和槽壁时可尝试直槽切除。
中间平面可用于提出凸台拉伸。报告记录采样点到实体边界的距离、法向误差和缺乏匹配观测的模型面面积比例。
容差默认取部件包围盒对角线的 0.25%。直接调用验证程序时可传入毫米容差覆盖此值。
当前通过门槛为评分样本内点率至少 90%、边界距离 P95 至多三倍容差、缺少观测支持的模型面面积比例至多 4%。
这些指标用于筛选候选。对未采样表面和体积的判断需要额外观测。

```powershell
& "apps/ParamCAD Studio/build/msvc-debug/cloud-only/Release/cad_reconstruction_validation.exe" `
  fit-projection path/to/cloud.ply report.json model.step 0.05
```

整件装配的 `--mode assembly` 和客户端 **Tools → Reconstruct CAD** 默认对未闭合候选尝试投影重建。
关闭投影可使用 `--mode assembly-no-projection` 或客户端 **Tools → Reconstruct CAD without Projection Fallback**。
该路径在有限壳闭合后尝试至多 6 个有多面片的开放候选，优先处理点数较多的候选。
原壳闭合与投影拟合分别报告。包含目标体布尔操作的 PCAD 使用项目格式 3。

## 安装与使用

桌面快捷方式可使用 `tools/launch_paramcad_latest.ps1` 作为稳定入口。它按文件修改时间选择最新 Release 构建，包含 `Release/plane-growth` 等变体子目录及 CMake 配置的输出目录；只有路径与构建时间均匹配时才复用已有窗口，旧版窗口保留，新版另行启动。脚本默认从自身位置解析仓库根目录，也可传入 `-RepositoryRoot`。

只检查将要启动的程序而不启动或切换窗口：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/launch_paramcad_latest.ps1 -ResolveOnly
```

诊断大平面仅识别局部时，可用同一输入比较关闭/开启支持扩展的结果，并检查最大残差及客户端实际归属：

```powershell
& 'artifacts/build-structured-light/Release/cad_reconstruction_validation.exe' `
  analyze-plane-growth path/to/cloud.ply report.json
```

输入需要毫米单位元数据。报告的 `baseline` 与 `expanded` 使用相同工作点、种子和容差；`client` 为真实表面识别入口的点归属统计。支持率是检测证据覆盖率，不等同于 CAD 召回率或实体闭合成功率。

在仓库根目录运行一次，将 `tools` 加入当前用户的 PATH，无需管理员权限：

```powershell
& .\tools\install_pcad.ps1
```

安装脚本会立即更新执行它的 PowerShell 会话。其他已打开的终端需要重新打开；
如果终端宿主仍继承旧 PATH，也可以在该终端重新运行安装脚本。重复安装不会添加重复条目。
仓库移动后重新运行安装脚本，并从用户 PATH 中移除旧目录。

```powershell
pcad --help
pcad studio
pcad studio --dry-run
pcad studio --exe "D:\custom\ParamCAD Studio.exe"
pcad clean output --dry-run
pcad clean output
```

`studio` 由匹配的原生 CLI 启动同目录客户端；旧版候选查找已由构建登记替代。启动后终端立即返回。

`clean output` 删除本仓库 `outputs/` 下生成的文件与目录，保留 `README.md` 和
`.gitignore`。包括生成的点云、截图、日志及重建结果；需要保留的结果请提前备份。
`--dry-run` 仅列出将删除的顶层条目。清理范围固定，符号链接和 Windows junction
只移除链接本身；若 `outputs/` 本身为链接或缺少说明文件则拒绝清理。

无需安装也可以直接运行：

```powershell
.\tools\pcad.cmd studio
python .\tools\pcad.py clean output --dry-run
```

原有 `python tools/clean_outputs.py [--dry-run]` 入口继续有效，转发到同一份实现。
退出码：`0` 成功，`1` 部分清理失败，`2` 参数、路径或启动错误。

## 代码组织

```text
tools/
├── pcad.cmd              Windows 命令入口，调用 Python
├── pcad.py               Python 统一入口
├── install_pcad.ps1      注册当前用户 PATH
├── clean_outputs.py      旧清理命令入口
├── review_component_clouds.py  既有审核快捷方式入口
├── pcad_cli/             原生 CLI 定位、参数透传和仓库输出清理
├── reconstruction/       PLY 批量重建
├── component_review/     点云导出、审核界面和审核记录
├── contracts/            模块契约测试驱动和参考产物生成
├── fixtures/             标准 STEP 和合成点云生成
├── validation/           拟合、逐面、性能和可靠性验证
│   └── native/           C++ 验证程序和模块契约阶段入口
├── diagnostics/          装配边界、性能和拟合稳定性诊断
├── assets/               Studio 图标生成
└── tests/                Python 单元与集成回归
```

从仓库根目录用 `python -m tools.<分类>.<脚本名>` 运行开发分类工具。正式客户端命令在 `apps/ParamCAD Studio/src/cli` 实现，同步 CommandProtocol 能力清单、在线工作区入口、测试和文档站 CLI 章节。Python 包只维护仓库启动适配和清理工具。

在仓库根目录执行相关测试：

```powershell
python -m unittest discover -s tools/tests -p test_pcad_cli.py -v
```

## 平面面片诊断

从不可变分析包解释单面记录，或比较两个平面的观测差异：

```powershell
pcad inspect --bundle path/to/cloud.ply.pcad-analysis explain --id 118
pcad inspect --bundle path/to/cloud.ply.pcad-analysis compare --ids 118,119
```

报告区分最终支持验证、恢复候选来源、记录的相邻/共面关系与显示分组。新 bundle 还会保留 Studio 原始点云支持恢复的候选对和门槛结果，并用 `source_recovery_proposal_links` 映射所有权前验证 ID。`trace_available`、`candidate_linked`、`matching_attempt_found` 和 `trace_truncated` 分别说明 trace 存在、所选面已关联候选、候选出现于尝试事件、事件记录是否截断。该 trace 不覆盖初始 RANSAC 或工作点云检测。只有候选 ID 匹配的事件才可用于解释合并拒绝。维护诊断流程时使用 `$paramcad-plane-diagnostics` Skill，并为新的逐阶段原因码补结构化字段和回归 fixture。
比较报告还区分两个面是否来自同一恢复 proposal，以及是否找到两者之间的配对事件。相同恢复 proposal 的多个最终面说明后续 patch/归属处理，不表示两个最终面之间发生合并拒绝。


## 点云重建模块契约测试

v1 清单覆盖模块 01–34。每次运行中，每个数据集只准备一份 PLY。清单通过数据集 ID 将用例映射到固定参考产物和串联依赖。参考产物由解析几何生成，点与证据通过源点 ID 引用同一份点云。盲孔样本用于基元检测、通孔拒绝、投影搜索、建模、工程重开和 STEP 回读。

```powershell
cmake --build "apps/ParamCAD Studio/build/msvc-debug" --config Release --target cad_reconstruction_validation
python -m tools.contracts.run_reconstruction_contracts --output outputs/contracts-all
python -m tools.contracts.run_reconstruction_contracts --mode cloud --case m21.two_boxes.closure --output outputs/contracts-closure
python -m tools.contracts.run_reconstruction_contracts --mode fixture --case m24.u_channel.profile --output outputs/contracts-profile
python -m tools.contracts.run_reconstruction_contracts --case m11.surface_box.revalidate --output outputs/contracts-repair
python -m tools.contracts.run_reconstruction_contracts --case m13.blind_hole.reject_short_wall --output outputs/contracts-hole
python -m tools.contracts.run_reconstruction_contracts --case m33.blind_hole.step_roundtrip --output outputs/contracts-step
python -m tools.contracts.run_reconstruction_contracts --case m34.surface_box.production --output outputs/contracts-workflow
```

默认同时运行两种模式。`fixture` 直接读取参考上游 JSON，只执行目标阶段。`cloud` 按依赖运行当前实现，同一次运行中共享上游结果。装配串联顺序为面片提取、分组、边界恢复、全量回填、闭合。回填在报告中单独记为 19 的 `backfill` 阶段，也可作为目标用例直接运行。上游未达到验收条件时，下游标记 `blocked_upstream`。

也可以重放串联模式保存的 `artifact.json`。例如将 `cloud/two_boxes/19/artifact.json` 传给 `--mode fixture --case m20.two_boxes.boundary --upstream <文件>`。模块 21 的上游产物是包含全量归属的 `cloud/two_boxes/19-backfill/artifact.json`。外部产物若引用不同的 PLY，用 `--cloud` 显式提供其原始文件。驱动会核对快照身份。投影串联测试按配置选一条候选轴传递，整件的多轴竞争仍由 `fit-projection` 回归覆盖。

`summary.json` 记录用例、实测指标、阻断原因、实际执行次数和输入、清单及验证程序的哈希。每次运行使用新的输出目录。同一数据集在同一阶段配置多个断言用例时，用 `upstream_gate` 指定供下游判断是否可继续的用例。新增模块时，需要声明依赖、注册阶段入口，再为该阶段补独立夹具和验收指标。

输入与分析串联顺序为 01 → 02 → 03 → 04。05 独立验证一次基元检测。06 从 04 的工作点汇集证据，按配置自行运行分族或多尺度检测，然后进入 07–11。可将 `cloud/surface_box/6/artifact.json` 传给 `--mode fixture --case m07.surface_box.patches --upstream <文件>`，只复验面片索引。10 通过清单中的探测坐标选定当前面片作为待恢复区域。11 读取该局部提案，合并观测与证据后复验候选，原始 PLY 仍通过同一数据集引用。

识别阶段 12 从 07 的面片识别宿主箱体，13 读取 12 的宿主和已有圆柱证据识别通孔。14 从 07 识别有界曲面体，15 从 04 识别组合体，16 从 08 识别混合特征，17 从 04 的工作点和引导种子识别实体。可用 `cloud/blind_hole/12/artifact.json` 重放 13 的盲孔拒绝用例。

28–33 读取 27 选中的投影操作，依次生成建模计划、验证草图、重建实体、提交事务、保存并重开 PCAD、导出并回读 STEP。模块 31 校验失败回滚和过期版本拒绝，模块 33 校验实体数量及材料体积交并比。可将 `cloud/blind_hole/32/artifact.json` 传给 `--mode fixture --case m33.blind_hole.step_roundtrip --upstream <文件>`。报告目录同时保留 `roundtrip.pcad` 和 `roundtrip.step`。

34 从原始 PLY 或指定了活动源点 ID 的参考产物启动应用的异步重建任务，校验任务结果投递、取消和建模提交。工作区交互由 `workspace_interaction_test` 覆盖。
