# 数据集与复现约定

页面中的路径均为仓库内路径或 `outputs/` 下的报告目录。案例定义、源 STEP 和固定换算参数进入版本库；拍摄图片、COLMAP 工作区、点云、拟合日志与重建 STEP 属于生成物，不进入版本库。

## 数据来源与分类

| 类别 | 来源与位置 | 主要特征 |
| --- | --- | --- |
| 合成解析点云 | `tools/contracts/reconstruction_contract_fixtures.py`、`tools/fixtures/` | 由解析几何直接生成，点 ID 与参考产物稳定，可独立复算真值 |
| STEP 表面采样 | `data/real-workpieces-step/` 下的 `box-wall`、`box-wall-1`、`cover-plate`、`oil-tank` | 从 STEP 三角网格逐面采样带法向毫米点云，用于曲面识别验证 |
| 虚拟拍摄重建点云 | `data/performance-validation/level-NN/<case>/model.step` 与 `textures/standard_capture.png` | 62 视角合成拍摄、已知毫米位姿、COLMAP 稠密重建，生成物落在 `outputs/` |
| 实拍点云 | 由点云审核与批量重建工具维护在 `outputs/` 下 | 真实扫描噪声与遮挡，真值主要来自人工划分审核，而非参考 STEP |

契约清单 `data/reconstruction-contracts/v1/manifest.json` 定义 **9 个数据集**：`two_boxes`、`open_box`、`u_channel`、`blind_hole`、`surface_box`、`surface_box_m`、`capped_cylinder`、`open_cylinder`、`mixed_plate`。其中 `surface_box_m` 专门验证米制输入到毫米的换算，`open_box` 故意缺少一个面，`open_cylinder` 故意缺端盖。

性能样例与解析标准件：

源数据按几何难度组织在 `data/performance-validation/level-01/` 到
`level-06/`，从基础几何、组合件、多方向孔和遮挡件，递增到工业件和多实体装配。

| 案例 | 几何与尺寸 | 用途或验收门槛 |
| --- | --- | --- |
| `box-through-hole` | 60 × 40 × 20 mm，中心 Ø12 mm 贯穿孔 | 尺寸与孔径误差 ≤0.5 mm，对称差 ≤5% |
| `l-bracket-gusset` | 80 × 60 × 68 mm，体积 84800 mm³ | 组合零件能力探针；尺寸误差 ≤0.5 mm，对称差 ≤5% |
| `SK08-limobearing` | 1 个实体、27 面、72 边、48 顶点 | STEP 导入、轨道拍摄、相机元数据、点云重建与基元拟合验证 |
| `sphere` | 半径 15 mm，直径 30 mm | 曲面参数误差 ≤0.1 mm，尺寸误差 ≤0.2 mm，对称差 ≤3% |
| `cylinder` | 半径 12 mm、高 30 mm，双端盖 | 同上，验证有界圆柱与端盖 |
| `cone` | 底半径 12 mm、高 30 mm、顶半径 0 | 同上，验证有限圆锥 |
| `torus` | 主半径 25 mm、管半径 6 mm | 外径 62 mm、内径 38 mm；同上 |

合成夹具覆盖单／并合长方体、L 形与三角加强筋、T／U 形支架、三／四层台阶、分离物块、整体旋转与打乱点序。曲面夹具另含截锥；半球、缺端盖圆柱／圆锥和部分圆环属于预期拒绝样本。

## 输入准备与真值

**单位与点 ID。** PLY 只携带位置与法向；单位由相邻的 `cloud.json` sidecar 或显式 `--unit` 声明，sidecar 不包含面、实体或参考标签。缺少单位时工具拒绝按任意尺度推进。点 ID 使用输入 PLY 的 1-based 行号；审核导出的 PLY 另存 `source_point_id`，可映射回工件点云。

**观测快照。** 入口将点云转换到统一毫米坐标，保留源单位、比例和观测指纹。工作点云只用于降采样与分析；装配回填、局部恢复和投影评分按各自口径读取全量观测或指定视图。

**参考 STEP 独立构造。** `python -m tools.fixtures.generate_standard_models` 读取每个 `case.json`，写出 `model.step`，执行 STEP 回读并确认一个有效实体，再核对解析体积和包围盒尺寸。`textures/standard_capture.png` 是 1024 × 1024、种子 4500 的确定性多尺度纹理。

**虚拟拍摄。** `run_performance_validation.py` 先复制模型、纹理和 `case.json` 快照并记录 SHA-256，再执行拍摄；相机必须全部对准原点，轨道半径一致，视角集合严格匹配 5 个仰角环带 × 12 个方位角，另加顶视与底视，共 62 张 1920 × 1080 图像。

**真值使用时机。** STEP 采样和性能验证都在检测结束后才读取参考清单或源 STEP。报告的 `source_model_read=false`、`topology_correspondence_used=false`、`reference_labels_used=false` 用于声明这一点。实拍点云的人工审核结论保存在 `decisions.json` 与 `fit_test_set.json`，只收录人工判定“正确”的点云。

## 指标、容差与报告位置

| 检查项 | 门槛或口径 | 性质 |
| --- | --- | --- |
| 性能样例尺寸 | 各轴误差 ≤0.5 mm | 验收门槛，见 `case.json` |
| 性能样例孔径 | 误差 ≤0.5 mm | 验收门槛，见 `case.json` |
| 性能样例对称差 | 对称差体积 / 源体积 ≤5% | 验收门槛；独立体积采样上界也须 ≤5% |
| 布尔比较容差 | 记录值 0.00001 mm | 仅用于近乎重合面的布尔计算，不放宽 0.5 mm 与 5% |
| 曲面标准件 | 参数误差 ≤0.1 mm | 验收门槛 |
| 曲面尺寸与源边界 | 尺寸、源边界距离 ≤0.2 mm | 验收门槛 |
| 曲面对称差 | 对称差 ≤3%，独立采样上界也须 ≤3% | 验收门槛 |
| STEP 逐面采样 | 每个参考面的同类型候选支持率、实际显示标签覆盖率均 ≥98% | 验收门槛，任一面失败返回非零 |

装配契约的数值门槛来自 manifest：`two_boxes` 期望 12 个面片、2 个部件、12 个边界环、2 个闭合体，体积 375 ± 1e-5 mm³；`open_box` 期望 5 个面片、1 个部件、5 个环、0 个闭合体。投影契约的 U 形槽面积 88–96 mm²、体积 528–576 mm³、内点率 ≥0.9、P95 距离 ≤0.225 mm。

报告位置：

- 性能验证：`outputs/performance-validation-<case>/validation_report.json`、`fitting_report.json`、`pipeline_status.json`、`reconstructed.step`。
- STEP 逐面采样：`outputs/<name>-validation/summary.json`，以及 `analyze.json`、`analyze-client.json` 和着色 PLY。
- 阶段契约：`outputs/contracts-*/summary.json`，每个阶段另有 `artifact.json`、`report.json`、`request.json`。
- 组合体与曲面回归：`outputs/compound-fit-tests*/<case>/`、`outputs/curved-fit-*/<case>/`。

所有输出目录要求新建或为空，避免旧候选、旧 STEP 或旧报告被误认为本次结果。
