# 端到端精度与性能评测

端到端链路从标准 STEP 出发，经 62 视角虚拟拍摄、相机元数据校验、COLMAP 稠密重建、点云预处理、语义识别、原子建模提交、PCAD 保存重开和 STEP 导出回读，最后才与源模型比较。以下数值分为“验收门槛”和“2026-10-04 参考运行”，后者只代表单次虚拟拍摄，不是重复运行成功率或真实照片基准。

## 图像到点云

```powershell
python -m tools.validation.run_performance_validation --model data/performance-validation/box-through-hole/model.step --texture data/performance-validation/textures/standard_capture.png --output outputs/performance-validation-box-through-hole --gpu
```

拍摄阶段固定 5 个仰角（−60°、−30°、0°、30°、60°）× 12 个方位角（0°–330°，步长 30°），另加顶视与底视，共 62 张 1920 × 1080 图像。校验器要求：

- `images/manifest.json` 为版本 2，`shots`、PNG 文件与 `camera_info.json` 的 `images[]` 数量均为 62；
- 所有相机目标点都在原点容差内，轨道半径一致；
- 视角集合严格等于预定集合，不能缺失或重复；
- `camera_info.json` 的图像名与 PNG 完全一致，相机尺寸与 PNG 尺寸一致。

COLMAP 默认使用 GPU SIFT 与 CUDA 稠密 PatchMatch。校验不只看“请求了 GPU”，还要求特征提取、特征匹配、`patch_match_stereo` 成功，并实际生成几何深度图；任一条件不满足即失败。`--no-gpu` 走 CPU 特征与稀疏重建，因为 COLMAP 没有 CPU PatchMatch 路径，不能用它冒充稠密结果。

2026-10-04 参考运行的点云规模：

| 案例 | 稠密点数 | 预处理后点数 |
| --- | --- | --- |
| `box-through-hole` | 675,791 | 67,516 |
| `l-bracket-gusset` | 544,302 | 66,550 |

上述点数和预处理结果来自两个案例目录的 README，属于单次虚拟拍摄记录。图片、相机内外参、COLMAP 工作区、数据库、点云和日志都保存在同一个案例目录内；`validation_report.json` 汇总 `capture`、`colmap` 与 `fitting` 三部分。

## 点云到可编辑 CAD

点云进入 `CadReconstructionTask` 后，按配置选择常规识别、人工引导或装配分支。候选通过建模适配器转成修订绑定的建模补丁，再由建模核心原子应用，生成特征历史与 B-Rep。成功路径随后写出 PCAD、特征 JSON 和 STEP，并分别执行工程重开与 STEP 回读。

性能验证调用同一个验证可执行程序，输入为点云、`fitting_report.json`、`reconstructed.step`、源 STEP 快照和 `case.json`。报告中的关键状态包括：

| 字段 | 含义 |
| --- | --- |
| `cad_constructed` | 已构造有效 B-Rep 并导出 CAD |
| `brep_valid` | B-Rep 通过有效性检查 |
| `solids` | 导出实体数量 |
| `step_round_trip_verified` | 导出 STEP 可回读且实体数与有效性保持 |
| `project_round_trip_verified` | PCAD 保存后重开，特征数量保持 |
| `complete_cad_fit` | 尺寸、孔、曲面参数、对称差等真值检查全部通过 |

有效 B-Rep 不等于通过精度验收。`l-bracket-gusset` 的初次误识别曾生成有效 B-Rep 和 STEP，但被 `cad_accuracy_failed` 拒绝；当前实现改为识别两个长方体加一个三角截面拉伸棱柱，才通过源模型检查。

## 几何精度与 STEP 回读

下表为 **2026-10-04 参考运行（单次虚拟拍摄）** 的实测值，不是验收门槛。

| 案例 | 识别结果 | 尺寸误差 | 对称差 | STEP 回读 |
| --- | --- | --- | --- | --- |
| `box-through-hole` | 1 个长方体 + 1 个贯穿孔 | 最大 0.031251 mm | 0.870324% | 有效，实体数与体积保持 |
| `l-bracket-gusset` | 2 个长方体 + 1 个三角棱柱 | 最大 0.041867 mm | 1.158129% | 有效，实体数与体积保持 |

`box-through-hole` 的孔径为 11.784618 mm，孔径误差 0.215382 mm；体积相对误差 0.341081%。`l-bracket-gusset` 的 X / Y / Z 尺寸误差分别为 0.041867 / 0.029760 / 0.014475 mm，独立体积采样的保守上界为 1.908613%。两组数值均为 2026-10-04 参考运行。

验收门槛固定为各轴尺寸误差 ≤0.5 mm、孔径误差 ≤0.5 mm、对称差体积 / 源体积 ≤5%。通过还要求独立体积采样的置信上界同样 ≤5%，不能只依赖可能数值异常的布尔结果。布尔比较使用记录的 0.00001 mm 容差处理近乎重合的面；该设置不放宽 0.5 mm 或 5% 门槛。

STEP 回读还检查回读实体数等于期望值、形状有效、体积相对变化小于 1e-7。报告中的 `accuracy` 字段保留尺寸误差、孔径误差、源边界距离、对称差、布尔原始差值、采样方法和 `difference_method`，便于区分“几何不达标”和“布尔不可信”。

## 耗时与复现环境

`validation_report.json` 记录拍摄、COLMAP 和拟合阶段的 `elapsed_seconds`，`workspace/reconstruction.json` 记录各 COLMAP 子命令的耗时。现有 2026-10-04 记录存在以下限制：

- 每个案例只做了**一次合成虚拟拍摄**，没有重复运行，因此不能给出成功率或方差；
- 两个案例的 GPU 阶段存在重叠执行，耗时不能作为独占 GPU 的性能基准；
- README 未提供归一化的硬件基线，不应把本机耗时直接外推到其他 GPU；
- `l-bracket-gusset` 的通过结果来自对已保存 544,302 点点云的复拟合，没有重新拍摄或重复 COLMAP；
- `--no-gpu` 是 CPU 稀疏路径，与 GPU 稠密路径的点云规模和精度不可直接比较。

因此，本页只声明“在该环境下该次运行完成并通过验收”，不声明跨环境性能排名。需要比较实现变化时，应固定模型、纹理、视角、种子、COLMAP 参数和 GPU 条件，并保留多次运行的报告，而不是复用单次历史数值。

复现与诊断：

```powershell
python -m tools.validation.run_performance_validation --model data/performance-validation/l-bracket-gusset/model.step --texture data/performance-validation/textures/standard_capture.png --output outputs/performance-validation-l-bracket-gusset --gpu
python -m tools.diagnostics.diagnose_performance_validation outputs/performance-validation-box-through-hole outputs/performance-validation-l-bracket-gusset
python -m tools.validation.refit_performance_validation outputs/performance-validation-l-bracket-gusset
```
