# 贯穿孔与盲孔

孔识别的核心问题是区分“贯穿宿主的圆柱壁”与“只在一侧开口的盲孔”。页面以 `box-through-hole` 标准件展示正向结果，以 `blind_hole` 契约数据和模块回归说明拒绝条件，并把验收指标与参考运行数值分开。

## 标准贯穿孔

源案例位于 `data/performance-validation/box-through-hole/`，`case.json` 是可维护的尺寸与验收定义。

| 项目 | 数值 |
| --- | --- |
| 长方体 X × Y × Z | 60 × 40 × 20 mm |
| 贯穿孔直径 | 12 mm |
| 孔轴与位置 | Z 方向，穿过 (30, 20) |
| 孔深 | 20 mm，完整贯穿 |
| 理论体积 | 48000 − 720π ≈ 45738.053289 mm³ |
| 源模型范围 | X=0..60，Y=0..40，Z=0..20 |

识别入口要求先得到非退化的宿主箱体，再用圆柱证据做分类。以下条件来自 `ThroughHoleRecognizer` 的默认选项与判定代码：

- 圆柱半径为正且有限，轴向范围有效；
- 角向覆盖率 ≥0.70，轴向覆盖率 ≥0.85；
- 观测圆柱两端与宿主两个外表面的距离均不超过宿主厚度的 10%；
- 圆柱轴与宿主某一正交轴的夹角不超过 5°；
- 孔中心到两个横向边界的距离不小于半径加边界余量。

满足条件后输出孔轴、半径、宿主厚度和来源圆柱；置信度综合角向覆盖、轴向跨度与轴对齐程度。只识别出箱体而没有孔，不会被视为完整 CAD 成功。

**参考结果（单次运行，2026-10-04）**：最大外形尺寸误差 0.031251 mm，识别孔径 11.784618 mm，孔径绝对误差 0.215382 mm，对称差体积 0.870324%，体积相对误差 0.341081%。该结果来自 `data/performance-validation/box-through-hole/README.md` 与 `data/performance-validation/README.md`，不是重复运行统计。

| 指标 | 参考值 | 验收门槛 |
| --- | --- | --- |
| X / Y / Z 尺寸误差 | 0.031251 / 0.028236 / 0.016616 mm | 各轴 ≤0.5 mm |
| 识别孔径 | 11.784618 mm | 误差 ≤0.5 mm |
| 体积相对误差 | 0.341081% | 不单独作为门槛 |
| 对称差体积 / 源体积 | 0.870324% | ≤5% |

参考运行还要求参数化 B-Rep 有效、STEP 导出后可回读，且源模型精度检查通过。报告中的 `brep_valid`、`step_round_trip_verified`、`complete_cad_fit` 分别对应这些断言。

## 盲孔拒绝用例

`m13.blind_hole.reject_short_wall` 使用 `blind_hole` 数据集：宿主为 8 × 6 × 8 mm 箱体，孔半径 1.5 mm，孔底在 z=4，开口在 z=8，圆柱壁只观测到 z=4..8。模块 12 的宿主识别为真，但模块 13 的断言是 `hole_count=0`、`wall_count≥1`。

拒绝原因可以直接由判定条件解释：

- 宿主厚度为 8 mm，圆柱轴向跨度只有 4 mm，轴向覆盖率 0.5，低于 0.85；
- 圆柱上端到达 z=8，但下端 z=4 与宿主下表面 z=0 相差 4 mm，远大于 8 × 10% = 0.8 mm 的端点容差；
- 因此该圆柱被保留为“壁”证据，而不是贯穿孔候选。

“短壁”指圆柱证据没有覆盖宿主厚度；“缺端面”指观测圆柱端点到宿主外表面的间隙超过端点容差。两者都不允许通过延长圆柱或假定另一侧开口来补齐。

| 场景 | 期望结果 | 依据 |
| --- | --- | --- |
| 盲孔圆柱未到达第二个外表面 | 不作为贯穿孔 | `ThroughHoleTests`：20 mm 厚箱体上 12 mm 深的圆柱被拒绝 |
| 圆柱穿过箱体侧边界 | 不作为包含孔 | `ThroughHoleTests`：孔中心偏出后拒绝 |
| 开口处仍观测到实体材料 | 不生成贯穿切除 | 投影阶段回归：开口材料阻止 through cut |
| 孔洞未同时到达两个端面 | 不推断为贯穿孔 | 装配阶段回归：不完整孔保持开放 |
| 独立圆柱缺少端盖 | 不识别为闭合曲面体 | `m14.open_cylinder.reject_missing_cap`：`recognized=false`、`curved_count=0` |

`m13` 的 `recognized=true` 只表示宿主箱体成立，不表示孔通过；孔是否成立必须看 `hole_count` 与下游 CAD 精度，不能只看单一布尔字段。

## 验收指标

| 检查项 | 门槛或口径 | 性质 |
| --- | --- | --- |
| 外形尺寸 | 各轴误差 ≤0.5 mm | 验收门槛，`case.json` |
| 孔径 | 误差 ≤0.5 mm，且数量与期望一致 | 验收门槛，`case.json` 与精度代码 |
| 对称差 | 对称差体积 / 源体积 ≤5% | 验收门槛，`case.json` |
| 独立采样上界 | 体积采样上界也须 ≤5% | 验收门槛，避免布尔数值异常掩盖差异 |
| 有效 B-Rep | `BRepCheck_Analyzer` 有效，实体数符合期望 | 验收门槛，导出与回读代码 |
| STEP 回读 | 回读后实体数一致、形状有效、体积不漂移 | 验收门槛，`StepModelLoader` |
| 契约 STEP 回读 | `solid_count_preserved=true`，`material_iou≥0.99999` | 验收门槛，`m33` |
| 宿主与孔的组合 | `hole_count=1`，`wall_count≥2`，半径 6.975–7.025 mm | 契约断言，`m13.mixed_plate.through_hole` |

布尔比较使用记录的 0.00001 mm 容差处理近乎重合的面，但这只是数值比较设置，不放宽 0.5 mm 尺寸门槛或 5% 对称差门槛。若布尔结果不可信，报告改用独立体积采样的置信上界；通过仍要求该上界满足 5%。失败时状态为 `cad_accuracy_failed`，并在 `accuracy` 中保留尺寸误差、孔径误差、对称差与采样方法。

复现命令：

```powershell
python -m tools.validation.run_performance_validation --model data/performance-validation/box-through-hole/model.step --texture data/performance-validation/textures/standard_capture.png --output outputs/performance-validation-box-through-hole --gpu
python -m tools.contracts.run_reconstruction_contracts --case m13.blind_hole.reject_short_wall --output outputs/contracts-hole
```
