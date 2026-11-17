# 点云输入与预处理

所属主线：[模块1：曲面分割的输入准备](/modules/point-cloud-to-cad)。本页负责单位、采样和法向；下一步由[曲面分割与几何类型拟合](/modules/surface-detection)建立曲面证据。

## 职责与入口

本模块把 PLY 文件或客户端活动点云转换为统一毫米尺度的不可变观测，并完成体素降采样、统计离群过滤、缺失法向估计、曲率估计和方向传播。核心实现位于 `core/geometric_reconstruction`，不依赖 Qt、VTK 或 PCL；Qt 只出现在客户端适配层。

入口分为两层：

- 客户端通过 `PointCloudAdapter::lengthUnitFor()` 从点云清单推断单位，用 `toPointStore()` 建立 `PointStore`，再由 `preprocessingOptionsFor()` 生成参数并调用 `preprocessPointCloud()`；
- 库内提供两个重载：直接接收 `PointStore`，或接收已统一尺度的 `ObservationSnapshot`；
- 独立命令行 `pcad preprocess` 读取 PLY，执行同一流程，输出处理后的 PLY 与 JSON 报告。

## 输入与输出

输入：

- `PlyCloud`：位置、可选 `nx/ny/nz` 法向、可选颜色，以及活动点快照的原始行号 `sourceRows`；
- 单位：`mm`、`m` 或 `arbitrary`。客户端从点云旁的 `reconstruction.json` 读取；直接打开的 PLY 没有元数据时保持 `arbitrary`，不会被静默当作毫米；
- `PointCloudPreprocessingOptions`；
- `protectedPointIds`：人工模板或选区点，必须穿过所有过滤仍可按原 ID 找回。

输出：

- `PointStore`：ID 非零且唯一，位置有限，法向与曲率可选；默认把已知米制转换为毫米，任意单位保持 `Arbitrary`；
- `PointCloudPreprocessingReport`：记录点数变化、过滤与法向统计。

报告字段：

| 字段 | 含义 |
| --- | --- |
| `inputPointCount` / `voxelPointCount` / `outputPointCount` | 输入、降采样后、最终点数 |
| `removedByVoxel` / `removedAsOutliers` | 体素与离群过滤移除数 |
| `normalsFromInput` | 输入自带且可归一化的法向数量 |
| `normalsEstimated` | PCA 新估计的法向数量 |
| `estimatedNormalsOriented` / `normalsWithoutOrientation` | 成功定向与仍无方向的数量 |
| `degenerateNeighborhoods` | 无法定义法向的退化邻域数量 |
| `pointsWithCurvature` | 得到曲率的点数 |
| `appliedScaleToMillimeters` | 米制输入使用的统一缩放，毫米与任意单位均为 1 |
| `outlierMeanNeighborDistance` / `outlierMedianNeighborDistance` / `outlierDistanceThreshold` | 过滤统计与实际阈值 |

## 核心数据结构与依赖

- `PointSample`：`PointId`、位置、可选法向、置信度、可选曲率与 `normalOrientationKnown`。
- `PointStore`：不可变源观测集合，构造时拒绝零 ID、重复 ID、非有限坐标和越界置信度；检测结果只引用 `PointId`，不改写点集。
- `ObservationSnapshot`：只做统一尺度变换的全量观测，保留源单位、`sourceToWorkingScale()` 和观测指纹。
- `PointCloudPreprocessingOptions`：体素、离群、法向、方向传播与单位转换参数。
- `PointCloudPreprocessingReport`：上表中的诊断计数与阈值。

依赖：

- **项目 `Math3`**：协方差与对称特征分解；
- **项目 `KdTree`**：K 近邻查询；
- **Qt 仅限客户端**：`PlyReader` 与 `PointCloudAdapter` 使用 Qt 文件接口，并负责模块间类型翻译。

## 单位归一、采样、离群过滤与法向估计

**单位与快照。** 已知米制输入在 `ObservationSnapshot` 构造时按 `1000` 统一缩放到毫米；任意单位保持比例 1，并继续标记为 `Arbitrary`。`PointStore` 的所有 ID 来自原文档行号加一，活动点删减和跳过非有限坐标都不会给剩余点重新编号。

**稳健包围尺度。** 客户端在点数不少于 100 时，对每个坐标轴各去掉 1% 尾部后计算包围盒对角线；不足 100 点则使用完整包围盒。这个尺度只用于选择分辨率和容差，不会删除任何观测。显式 `voxel_size_mm > 0` 时优先使用配置值，否则体素尺寸取 `对角线 × voxel_diagonal_fraction`，默认比例为 `0.002`。

**体素降采样。** 没有法向时，每个体素保留离体素中心最近的原始观测；有法向时，先按固定种子划分兼容局部表面，要求方向状态一致、法向夹角不超过 30°、双向点到切平面距离不超过体素边长的四分之一，每组独立选代表点。已定向的相反法向不会合并；未知方向只比较无符号对齐度，且不会从其他组继承方向状态。距离相同时取较小 `PointId`，因此代表点始终是原始点，ID 继续有效。受保护点绕过体素直接保留；坐标过大、无法映射到整数体素格的远处点也保留下来交给离群过滤，而不是在浮点转整数时溢出或丢失 ID。输出按源顺序排序，保证结果可复现。拆分保留了薄壁两侧与平行层的独立法向，供后续[平面支持连通扩展](/algorithms/primitive-fitting#plane-support-recovery)使用。

**统计离群过滤。** 默认启用，近邻数 12，标准差倍数 2.5。对每个点计算到最近邻的平均距离，阈值先取 `全局均值 + 2.5 × 标准差`；再用中位数与 MAD 设置稳健上限：

```text
threshold = min(threshold,
                max(3 × median,
                    median + 2.5 × 1.4826 × MAD))
```

远端噪点会同时抬高均值和方差，稳健上限防止它们掩盖更近的离群点。点数少于 4 或近邻数为 0 时跳过该阶段；受保护点即使在阈值之外也保留。

**法向与曲率。** 输入法向先归一化并保留原有内外方向。邻域至少 3 个点、点集至少 4 个点时，对 18 个近邻加自身构造协方差矩阵，取最小特征向量作为法向，曲率为 `λ0 / (λ0 + λ1 + λ2)`。若 `λ1 ≤ max(1e-18, λ2 × 1e-8)`，邻域退化为直线或重合点，不生成法向。

**方向传播。** PCA 只给出方向，不给出材料内外。实现从已有带符号的观测出发，建立多源最短路径森林，沿切向兼容的近邻传播符号；门限为法向对齐度不低于 0.75、切向偏离不超过 0.55 倍距离、相邻间距比不超过 4，距离代价权重为 0.001。输入符号不会被改写，没有带符号根的连通分量保持 `normalOrientationKnown = false`。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 预处理公共接口与报告 | `core/geometric_reconstruction/include/reconstruction/preprocessing/PointCloudPreprocessor.h` |
| 预处理组合与单位换算 | `core/geometric_reconstruction/src/preprocessing/PointCloudPreprocessor.cpp` |
| 体素降采样 | `core/geometric_reconstruction/src/preprocessing/Downsampling.cpp` |
| 统计离群过滤 | `core/geometric_reconstruction/src/preprocessing/OutlierFilter.cpp` |
| PCA 法向、曲率与方向传播 | `core/geometric_reconstruction/src/preprocessing/NormalEstimation.cpp` |
| 观测存储与指纹 | `core/geometric_reconstruction/include/reconstruction/data/PointStore.h` |
| PLY 读取 | `apps/ParamCAD Studio/src/io/PlyReader.cpp` |
| 客户端单位与选项适配 | `apps/ParamCAD Studio/src/io/PointCloudAdapter.cpp` |

## 已知限制

- `Arbitrary` 单位没有物理换算，预处理与检测中的毫米阈值对它没有物理意义；客户端会在未知尺度时要求用户确认。
- 1% 截断的包围尺度只决定体素分辨率，不会移除远端点；极端离群点仍要依赖统计过滤，且点数少于 100 时退化为完整包围盒。
- PCA 只给出法向方向，不建立材料内外；没有带符号观测的连通部件保持方向未知。混合输入按空间连通分量处理，有充分支持的独立有向部件可以生成部分结果，未重建点 ID 保存在 `CompoundCandidate::unresolvedPointIds`；同一部件内的未知方向仍会阻止常规建体。CAD 失败时客户端保留并显示中间检测面片。
- 法向估计要求点集至少 4 点、近邻至少 3 点；直线、重合点和过薄邻域会被判为退化而不生成法向。
- 受保护点只保证不被降采样和离群过滤删除，不会改善其邻域质量；如果剩余点全部受保护，预处理不会产生降采样效果。
- 单位推断依赖点云旁的 `reconstruction.json`；直接打开的 PLY 默认按任意单位处理，避免把未知尺度误当毫米。
