# 曲面分割与几何类型拟合

所属主线：[模块1：曲面分割](/modules/point-cloud-to-cad)。本页说明如何得到平面、圆柱等几何类型和支持点；孔、凸台等特征语义由[特征与实体识别](/modules/feature-recognition)结合宿主和材料判断。

## 职责与入口

本模块把预处理后的点云提议为平面、圆柱、球、圆锥和圆环五类解析基元，并输出带支持点、残差、覆盖率和置信度的 `PrimitiveEvidence`。它是 Qt/VTK 无关的库，只依赖 `PointStore` 和项目自有数据类型。

对外的检测入口是 `detectPrimitiveEvidence(points, options, result, error)`。生产流程通过 `collectSurfaceEvidence()` 汇集分族或多尺度检测结果，并在收集阶段运行自动平面拓展，再交给曲面分析做有限支撑验证、建立面片和关系。CGAL 类型只出现在 `detection/cgal/` 私有后端中，不跨过公开头文件。

## 输入与输出

输入：

- `PointStore`：统一为毫米的观测点，`PointId` 非零且唯一；坐标与法向按源点 ID 对齐；
- `PrimitiveDetectionOptions`：启用哪些基元族、各族距离与支持阈值、半径/张角范围、随机种子；
- 调用方可通过取消回调在轮次边界中止检测。

输出 `PrimitiveDetectionResult`：

- `evidence`：按检测顺序排列的五类证据；
- `unassignedPointIds`：没有被任何基元解释的点。

公开选项结构的初值按毫米给出：平面距离0.1、最少支持50点、最多6个；圆柱、球、圆锥、圆环距离0.1、最少支持100点、最多各4个；法向一致性门槛0.9，随机概率0.01。圆锥张角限制在0.01–1.56 rad，半径上下限可显式收紧。统一检测默认使用种子4500，保证同一输入和选项可复现。

生产流程会加载 `detection.conf`：其中距离和支持点配置的0表示由点云尺度推导，数量上限当前为平面6、圆柱8、球/圆锥/圆环各32。CAD识别和客户端又可按任务覆盖这些值，因此裸选项初值、配置值与一次任务的有效值应分别查看。配置规则见[超参数配置](/development/configuration)。

证据中的公共质量字段：

| 字段 | 含义 |
| --- | --- |
| `supportPointIds` | 该基元的源观测点 ID |
| `meanAbsoluteResidual` / `maxAbsoluteResidual` | 支持点到解析面的距离统计 |
| `coverage` | 支持点占当前检测点集的比例 |
| `confidence` | 由支持度、残差和覆盖综合得到的 0–1 指标 |
| `angularCoverage` | 圆柱、圆锥的角度覆盖比例 |
| `axialMinimum` / `axialMaximum` | 圆柱、圆锥的轴向观测范围 |

## 核心数据结构与依赖

`PrimitiveEvidence` 是 `std::variant<PlaneEvidence, CylinderEvidence, SphereEvidence, ConeEvidence, TorusEvidence>`。五类证据分别携带解析参数：

- `PlaneEquation`：单位法向与偏移，满足 `normal · position + offset = 0`；
- `CylinderEquation`：轴线上离原点最近的点、轴向与半径；
- `SphereEquation`：球心与半径；
- `ConeEquation`：锥顶、轴向与半张角；
- `TorusEquation`：中心、轴向、主半径与管半径。

依赖边界：

- **CGAL 6.2.1 Shape Detection**：一次 Efficient RANSAC 内竞争性提议五类基元；
- **Eigen3**：PCA、最小二乘和矩阵运算；
- 下游 `analysis/` 负责把证据拆成有限连通面片并推断平行、垂直、同轴等关系。

不依赖 Qt、VTK、PCL，也不维护界面状态。

## 解析基元提议、拟合与质量指标

检测与精修顺序：

1. CGAL Efficient RANSAC 在同一个点集上提议五类基元，返回支持点索引；项目侧把索引映射回稳定 `PointId`，避免 CGAL 重排影响外部引用。
2. 归一化解析参数：法向、轴向统一为单位向量；圆柱轴线用轴上点加方向表示；圆锥统一为锥顶加张角。
3. 平面用 PCA 在支持点上精修法向和偏移，并抑制相距很近的重复深度层。
4. 圆柱用支持点法向和截面圆拟合精修轴线与中心，再用全部支持点精修半径；精修按源 `PointId` 顺序求和，保证浮点结果稳定。
5. 球用全部支持点精修球心与半径；圆锥、圆环使用稳健非线性最小二乘。
6. 合并重复证据：轴线和半径一致、较小支持集重叠达到 70% 的圆柱候选合并为支持最强的一个；实心圆柱的近距离重复提议不会被误合并。
7. 拒绝零半径、非有限参数等退化候选。精确点云的箱体棱边可能让垂直法向产生退化圆柱，项目侧在进入 CGAL 连通性计算前拦截，避免除零崩溃。

共享曲面收集入口随后对自动平面运行连通支持扩展：合并相容种子、按局部点间距扩展、定期用累计支持点重新拟合、重新检查边界点，最后再合并重叠结果。距离门槛保持不变，每次更新检查全部支持点并限制整体方向漂移；手动证据保持其原始点归属，材料朝向状态不会因几何归属而改变。原始提议 API 与完整分析 API 因而有不同职责，详见[平面支持连通扩展](/algorithms/primitive-fitting#plane-support-recovery)。

每次检测使用独立的随机流，并在返回前恢复调用方的 CGAL 随机状态。因此相同输入、相同种子与选项可重复得到相同参数，也不受此前其他检测调用影响。

多尺度恢复用于补足首轮遗漏：默认最多 3 轮，初始法向一致性 0.94，恢复轮降到 0.85，邻域 24 点，最低支持下限 3 点。它只在残余未分配点上运行，收敛后把新证据合并回同一结果。

`coverage` 和 `confidence` 是候选筛选依据，不是最终 CAD 接受条件。下游识别还会检查实体完整性，例如圆柱和圆锥是否观测到端面、球和圆环的角度覆盖是否足够。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 公开检测接口与选项 | `core/geometric_reconstruction/include/reconstruction/segmentation/PrimitiveDetector.h` |
| 证据类型与解析方程 | `core/geometric_reconstruction/include/reconstruction/data/SurfaceEvidence.h` |
| 检测调度 | `core/geometric_reconstruction/src/segmentation/PrimitiveDetector.cpp` |
| CGAL 后端 | `core/geometric_reconstruction/src/segmentation/cgal/` |
| 多尺度检测 | `core/geometric_reconstruction/src/segmentation/MultiScaleDetector.cpp` |
| 分族汇集 | `core/geometric_reconstruction/src/surface/SurfaceAnalysis.cpp` |
| 默认阈值 | `apps/ParamCAD Studio/config/detection.conf` |

## 测试与诊断

几何拟合回归包括 `CylinderDetectionTests`、`CurvedDetectionTests`；平面拓展见 `plane_support_recovery_test`，有限支撑检查见 `surface_support_validation_test`。初始提案可能重叠或被拒绝，分析包中的支撑验证报告用于说明后续接受/拒绝原因。

## 已知限制

- 检测能力覆盖五类解析曲面，但完整 CAD 实体识别只覆盖长方体组合、平面加强筋、单长方体贯穿孔，以及独立球、圆柱、圆锥/圆台和完整圆环体；检测到证据不等于能恢复为特征。
- 效率与稳健性依赖法向质量和尺度。法向缺失、方向未定或点云尺度未知时，检测结果不能直接用于自动建体。
- 阈值默认按毫米给出。输入为米或任意尺度时，必须先完成单位归一，否则距离阈值失去意义。
- 缺失面、严重遮挡、薄壁和小于采样分辨率的特征仍会漏检；`coverage` 只能说明已观测部分的拟合程度，不能证明未观测区域。
- 没有原生 RANSAC 兜底实现；CGAL 不可用时该模块无法工作。
- CGAL Shape Detection 采用 GPL-3.0-or-later 或商业许可，闭源分发前需要确认许可方案。
