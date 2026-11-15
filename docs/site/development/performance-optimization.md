# 性能优化与验证记录（2026-11-15）

::: tip 分主题阅读
本页保留原始实验记录。已按主题整理为独立的[性能调优栏目](/performance/)，可直接查看[并发与串行状态](/performance/execution-model)、[渲染与交互](/performance/rendering)、[预处理与曲面识别](/performance/preprocessing-recognition)、[结构光与 Python](/performance/structured-light-python)及[性能基准与正确性验证](/performance/benchmarks)。
:::

本次完成预处理并行化、结构光法向分块、OCCT 去除重复构建，以及视口交互和后台任务优化。Windows MSVC Release 构建成功，38 项 C++ CTest 与 28 项 Python 重建测试全部通过。

机器为 Intel Core i7-14700KF（20 核、28 逻辑处理器），Python 3.13、NumPy 2.3.5。基线提交为 `1ce2478929a9edfc9600f8d5161aee116ce4036f`。测量期间不并发编译、跑测试或运行其他本次基准；系统后台活动仍会造成波动。

完整数值和校验摘要保存在仓库 `docs/performance/2026-11-15.json`；原始日志、JSON、点云回放及隔离基线构建保存在 `artifacts/performance-optimization-20261115/`。

## 实测收益

| 场景 | 优化前 | 优化后 | 加速比 |
|---|---:|---:|---:|
| 10 万点完整预处理 | 1015.04 ms | 559.97 ms | 1.81× |
| 40 万点完整预处理 | 5155.33 ms | 3191.88 ms | 1.62× |
| 长方体及六个贯穿孔的 CAD 构建 | 26.18 ms | 17.63 ms | 1.48× |
| 拖动时材质预览帧耗时 p50 | 11.05 ms | 3.04 ms | 3.63× |
| 拖动时材质预览帧耗时 p95 | 11.83 ms | 3.69 ms | 3.20× |
| 1280×960 结构光法向估计 | 4.958 s | 1.536 s | 3.23× |
| 同一法向基准的进程峰值内存 | 2.376 GB | 0.570 GB | 减少约 76.0% |

### 测量边界

- C++ 预处理包含离群点过滤、局部 PCA、按有向观测传播法向和输出指纹检查。输入是规则曲面网格，20% 点携带有向法向，另含远处离群点。每项预热一次、测量三次，取中位数。自动模式使用最多 8 个预处理工作线程，方向传播保持串行。
- CAD 基准包含创建模型、提交六孔特征、B-Rep 有效性和体积检查。预热一次、测量三次取中位数。实体体积保持 `89968.1421051 mm³`，与解析值误差要求小于 `1e-5 mm³`。
- 渲染基准使用 800×600 离屏视口和长方体，连续改变相机，记录 40 帧的墙钟时间。交互期间阴影临时从 1024² 降为 512²；结束或截图时恢复 1024²。完整质量路径的 p50 为 10.94 ms，不能把交互收益解释成复杂模型、相同阴影分辨率下的全局 GPU 加速。
- 法向基准在独立 Python 进程中比较原实现和新实现，包含深度台阶及 NaN 缺口。每项预热一次、测量三次取中位数；内存是 OS 记录的整个进程峰值工作集，包含输入、输出、临时数组、解释器和本地库。两种尺寸、单线程分块和自动并行的输出 SHA-256 均与原实现相同。
- 512×384 法向估计从 0.706 s 降为 0.255 s。1280×960 使用 `workers=1` 时约 4.892 s、峰值 0.266 GB，可用于更重视内存或需要限制并发的调用方。

这张表衡量具体计算和交互路径，不能相乘得到整个 CAD 重建流程的加速比。

## 实际扫描回放

另对已有 8 站、256×192、1008 帧结构光扫描做了当前代码回放，三次均输出 79,372 点。PLY 文件与优化前保存的回放逐字节相同，SHA-256 为：

```text
93a2dc97c8d207c8c2d7087776ec4d1f28dbcd86f24ff796bfe653f26fc9bfcd
```

首次回放为 4.52 s，其中文件/清单校验约 3.46 s；随后热缓存回放约 1.26 s。进一步交替使用原、新法向函数做三对同条件回放，整体中位数分别为 1.31 s 和 1.25 s，单次结果存在交叉。该低分辨率样本沿用小图串行路径，图像解码及文件校验占比较高，因此不宣称这一整条链路有显著加速。本次未重新拍摄或运行 COLMAP。

## 已落实的实现

### 确定性的 C++ 预处理并行

`common/Parallel.h` 将独立索引区间分给有界工作线程，连接所有线程后传播异常。输入较小时保持串行。统计离群点的 KNN 和局部 PCA 并行执行；每个点只写自己的结果，均值、方差、计数和法向方向传播仍按原顺序汇总。

新增 `preprocessing.conf` 的 `worker_count`，范围 0–32：

- `0`：自动选择，最多 8 个线程。
- `1`：串行执行，便于诊断和资源控制。
- 其他值：指定上限，仍受输入规模限制。

不改变 KNN 邻居排序、PointId、法向方向门限、曲率和离群点过滤阈值，也没有并行共享可变的 CGAL RANSAC 输入。

### OCCT 只构建一次布尔结果

Fuse、Cut、Common 的双 Shape 构造函数原本已执行 Build，随后代码再次 Build。现在统一为空构造、设置输入和选项、执行一次 Build；同时处理带 fuzzy tolerance 的轮廓融合路径。保留原来的实体检查、失败回滚和布尔串行设置。

### 结构光法向分块

保留原来的 5×5 PCA 算法、可靠性判定及朝向相机的符号规则。按行切分，每块保留两行 halo，目标块大小不超过约 65,536 像素，最多 4 个线程同时计算。取消时取消未开始的任务，并等待当前小块收尾。小图保留原来的直接计算路径。

### 视口渲染与高亮

- 相机交互时使用 512² 阴影，停止后强制恢复完整质量帧。
- 拍摄与截图始终使用 1024² 阴影；已完成渲染后读取像素时关闭重复 Render。
- 选中点使用独立的稀疏 overlay；集合基础颜色按文档版本缓存。只改变选择时不再重写所有点的颜色。
- overlay 不参与拾取，原始点编号保持稳定；隐藏、清空和重新加载点云会正确处理高亮状态。
- 将选择变化的渲染请求合并到事件循环。

渲染测试检查阴影分辨率切换、非空图像、交互图像偏差，以及恢复完整质量后的像素逐字节一致。拾取、相机输入和截图最终姿态由实际 Qt/VTK 交互测试覆盖。

### 后台工作与数据复制

- 降采样的源文件读取、排序融合和原子写盘放到工作线程。完成后校验文档版本，取消或期间发生编辑时不替换当前点云；保留单位 sidecar、缓存和恢复原始点云行为。
- 曲面关系推断使用独立的坐标、集合和删除标记快照。主线程只在文档版本和容差均匹配时提交结果，继续保留手动关系、抑制记录及稳定关系 ID。
- 点云分析线程通过 move 捕获计算输入和索引，避免再次复制完整数组。
- 窗口关闭时等待工作线程退出；合作取消也在结果交付时复查，避免已经完成但尚未交付的旧结果覆盖新状态。

最终点云载入、VTK 连接/上传、CAD 提交和部分 UI 刷新仍有主线程工作。本次没有宣称完全消除所有界面停顿。

## 正确性验证

最终 Release 构建完成后，CTest **38/38 通过**（83.22 s）；Python 重建测试 **28/28 通过**。

新增或加强的验证包括：

- 串行、指定线程数和自动线程数下，预处理点指纹、曲率、过滤阈值及全部统计完全一致；也覆盖反转输入顺序。
- 法向块边界、NaN/Inf 缺口、深度边缘、退化点图、参数校验、取消及输入数组不被修改。
- 选择高亮不会改写基础颜色，集合编辑会刷新颜色，截图恢复源颜色，拾取编号不变。
- 后台降采样完成、原始点云恢复、期间编辑导致结果失效，以及取消后保留当前点云。
- 关系快照计算不修改文档，不受后续编辑影响，并拒绝数组不对齐的快照。
- 现有长方体、孔、曲面识别、装配、投影、参数编辑、工程保存恢复、语义面和结构光客户端回归。

初次完整回归出现的 4 项失败，在隔离的原始提交上全部复现。原因是测试硬编码英文菜单、平面标签和错误文案，而当前源语言为中文。修正为稳定 QAction 对象标识或 Qt 当前语言的译文，保留原有几何、校验和、文件往返和拾取断言，没有放宽容差或跳过用例。

## 复现与启动

下面命令从仓库根目录运行，复用本机已经配置的 `artifacts/build-structured-light`；新环境先按开发环境文档配置 Qt、VTK、OCCT 和 vcpkg。

```powershell
cmake --build artifacts/build-structured-light --config Release --target native_performance_benchmark rendering_performance_benchmark --parallel 8
& artifacts/build-structured-light/Release/native_performance_benchmark.exe
& artifacts/build-structured-light/Release/rendering_performance_benchmark.exe
& artifacts/build-structured-light/Release/rendering_performance_benchmark.exe full
python tools/benchmarks/compare_normals.py --baseline-ref 1ce2478929a9edfc9600f8d5161aee116ce4036f --output artifacts/normals-performance.json
```

`compare_normals.py` 从指定 Git 提交读取原函数，使用独立进程比较耗时、进程峰值和输出哈希，不改写工作区文件。原生优化前数据是在修改计算实现前，用同一个新增基准目标构建、运行并保存的。

客户端和 Python 集成测试需要结构光依赖。本机使用仓库已有包目录，无需改动系统 Python：

```powershell
$repoRoot = (Resolve-Path .).Path
$env:PYTHONPATH = "$repoRoot/core/reconstruction;$repoRoot/artifacts/structured-light-python"
ctest --test-dir artifacts/build-structured-light -C Release --output-on-failure -j 1
python -m unittest discover -s core/reconstruction/tests -v
```

使用已构建的优化版客户端：

```powershell
python tools/pcad.py studio --exe "artifacts/build-structured-light/Release/ParamCAD Studio.exe"
```

## 后续边界

本次优先完成可直接验证的改动。多尺度 RANSAC 的区域/曲面族任务池、投影候选前缀缓存、精确 B-Rep 评分的粗筛、GPU 内部阴影传递和大点云显示 LOD 尚未实现；这些需要额外的分阶段数据和更复杂场景的几何/图像验证。没有通过取消恢复轮次、减少曲面族、放宽几何验收或整体替换 Python 来获取上述收益。
