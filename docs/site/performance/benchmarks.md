# 性能基准与正确性验证

本页保存 2026-11-15 的基准和 [2026-11-18 Y373 点云重建回放](#y373-全量点云重建-2026-11-18)。早期原始记录见[开发维护的实验记录](/development/performance-optimization)，执行状态见[并发与串行状态](/performance/execution-model)。

## 测试环境和口径

| 项目 | 值 |
| --- | --- |
| CPU | Intel Core i7-14700KF，20 核 / 28 逻辑处理器 |
| C++ 构建 | Windows、MSVC 2022 x64、Release |
| Python / NumPy | Python 3.13 / NumPy 2.3.5 |
| 基线提交 | `1ce2478929a9edfc9600f8d5161aee116ce4036f` |
| 原生与法向耗时 | 预热一次，测量三次，取中位数 |
| 渲染耗时 | 800×600 离屏长方体视口，连续改变相机，40 帧 p50/p95 |
| 测量顺序 | 不并发编译、跑测试或运行其他本次基准；系统后台活动仍可能影响结果 |
| 内存 | 独立进程的 OS 峰值工作集，包含解释器、本地库、输入、输出和临时数组 |

原生预处理是规则曲面网格，20% 点有带符号法向，另含远处离群点。法向点图包含平面斜率、深度台阶和 NaN 缺口。这些不是任意真实机械扫描的性能承诺。

## 优化前后结果

| 场景 | 优化前 | 优化后 | 加速比或变化 |
| --- | ---: | ---: | ---: |
| 10 万点完整预处理 | 1015.04 ms | 559.97 ms | 1.81× |
| 40 万点完整预处理 | 5155.33 ms | 3191.88 ms | 1.62× |
| 六孔 CAD 构建及校验 | 26.18 ms | 17.63 ms | 1.48× |
| 交互预览帧 p50 | 11.05 ms | 3.04 ms | 3.63× |
| 交互预览帧 p95 | 11.83 ms | 3.69 ms | 3.20× |
| 512×384 法向估计 | 0.706 s | 0.255 s | 2.77× |
| 1280×960 法向估计 | 4.958 s | 1.536 s | 3.23× |
| 1280×960 法向进程峰值 | 2.376 GB | 0.570 GB | 减少约 76% |

完整预处理包含离群过滤、PCA、方向传播和指纹检查；CAD 基准包含模型提交、B-Rep 有效性及体积检查。六孔实体体积保持 `89968.1421051 mm³`，解析值误差要求小于 `1e-5 mm³`。

交互预览临时把阴影从 1024² 调为 512²，完整质量路径 p50 为 10.94 ms。不能把交互数字理解成相同阴影质量下的渲染加速，也不能将这些独立加速比相乘，计算整条重建链路的收益。

## Y373 全量点云重建（2026-11-18）

输入为本地 Y373 原始点云（460,488 点，毫米），不是 20 mm 降采样版。Intel Core i7-14700KF（28 逻辑处理器），Windows / MSVC 2022 x64 Release，使用 `cad_reconstruction_validation reconstruct-cloud`。每组是同一个二进制、同一输入、相同配置下只改变一项线程数，各跑一次；表中为墙钟时间，不是多次试验的中位数。

**对照 A：只改变曲面支撑验证线程数。** 曲面族检测在这轮仍为串行。

| 阶段 | 验证 1 线程 | 验证 8 线程 |
| --- | ---: | ---: |
| 工作点验证与索引 | 50.5 s | 11.9 s |
| 原始点复验（含投回工作点再验证） | 80.9 s | 20.6 s |
| ↳ 三轮验证的面域构建合计 | 84.0 s | 13.4 s |
| ↳ 三轮验证的支撑复核合计 | 33.9 s | 5.5 s |
| **整轮离线重建** | **228.1 s** | **129.8 s（−43.1%）** |

面域构建和支撑复核是前两行的子阶段，不应重复相加。原始点复验不是单次检查：它先补证、验证原始点，再把保留证据投回工作点重新验证和索引。大面片的细分探针中，三角化的线程 CPU 时间高于邻域阶段。

**对照 B：只改变独立曲面族检测线程数。** 曲面支撑验证固定为 8 线程。

| 阶段 | 检测 1 线程 | 检测 5 线程 |
| --- | ---: | ---: |
| 独立曲面族检测 | 37.2 s | 19.0 s（−49.0%） |
| 曲面证据收集（含检测与平面拓展） | 39.8 s | 21.6 s |
| **整轮离线重建** | **134.6 s** | **120.0 s（−10.8%）** |

两组对照使用不同版本的二进制，应该组内比较，不能把 A、B 的加速比相乘。B 的 5 线程检测用 47.7 CPU 秒 ÷ 19.0 墙钟秒，约占 **2.51 个逻辑核（整机 9.0%）**；整轮平均约 **9.3%**。这是进程的阶段平均值，不是峰值。5 个任务中平面与球面约 1–1.5 秒就完成，最后剩圆柱跑到约 19 秒。相比之下，最初带探针的串行识别基线整轮约 236.8 秒、占整机约 3.7%，不是 A、B 的配对样本。

两组对照的分组点云均各自逐字节一致；固定种子的独立证据回放还核对了曲面参数、支持点 ID 和证据顺序。Y373 两轮均为 117 个候选、0 个闭合，未进入最终 CAD 构建与 STEP 导出；数字衡量的是**本例整条离线路径到返回结果**的耗时，不是成功导出实体的耗时。装配路径仍是约 54 秒量级；验证并行对照中，局部轮廓重试约 38.5 秒，投影回退约 11.1 秒，直接闭合不是主要开销。

本机原始记录：`outputs/y373-surface-profile-20261118-174820/comparison.csv` 和 `outputs/y373-family-profile-20261118-232532/comparison.csv`。各运行目录还有 `trace.log`、`stages.csv` 和 `report.json`；本地输入及运行产物不随网站分发。

### 复现 Y373 线程对照

1. 用已配置的 Release 构建目录启用 `CAD_CLIENT_BUILD_VALIDATION_TOOLS=ON`，构建 `cad_reconstruction_validation`。
2. 将 `data/performance-validation/local/Y373/cloud.ply` 原样复制到新的输出目录；如果工具未从元数据识别单位，在副本旁添加内容为 `{"unit":"mm"}` 的 `cloud.json`。不要改原点云。
3. 复制一份新版本的 `analysis.conf`，设 `PARAMCAD_ANALYSIS_CONFIG` 为它的绝对路径。复现对照 A 时固定 `family_detection_worker_count=1`，验证线程用 `1` 和 `8`；对照 B 固定 `support_validation_worker_count=8`，检测线程用 `1` 和 `5`。
4. 启用 [CAD 阶段探针](/performance/execution-model#cad-阶段探针)，每次写到不同的报告目录。不并发执行两次回放。该样本正常完成但未闭合时，工具返回 `1`，以 JSON 的 `unsupported` 状态和诊断为准。

## 已有扫描图像回放

输入为 8 站、256×192、1008 帧的已有结构光扫描。三次当前代码回放均输出 79,372 点，PLY 与优化前保存的结果逐字节相同：

```text
SHA-256
93a2dc97c8d207c8c2d7087776ec4d1f28dbcd86f24ff796bfe653f26fc9bfcd
```

首次回放 4.52 s，其中校验约 3.46 s；随后热缓存回放约 1.26 s。另交替运行原、新法向函数三对，整体中位数为 1.31 s 和 1.25 s，单次结果有交叉。低分辨率样本未触发法向线程池，且文件读取和图像解码占比较高，因此这里不宣称显著加速，也没有重新拍摄或运行 COLMAP。

## 正确性检查

优化版最终 **38/38 项 C++ CTest 通过，28/28 项 Python 重建测试通过**；CTest 总用时 83.22 s。该结果对应已验证的优化工作树，不代表每次网页构建都会重新执行客户端几何回归。

| 检查对象 | 验证内容 |
| --- | --- |
| 串行/并行预处理 | 点指纹、点序、法向、曲率、过滤阈值和全部统计一致；覆盖反转输入顺序 |
| Python 法向 | 两种尺寸、串行分块及自动并行的输出哈希相同；覆盖边界、缺失点、退化输入和取消 |
| 视口 | 完整质量恢复后像素逐字节一致；拾取编号、基础颜色和高亮状态正确 |
| 后台降采样 | 完成、恢复原始点云、单位保留、取消及编辑期间拒绝过期结果 |
| 关系快照 | 计算不修改文档，不受后续编辑影响，拒绝数组不对齐的输入 |
| 现有业务流程 | 长方体、孔、曲面、装配、投影、参数编辑、工程往返和语义面引用 |

初次出现的 4 项 UI 测试失败在原始提交的隔离构建中全部复现，原因是测试硬编码英文菜单、平面标签和错误文案。改为稳定 QAction 标识或当前语言译文，保留原几何与错误检查断言，没有放宽容差或跳过用例。

## 复现命令

从仓库根目录运行，复用已配置并启用测试目标的 Windows 构建目录：

```powershell
cmake --build artifacts/build-structured-light --config Release --target native_performance_benchmark rendering_performance_benchmark --parallel 8
& artifacts/build-structured-light/Release/native_performance_benchmark.exe
& artifacts/build-structured-light/Release/rendering_performance_benchmark.exe
& artifacts/build-structured-light/Release/rendering_performance_benchmark.exe full
python tools/benchmarks/compare_normals.py --baseline-ref 1ce2478929a9edfc9600f8d5161aee116ce4036f --output artifacts/normals-performance.json
```

`compare_normals.py` 从指定 Git 提交读取基线函数，分别启动进程比较耗时、峰值和输出哈希，不改写工作区。原生优化前数据是在修改计算实现之前，用同一新增基准目标构建并记录的。

客户端/Python 集成测试需要结构光依赖。本机使用已有包目录，测试环境为：

```powershell
$repoRoot = (Resolve-Path .).Path
$env:PYTHONPATH = "$repoRoot/core/reconstruction;$repoRoot/artifacts/structured-light-python"
ctest --test-dir artifacts/build-structured-light -C Release --output-on-failure -j 1
python -m unittest discover -s core/reconstruction/tests -v
```

可复用的基准源代码位于 `tools/benchmarks/`，结构化记录位于 `docs/performance/2026-11-15.json`，本机原始日志和回放位于 `artifacts/performance-optimization-20261115/`。这些仓库路径不表示网页访问者的机器上已安装相关运行时。
