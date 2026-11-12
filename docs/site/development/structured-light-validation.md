# 结构光实现验证记录

日期：2026-11-13。本文记录 VTK 直接光照后端与 Mitsuba 路径追踪适配的测量结果，不把软件中的均匀金属参数当作经过测量的真实钢材。完整要求见[扩展方案](/development/multiview-reconstruction-extension)，操作和数据格式见[结构光模块](/modules/structured-light)。

## 固定参数对照

开发验证脚本 `tools/validation/run_structured_light_validation.py` 只调用正式客户端的批量入口。每个案例均由同一客户端完成 STEP 导入、扫描、重建、结果载入和独立评价，没有第二套采集程序。

```powershell
$env:PYTHONPATH = 'D:\code\Bochu-ParamCAD\core\reconstruction;D:\code\Bochu-ParamCAD\artifacts\structured-light-python'
python tools/validation/run_structured_light_validation.py `
  --client 'artifacts/build-structured-light/Release/ParamCAD Studio.exe' `
  --output artifacts/structured-light-validation/comparison-01
```

此处 `PYTHONPATH` 仅供源码开发环境使用；完整离线包使用自带解释器和依赖。输出目录必须是新的目录。默认比较参数在执行前写入 `settings.json`：相机 256×192、投影器 256×256、8 站、融合体素 0.2 mm；相移为每轴四步、周期 16；多曝光为 0.25/1/4。所有案例均为无材质贴图的均匀表面。参数变化仅为编码方式、曝光和明确列出的材质参数。

评价使用渲染三角网格，按面积采样 20000 点、随机种子 617、覆盖/离群容差 0.5 mm，不对结果做拟合对齐。以下值取自 `artifacts/structured-light-validation/comparison-01/report.json`：

| 案例 | 金属度 / 粗糙度 | 点数 | P50 / P95 误差 mm | 全表面覆盖 | 帧数 | 客户端总耗时 s |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 圆柱 Gray 单曝光 | 0.9 / 0.4 | 37767 | 0.70 / 1.46 | 22% | 272 | 4.26 |
| 圆柱相移单曝光 | 0.9 / 0.4 | 16097 | 0.06 / 0.26 | 29% | 336 | 4.10 |
| 圆柱相移多曝光 | 0.9 / 0.4 | 57042 | 0.05 / 0.18 | 95% | 1008 | 10.47 |
| 较光亮圆柱 Gray 单曝光 | 1.0 / 0.15 | 2239 | 0.45 / 1.25 | 2% | 272 | 3.43 |
| 较光亮圆柱相移多曝光 | 1.0 / 0.15 | 3580 | 0.05 / 0.23 | 7% | 1008 | 9.68 |
| 带通孔模型相移多曝光 | 0.9 / 0.4 | 64008 | 0.09 / 0.35 | 89% | 1008 | 11.70 |
| 加强筋模型相移多曝光 | 0.9 / 0.4 | 55741 | 0.14 / 0.63 | 76% | 1008 | 11.51 |

这组案例的相机与投影器几何共同可见比例应结合 JSON 原始记录查看；可见不等于有足够编码信号。数值误差只描述已恢复的点，不能代表缺失表面的误差。相移降低误差，多曝光扩大可恢复信号范围，但较光亮材质仍大量缺测。此结果不支持“条纹可完整重建镜面钢件”的结论。

测试前固定的圆柱诊断门槛为相移多曝光案例 P95 ≤ 0.5 mm、全表面覆盖 ≥ 80%，本轮通过。此门槛只约束这份固定模拟输入；带孔、加强筋和光亮材质按实测记录，不扩展为精度承诺。后端报告还保存照片字节数、采集耗时、解码/三角化/融合耗时和包括 NumPy/OpenCV 在内的 OS 进程峰值工作集。

## Mitsuba 路径追踪对照（2026-11-12 实测）

客户端拍摄页与 CLI 增加可选 `mitsuba` 采集后端：Mitsuba 3.9.1 `scalar_rgb`、Principled 金属度—粗糙度材质、原生 `projector` 光源与 `path` 积分器。VTK 仍是默认后端；两者共用同一扫描计划、采集清单、解码与融合，只替换成像。

图案帧的随机种子按「站在内的曝光组」固定，同一组的所有 Gray/相移帧共享采样路径，因此图案只缩放这些路径的辐亮度，蒙特卡洛噪声在差分与相位里抵消。这不是独立传感器噪声模型：默认配置刻意等价于无噪声合成相机。作为对照，按帧递增种子时 8 spp 的有效解码像素只有 281，坐标误差标准差 0.53 px；同组固定种子后为 23422 与 0.23 px，32 spp 时从 0.35 px 降到 0.107 px。若需要模拟独立噪声，应另加传感器噪声模型，而不是把解码失败误当算法缺陷。

同一计划（8 站、256×192 / 256×256、相移 + 三档曝光、1008 帧）下，由正式客户端完成采集、重建和独立评价：

| 配置 | 案例 | 点数 | P50 / P95 mm | 全表面覆盖 | 离群 | 采集耗时 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| VTK 直接光 | 圆柱 | 57042 | 0.047 / 0.177 | 94.9% | 0.0% | 9.4 s |
| Mitsuba 8 spp | 圆柱 | 91785 | 0.245 / 0.867 | 100% | 21.0% | 25.2 s |
| Mitsuba 32 spp | 圆柱 | 87213 | 0.122 / 0.412 | 100% | 2.1% | 54.0 s |
| Mitsuba 128 spp | 圆柱 | 79557 | 0.064 / 0.217 | 100% | 0.1% | 160.4 s |
| VTK 直接光 | 较光亮圆柱 | 3580 | 0.046 / 0.226 | 6.9% | 0.2% | 7.7 s |
| Mitsuba 8 spp | 较光亮圆柱 | 5433 | 0.344 / 1.046 | 13.0% | 32.9% | 25.6 s |
| Mitsuba 32 spp | 较光亮圆柱 | 5447 | 0.185 / 0.555 | 12.0% | 7.8% | 53.4 s |
| Mitsuba 128 spp | 较光亮圆柱 | 5662 | 0.122 / 0.357 | 11.5% | 0.7% | 158.3 s |
| VTK 直接光 | 带通孔 | 64008 | 0.087 / 0.351 | 89.3% | 1.2% | 9.2 s |
| Mitsuba 8 spp | 带通孔 | 100645 | 0.418 / 1.504 | 95.9% | 42.8% | 26.1 s |
| Mitsuba 32 spp | 带通孔 | 100270 | 0.205 / 0.726 | 98.1% | 14.8% | 53.1 s |
| Mitsuba 128 spp | 带通孔 | 96275 | 0.108 / 0.385 | 98.7% | 1.9% | 150.0 s |

采样数提高后 Mitsuba 向 VTK 精度收敛，并在圆柱与通孔上取得更高覆盖；128 spp 的圆柱 P95 为 0.217 mm（VTK 0.177 mm），代价是采集时间约 17 倍。较光亮圆柱的覆盖率在 8→128 spp 几乎不变（13.0%→11.5%），说明缺测来自金属高光与投影几何，而不是采样不足；不能据此宣称镜面钢件可重建。

反射深度在 `bracket_phase_hdr` 上出现实质差异（32 spp、同一案例）：

| 最大光路深度 | P50 / P95 mm | 覆盖 | 离群 |
| ---: | ---: | ---: | ---: |
| 2（仅直接光） | 0.358 / 1.364 | 75.0% | 36.8% |
| 4 | 0.378 / 1.885 | 62.4% | 39.3% |
| 8 | 0.377 / 1.878 | 62.3% | 39.3% |

深度 2→4 改变白帧 13.4% 的像素（最大差 249），四个受凹角影响扫描站的可解码像素明显下降（例如 10054→8161、12772→8401），深度 4→8 仅再改变 2.8% 像素，指标基本饱和。带通孔模型在 2/4/8 深度下差异很小。这组结果说明：间接反射在当前场景中确实会污染编码信号并降低覆盖率，但在这两个模型中尚未出现需要专用纠错算法的失败模式；更高反射率或更深的凹腔仍需单独验证。

运行方式：

```powershell
python tools/validation/run_structured_light_validation.py `
  --client 'artifacts/build-structured-light/Release/ParamCAD Studio.exe' `
  --output artifacts/structured-light-validation/convergence-m-d2-s32 `
  --renderer mitsuba --samples 32 --max-depth 2 `
  --cases cylinder_phase_hdr,cylinder_bright_phase_hdr,hole_phase_hdr
```

原始 `report.json`、逐案例日志与采集目录不纳入 Git。

## 图像与几何契约

- `core/reconstruction/tests` 共 22 个测试通过：原有 5 个重建辅助测试、11 个结构光测试和 6 个 Mitsuba 适配测试。结构光测试覆盖独立 OpenCV 图案对照、非二次幂尺寸、明暗/饱和拒绝、一般世界位姿、毫米尺度、相机 Z 深度、负深度、错误对应、加权融合、两站相移多曝光、坏输入、必需校验和、取消和 JSON 事件；输出目录位于采集内部时，在写入任何文件前拒绝。Mitsuba 测试覆盖设备模型校验、sRGB 线性化、RGB 读取顺序、黑图/白图投影信号、EXR 与 PNG 一致、固定种子可重复及场景资产被改动时拒绝；没有安装 Mitsuba 时后两项自动跳过。
- `structured_light_render_test` 验证实际 VTK 图像：未遮挡平面中心白黑差 127，遮挡后为 0；移动投影器改变条纹，黑图下材质细节保持固定；背向面不收编码光；钢材观察方向改变成像；相同输入可重复；取消生成终态清单。
- VTK 平面图片再经正式 Python 图片算法重建，100 mm 平面深度中位数 100.00095 mm、P95 绝对深度误差 0.14824 mm、3024 个点。对应 `native-final-01` 与 `native-final-01-plane-reconstruction`；阴影案例中心掩码为无效，金属案例有 188 个点。将同一实际黑图用于所有编码帧的信号消融诊断得到 0 个有效像素；此诊断只证明均匀无编码信号不产生对应，不代表独立实拍。结果保存于 `native-final-01-image-validation.json`。

## 客户端与分发验收状态

本轮通过 7 个 CTest：结构光原生渲染、结构光客户端、Mitsuba 适配协议、原工作区、导航导入、相机交互和相机元数据。结构光客户端测试验证 6 站采集、7337 点载入、毫米元数据、交互相机不变；同一采集成功重建两次并隔离目录、缺帧失败时保留旧点云、单站拍摄及重建（886 点）、拍摄/重建取消。Mitsuba 适配协议测试用桩 worker 验证参数传递、帧清单校验、EXR 记录、取消清理和启动前拒绝，不依赖本机安装 Mitsuba。测试通过真实窗口中的按钮和正式控制器执行。开发机运行客户端集成测试时需设置 `PYTHONPATH=core/reconstruction;artifacts/structured-light-python`，否则系统 Python 找不到 NumPy/OpenCV 会触发 120 秒超时；离线包内由内嵌解释器和 site-packages 提供依赖，不需要该变量。

原有普通拍摄批量入口生成 62 张带稳定表面纹理的诊断图片；经原 COLMAP CLI、已知位姿、sequential matcher 与 GPU SIFT 路径得到 18271 个点。输入为 `ordinary-colmap-regression`，计算日志为 `ordinary-colmap-reconstruction.log`。该测试独立于结构光，未将编码帧输入 SfM。

最终验收包为 `artifacts/releases/0.3.0-structuredlight.20261112.4/ParamCAD-Studio-0.3.0-structuredlight.20261112.4-windows-x64-full-cuda.zip`，429553852 字节（409.65 MiB），SHA-256 为 `537c41718e0dd359b4067f1d9c24f03fd833b3f30012c139b3b6a00fec1b4f85`。包包含 1373 个 ZIP 条目，校验 1372 个文件哈希及 139 个 GUI 二进制依赖闭包。包含 NumPy/OpenCV 的 `site-packages` 为 170774234 字节、ZIP 压缩数据合计 58841876 字节；这些是实际新增 Python 依赖的体积，不能只用 VTK 无额外渲染器推断整个功能没有包体积增量。完整 payload 文件合计 933177378 字节，不含运行数据和 manifest 自身。

实际 ZIP 解压至 `最终 解压 04`，通过 CMD 入口离线安装至 `最终 安装 04`，两份副本均通过哈希、依赖和清洁 PATH 启动检查；未创建快捷方式或修改正式安装位置。随后从安装副本启动客户端，PATH 仅含 Windows 目录，清除 Python/Qt 开发环境变量，完成 6 站拍摄、重建及载入：9431 点、P95 0.34240 mm、覆盖率 65.595%。该冒烟测试使用 128×96 / 128×128，比上文比较案例分辨率低，不能混用覆盖门槛。输出位于含中文和空格的 `离线 最终客户端 04`。

对应日志均在 `artifacts/structured-light-validation/`：`packaging-final-04.log`、`extracted-package-final-04.log`、`install-final-04.log`、`installed-package-final-04.log`、`installed-client-final-04.log`。日志、数据和 ZIP 不纳入 Git。清洁 PATH 验证是在当前开发机上隔离运行，不能称为干净 Windows 虚拟机或物理断网测试。

### Mitsuba 随包集成（2026-11-13）

Mitsuba 3.9.1 与 Dr.Jit 1.5.0 的 cp313 Windows wheel 已加入 `packaging/structured-light-wheels.json`，由打包脚本按 SHA-256 校验后解包到内嵌 Python，不执行 pip 或联网。`render_structured_light.py` 与 `structured_light/mitsuba_renderer.py` 随包分发，Mitsuba/Dr.Jit 的 BSD-3-Clause 许可证从 wheel 原样复制到 `docs/licenses/`；隔离运行时验证新增 `mitsuba` + `scalar_rgb` 加载检查。

新包 `artifacts/releases/0.3.0-structuredlight.20261113.1/ParamCAD-Studio-0.3.0-structuredlight.20261113.1-windows-x64-full-cuda.zip` 为 479073463 字节（456.88 MiB），SHA-256 `5ca453d6d0b9784400c6038a9402884a1efc81953232aae75e4a3b77de6d7b90`，ZIP 条目 2134，校验 2133 个文件。相较上一包：ZIP 增加 47.23 MiB（+11.5%），解压后 1022.36 MiB（增加 132.05 MiB），其中 Mitsuba 115.56 MiB、Dr.Jit 15.63 MiB、Mitsuba 数据 0.34 MiB。GUI 冷启动响应 0.12 s（旧包同法测量 0.10 s），Mitsuba 不在启动时加载。

解压副本与离线安装副本在隔离 PATH（仅 System32，清除 Python/Qt 变量）下各完成一次 Mitsuba 拍摄与重建：2 站、320×240 / 128×128、相移 + 三档曝光、4 spp、最大深度 4，228 帧采集 7.15 s（解压）/ 6.08 s（安装），单帧约 27–31 ms，重建 0.32 s，40353 点。同一参数下 VTK 采集 2.04 s、单帧约 9 ms、15286 点。两份副本点数完全一致，说明固定种子调度在打包环境中可复现。所有数值均为本机测量，不是产品性能承诺。

## 仍未完成的专项

VTK 后端仍固定单次采样和一次表面反射，只计算直接光照及阴影，不具备改变采样数或反射深度的能力；Mitsuba 适配补齐了这项能力，但当前使用 `scalar_rgb` CPU 执行、Principled 材质和固定种子（相关噪声），没有独立传感器噪声、镜头畸变、散焦或 GPU 验证。32 spp 时圆柱 P95 为 0.412 mm、128 spp 为 0.217 mm，采集时间分别约为 VTK 的 6 倍和 17 倍；默认后端仍为 VTK，Mitsuba 作为可选的高质量/物理验证后端。间接反射已在加强筋凹角上表现出覆盖率下降（75.0%→62.4%），但没有专用纠错算法；更光亮钢面、深腔和真实曝光噪声仍需单独验证。真实钢材标定、真实硬件和跨 COLMAP 位姿/结果融合仍需单独验证；离线包体积与隔离运行已在本轮完成测量。
