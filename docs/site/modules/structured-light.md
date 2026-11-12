# 编码结构光与投影拍摄

本模块扩展现有客户端的图片转点云入口。拍摄由 ParamCAD Studio 中的渲染后端完成：默认 VTK 直接光照，也可在拍摄页切换到随包的 Mitsuba 路径追踪；计算由随包 Python 后端完成，不依赖 Blender 或在线下载。渲染接口与重建后端独立。

## 客户端操作

1. 载入或建立模型，切换为透视相机，在“工具 → 拍摄”打开拍摄页。
2. 在“结构光扫描”选择渲染器（VTK 直接光照或 Mitsuba 路径追踪），设置相机和投影分辨率、站数、基线、投影光强及金属度/粗糙度。投影光强是参考距离处的辐亮度比例，提高会让整幅图变亮，过高会在高光处饱和；基线为 0 时按模型尺寸自动选取。
3. 可载入材质基色、金属度和粗糙度贴图。这些材质资产在所有编码帧中固定；投影图案由独立光源产生。
4. 选择“当前视角结构光扫描”或“多站结构光扫描”。可选相移精化及三档曝光；拍摄期间可取消。
5. 成功后，图像重建页自动选中本次采集目录。重建方式保持自动，点击“重建”，客户端调用计算后端并载入毫米单位的点云。

普通照片仍使用原拍摄按钮和 COLMAP。结构光采集目录含 `capture.json`，自动选择结构光后端；客户端禁止把这组变化的编码帧当作普通 SfM 照片输入 COLMAP。

## 渲染与数据边界

`capture::RenderJob` 包含冻结的网格、法线、UV、PBR 通道、设备内外参、扫描站和帧计划，不暴露 VTK 对象或着色器。`CaptureSessionController` 负责会话目录、资产快照、校验和、帧完整性及状态提交；`VtkProjectionBackend` 只执行渲染。

VTK 后端采用独立离屏窗口，在投影器视角建立深度阴影，按投影器矩阵采样编码光，用 GGX 金属度—粗糙度模型计算直接反射。条纹不进入物体基色或自发光。正式拍摄不包含视口网格线、轮廓线和点云叠加，也不改变交互相机。

Mitsuba 后端通过 `QProcess` 调用随包解释器执行 `render_structured_light.py`，使用 `scalar_rgb` 标量 CPU 变体，不需要 LLVM 或 CUDA 运行时。它与 VTK 共用同一 `RenderJob`、针孔设备契约和采集清单；路径深度大于 2 时计算表面间间接反射，输出线性浮点 EXR 及量化 PNG。同一扫描站、同一曝光内的全部图案帧共享随机种子，避免蒙特卡洛噪声破坏编码解码。

相机/投影器均为零畸变、零斜切的 `PINHOLE`，局部 +X 向右、+Y 向下、+Z 向前，图像原点位于左上，像素中心为 `(i+0.5,j+0.5)`。所有位姿是刚性变换，世界单位为毫米。正式成像尺寸和内参直接指定，不从用户窗口尺寸推断。

Gray Code 采用横纵完整正反序列和黑白参考图。可选精化为每轴四步相移，默认周期 16 投影像素。多曝光分别采集增益 0.25、1、4 的完整序列，解码后按像素选择可用且对比度较高的曝光，避免跨曝光拼接编码位。

## 输入、输出与计算

采集目录由客户端生成：

```text
capture/
├── capture.json             # paramcad.structured-light.capture v1
├── calibration.json         # 相机/投影器 K 与 device_to_rig
├── scene.json               # 网格与材质快照，仅供渲染/独立评价
├── assets/                  # 本次材质图片
├── patterns/                # 编码原图与哈希
├── stations/s000/e0/         # 编码拍摄图片
└── events.jsonl
```

`capture.json` 保存每站 `rig_to_world`、设备 ID、图案描述和帧 ID/曝光/相对路径/SHA-256。仅状态为 `complete` 的完整会话可重建；取消会话保留诊断但不会进入成功路径。

`core/reconstruction/structured_light` 读取图片、采集清单和标定，执行明暗差检查、饱和拒绝、编码/相移解码、射线三角化和质量加权体素融合。算法不读取场景网格或渲染真值深度生成点云；`scene.json` 只验证资产标识。

每次重建使用新的目录，必需产物为 `cloud.ply` 和 `result.json`。还保存 `quality.json`、`events.jsonl`、体素观测数及逐站的局部 PLY、解码坐标、置信度、曝光选择、有效掩码与相机光轴 Z 深度。无效深度为 NaN，单位毫米。

`quality.json` 的 `cost` 包含扫描站数、帧数、原图字节数、采集耗时、分阶段计算耗时和 OS 进程峰值工作集；逐站报告的 `exposure_ids` 对应 `exposure_index.npy` 中的索引。固定场景对照、验收证据及物理能力边界见[验证记录](/development/structured-light-validation)。

客户端通过 JSON 行事件读取进度、日志、失败和最终结果。必须同时满足进程成功、结果清单完整、点数非零及 PLY 存在，才加载点云。重建取消先写协作取消标记，必要时停止进程，取消结果不会替换旧点云。

## 开发验证入口

算法验证可独立调用同一后端，正式用户不需要终端：

```powershell
python core/reconstruction/reconstruct_structured_light.py `
  --capture <客户端采集目录> --output <新的重建目录>
```

客户端自身也提供批量验证入口，复用同一拍摄控制器和重建控制器：

```powershell
& 'ParamCAD Studio.exe' --structured-light-capture <model.step> `
  --output <新的案例目录> --scan-stations 8 `
  --scan-size 640x480 --projector-size 256x256 `
  --scan-phase-shift --scan-multi-exposure --scan-reconstruct `
  --scan-renderer mitsuba --scan-samples 32 --scan-max-depth 4
```

入口生成 `capture/`、`reconstructions/` 和独立 `evaluation.json`。评价在点云生成后进行：点到参考三角面的误差、按三角形面积采样的覆盖率、相机/投影器几何共同可见比例及离群点比例。默认评价容差为 0.5 mm，采样 20000 点，随机种子 617；参考是渲染三角网格，不是精确 B-Rep。此容差是报告口径，不是产品精度承诺。

## 离线依赖与测试

计算依赖固定为 NumPy 2.3.5、`opencv-contrib-python-headless` 4.13.0.92，以及 Mitsuba 后端所需的 `drjit` 1.5.0 和 `mitsuba` 3.9.1。打包侧使用 `packaging/structured-light-wheels.json` 核对 Windows x64 / Python 3.13 wheel 哈希，解包至嵌入式 Python；运行时不执行 pip 或下载。

- Python 测试：`core/reconstruction/tests/test_structured_light.py`，含 OpenCV 图案对照、标定/尺度、解析平面图像、多站、多曝光、相移、坏输入及取消。
- 原生渲染测试：`structured_light_render_test`，验证实际投影、遮挡及采集取消，产物可交给同一图片后端。
- 客户端测试：`structured_light_client_test`，通过真实窗口中的按钮执行多站采集、重建、点云载入和两种取消。
- CMake 开关：`CAD_CLIENT_BUILD_CAPTURE_TESTS=ON`。测试程序不进入正式离线包。

## 已知能力边界

- VTK 后端只计算直接光照和投影阴影。Mitsuba 后端在路径深度大于 2 时计算表面间多次反射，可揭示直接光照下不可见的照度传递；提高采样数可降低噪声，但仍不能保证光亮钢件可重建。
- 首版支持基色、金属度和粗糙度常量/贴图，采用冻结的箱式 UV；不支持任意材质节点树、法线贴图或真实位移。
- 扫描站为固定球面分布或当前相机位置；不会自动深入深孔、补全不可见面或从 CAD 真值补点。
- 相移和曝光可改善对应精度/动态范围，但不能保证镜面钢件可重建。覆盖率与误差必须同时检查，不能只看点数。
- COLMAP 与结构光目前共享客户端入口但独立计算；跨后端位姿估计或点云融合仍按[扩展方案](/development/multiview-reconstruction-extension)中的后续条件推进。
