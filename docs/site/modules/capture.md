# 虚拟拍摄与相机元数据

本页描述普通单张/环绕拍摄。新增的 Gray Code、相移与多曝光投影拍摄见[编码结构光与投影拍摄](/modules/structured-light)，其采集清单与普通照片分开管理。

## 职责与入口

本模块在 ParamCAD Studio 的 VTK 视口中把 STEP 模型渲染成多视角 PNG，并为每张图片写出相机位姿、视场角、投影方式和输出尺寸等元数据。它不负责图像匹配、COLMAP 重建或点云处理，也不控制真实相机硬件。

入口分为三类：

- 交互拍摄由 `CaptureWorkspaceController` 编排。`capturePhoto()` 调用 `CaptureController::captureSingle(width, height)`；`captureOrbit(distance)` 先从 `capture.conf` 读取环计划，再调用 `CaptureController::captureOrbit(rings, distance, width, height)`。
- `CaptureController` 直接驱动 `VTKViewer`，负责创建时间戳目录、离屏截图、逐张 sidecar 和聚合 manifest。
- 批量验证由 `ParamCAD Studio.exe --performance-capture <step> --output <dir>` 触发，`ModelViewController::runPerformanceCapture` 导入 STEP、可选投影纹理，按同一套环计划拍摄，并额外生成 `camera_info.json` 与 `capture_report.json`。

交互拍摄与批量验证共用同一套 62 视角计划：默认值定义在 `CapturePlan.h` 的 `sphericalCaptureRings()`，运行时由 `capture.conf` 的 7 个环槽位提供；五条 12 张的纬线环加上正上方、正下方各 1 张，共 62 个视角。

## 输入与输出

输入：

- 一个已加载的 STEP/STP 模型，或在视口中已摆好姿态的当前模型；
- 输出宽高，默认 `1920 × 1080`，配置范围为 64–16384；
- 环计划：最多 7 个槽位，每个槽位记录张数与仰角，张数为 0 的槽位被跳过；
- 可选的拍摄距离与纹理图片，距离以模型中心和视口轨道半径为准。

交互拍摄的默认输出目录为 `outputs/captures/`，每次拍摄新建 `<yyyyMMdd_HHmmss>` 子目录；同一秒内重复拍摄会追加 `-2`、`-3` 等后缀。

```text
outputs/captures/<时间戳>/
├── shot_000.png
├── shot_000.json
├── shot_001.png
├── shot_001.json
├── ...
└── manifest.json
```

- 单张模式固定写入 `shot_000.png`，manifest 的 `mode` 为 `single`。
- 环绕模式在一次拍摄内连续编号，manifest 的 `mode` 为 `orbit`。
- `manifest.json` 采用 version 2，是权威聚合文件；每张 `shots[]` 条目包含 `index`、`image`、`sidecar`、`azimuthDeg`、`elevationDeg` 和完整 `camera` 记录。
- 根级 `orbit` 只描述第一环；多环时 `rings[]` 才描述完整纬线计划，字段包括 `count`、`elevationDeg`、`startAzimuthDeg`、`stepAzimuthDeg` 和 `firstShotNumber`。
- 根级 `render {width, height}` 记录渲染窗口原始尺寸，用于还原输出图被拉伸前的横纵焦距比例。
- `shot_NNN.json` 是 version 1 的逐图 sidecar，字段为 `schema`、`version`、`image`、`sidecar`、`group`、`capturedAt`、`camera`、`output` 和可选 `render`。

性能验证批处理直接写入给定案例目录下的 `images/`；若该目录已存在则拒绝覆盖。`CameraInfoBuilder` 只把完整透视元数据转换为 COLMAP JSON：

```json
{ "cameras": { "cam_0": { "model": "PINHOLE", "width": 1920, "height": 1080,
                          "params": [fx, fy, 960.0, 540.0] } },
  "images": [ { "name": "shot_000.png", "camera": "cam_0",
                "camera_to_world": { "qvec": [qw, qx, qy, qz], "position": [x, y, z] } } ] }
```

## 核心数据结构与依赖

- `CaptureShot`：一次拍摄的序号、分组目录、图片名、方位角、仰角、距离和 `CameraParameters`。
- `OrbitRing`：一条纬线环的 `count` 与 `elevationDeg`。
- `CaptureSettings`：输出宽高、7 个环的张数与仰角；`orbitRings()` 过滤掉零张槽位。
- `CameraParameters` / `OrbitParameters`：来自客户端 `OrbitCamera`，记录位置、目标点、上方向、视场角和投影方式。
- `CameraInfoBuildResult`：`built`、`totalImages`、`posedImages`、`cameraMode` 与 `skipReason`。

依赖边界：

- **Qt 6 Widgets**：拍摄面板、目录选择、进度反馈和 `QJsonDocument` 读写；
- **VTK 9.7.0**：相机控制、离屏渲染和 `captureImage()`；
- **OpenCASCADE 8.0.1**：`StepModelLoader` 读取 STEP 并保留 B-Rep；
- **项目自有轨道相机与元数据代码**：`OrbitCamera`、`CaptureController`、`CameraInfoBuilder`。

## 模型定位、多视角采样与相机元数据转换

**模型定位。** `StepModelLoader` 读取 STEP 后调用 `moveCentroidToOrigin()`：有体积的实体按体积质心平移，无封闭体的开放壳按包围盒中心平移。平移只改变位置，尺寸和距离不变；文件原有的装配摆放不会被保留。这样相机始终围绕原点轨道旋转。

**球面采样。** 默认计划在仰角 `-60°`、`-30°`、`0°`、`30°`、`60°` 上各拍 12 张，步进 `360° / 12 = 30°`，再在 `90°` 和 `-90°` 各拍 1 张，合计 62 张。每台相机都对准居中后的模型原点。`captureOrbit()` 在批量开始前保存当前相机姿态，结束后恢复，用户视口不会被留在最后一个采样位。

**环配置。** `capture.conf` 的 `ring1` 到 `ring7` 分别提供 `shots` 和 `elevation_deg`；仰角限制在 `-90°` 到 `90°`，张数限制在 0–512，整份配置至少有一个环为正张数。改变环数可以生成自定义环绕，但 schema 固定只有 7 个槽位，超过 7 环需要扩展配置结构。

**纹理投影。** 纹理不展开 UV，而是由 `BodyActor` 从单一方向投射到模型表面，因此任意图片都能直接贴到任意模型上。批量验证在提供 `--texture` 时把投影轴设为 4（逐面选择方向），让每个侧面都获得可匹配的视觉细节；交互拍摄则沿用界面当前的纹理设置。

**相机元数据转换。** `CameraInfoBuilder` 优先读取 manifest v2，旧的拍摄目录回退到逐图 sidecar。它要求目录中每张可读图片都有可用透视元数据，且图片数不少于 2；缺元数据、部分缺失或存在正交投影时整体放弃先验，让 COLMAP 自行估计相机。转换时：

1. 以 `target - position` 为前向，构造 `+X 右、+Y 下、+Z 前` 的 COLMAP 相机基；
2. 把 `fieldOfViewDeg` 当作垂直视场角，计算 `focalY`，再用输出尺寸与 `render` 尺寸的比例修正 `focalX`；
3. 主点取输出图中心 `(width / 2, height / 2)`；
4. 按 `宽 × 高 | render 尺寸 | 视场角` 签名分组，每组生成一个 `cam_N`，内参不同的图片不会被错误合并；
5. 输出 `camera_to_world` 的 Hamilton 四元数与世界坐标相机中心，由 Python 适配层转换为 COLMAP 外参。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 拍摄控制与元数据写出 | `apps/ParamCAD Studio/src/app/CaptureController.cpp` |
| 拍摄会话与镜头结构 | `apps/ParamCAD Studio/src/app/CaptureController.h` |
| 交互拍摄编排 | `apps/ParamCAD Studio/src/app/CaptureWorkspaceController.cpp` |
| 共享 62 视角计划 | `apps/ParamCAD Studio/src/app/CapturePlan.h` |
| 拍摄配置加载与校验 | `apps/ParamCAD Studio/src/app/CaptureSettings.cpp` |
| 清单到 COLMAP 内参转换 | `apps/ParamCAD Studio/src/io/CameraInfoBuilder.cpp` |
| STEP 导入与质心平移 | `apps/ParamCAD Studio/src/io/StepModelLoader.cpp` |
| 拍摄默认参数 | `apps/ParamCAD Studio/config/capture.conf` |

## 测试与诊断

## 已知限制

- 拍摄计划固定为 7 个环槽位，默认只能用 `ring1`–`ring7` 表达；超过 7 条纬线需要扩展 `CaptureSettings` 与配置文件 schema。
- 相机先验只对虚拟拍摄生成。现实照片没有 manifest 或 sidecar，或包含正交投影视角时，客户端会整体放弃 `--camera-info`，由 COLMAP 估计相机。
- 输出图由渲染窗口拉伸到目标尺寸。旧 sidecar 若没有 `render` 字段，会假定横纵像素为正方形，转换出的两个焦距只能视为近似值。
- 转换只产生 `PINHOLE` 内参，不建模径向畸变；需要 `OPENCV` 等模型时必须在上游自行提供 `camera_info.json`。
- STEP 导入会把模型平移到质心原点，原始装配摆放不保留；依赖世界坐标的相机或外部对齐需要重新建立。
- 桌面端没有相机硬件采集与实时取景，只支持虚拟渲染截图和读取已有图片目录。

