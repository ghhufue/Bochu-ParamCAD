# 从多视角图像生成点云

## 准备图像

图像重建由 `core/reconstruction/reconstruct.py` 调度本地 COLMAP 4.2.0 完成，输入是一个目录中的多视角图片。

- 至少需要两张 COLMAP 能读取的图片；少于两张会被客户端和流程同时拒绝。
- 图片应有足够纹理和视角重叠，纯色、反光或严重失焦的表面无法提供稳定的 SIFT 特征。
- 使用相机先验时必须按透视投影拍摄；正交投影照片没有等价的 pinhole 内参，客户端会放弃整组先验。
- 图像文件名会写入 CameraInfo 和 COLMAP 数据库，必须与目录中的实际图片集合完全一致，大小写和相对路径都要匹配。
- 无序环绕照片使用穷举匹配，按顺序采集的视频帧使用顺序匹配。

使用 ParamCAD Studio 的虚拟拍摄时，**工具 → 拍摄图像** 会按球面计划渲染 PNG，并写出 `manifest.json`（version 2）和逐张 `shot_NNN.json`。重建优先读取聚合清单，旧目录回退到单张 sidecar。现实照片可直接选择目录，但当前没有相机硬件采集控制。

## 相机先验与重建模式

是否传入 `camera_info.json` 决定重建模式：

| 提供的先验 | COLMAP 路径 | `camera_mode` | 尺度 |
| --- | --- | --- | --- |
| 无相机信息 | `mapper` 估计内参与外参 | `estimated` | `arbitrary` |
| 仅内参 | 应用给定内参，`mapper` 估计外参 | `estimated` | `arbitrary` |
| 全部外参 | 写入种子模型，`point_triangulator` 三角化 | `provided` | 继承 `--input-unit` |
| 部分外参（至少两张） | 先三角化种子，再固定已有位姿注册其余图片 | `partial` | 继承 `--input-unit` |

`CameraInfo` JSON 的结构参考 `camera_info.example.json`。内参模型和参数顺序遵循 COLMAP，例如 `SIMPLE_PINHOLE` 为 `[f, cx, cy]`，`PINHOLE` 为 `[fx, fy, cx, cy]`，`OPENCV` 为 `[fx, fy, cx, cy, k1, k2, p1, p2]`。

位姿可以逐张省略，支持两种明确约定：

- `world_to_camera`：COLMAP 原生的 Hamilton 四元数 `qvec=[qw,qx,qy,qz]` 与 `tvec`，满足 `x_camera = R * x_world + t`；
- `camera_to_world`：相机朝向四元数和世界坐标中的相机中心 `position`，适配层会转换为 COLMAP 外参。

COLMAP 相机局部坐标为 X 向右、Y 向下、Z 向前，项目要求右手坐标系；上游若使用 Y 向上或 Z 向上的渲染坐标，应在生成 CameraInfo 时先做轴变换。客户端从虚拟拍摄元数据构造相机时，写出 `PINHOLE`、`[fx, fy, cx, cy]` 和 `camera_to_world`，平移量使用毫米。

## 运行重建

在 GUI 中打开 **工具 → 重建点云**（**Tools → Reconstruct Point Cloud**）。面板提供图像目录、「使用最近一次拍摄」、输出目录，以及三个选项：

- **使用拍摄位姿与内参**：默认勾选；
- **稠密重建（GPU）**：勾选后同时传 `--dense --gpu`；
- **匹配器**：穷举匹配或顺序匹配。

输出目录默认 `outputs/reconstructions`，每次运行新建 `yyyyMMdd_HHmmss` 子目录，同一秒内重复运行追加 `-2`、`-3` 等后缀。点击「重建」后，运行目录生成 `cloud.ply`、`camera_info.json`（成功构造先验时）和 `workspace/`。构造出相机先验时固定传 `--input-unit mm`；元数据缺失、部分图片缺元数据或存在正交视角时，不传 `--camera-info`，由 COLMAP 自行估计，并在日志中说明原因。

命令行可独立运行：

```powershell
# 自动估计相机，输出稀疏点云
python .\core\reconstruction\reconstruct.py `
  --images .\data\part_images --output .\data\part_sparse.ply

# 提供内参和全部/部分外参，声明平移单位为毫米
python .\core\reconstruction\reconstruct.py `
  --images .\data\part_images --camera-info .\data\camera_info.json `
  --input-unit mm --output .\data\part_sparse.ply

# 有 CUDA/HIP 支持的 COLMAP 才可输出稠密融合点云
python .\core\reconstruction\reconstruct.py `
  --images .\data\part_images --camera-info .\data\camera_info.json `
  --input-unit mm --dense --gpu --output .\data\part_dense.ply
```

默认不使用 GPU 做特征提取和匹配，CPU 版 COLMAP 也能完成稀疏流程；`--gpu` 同时开启 SIFT 特征提取和匹配的 GPU 路径。`--workspace` 未指定时，工作目录默认为输出 PLY 同级的 `<输出名>_workspace`。

## 产物与常见问题

```text
<运行目录>/
├── cloud.ply                    # GUI 输出；CLI 为 --output 指定路径
├── camera_info.json             # GUI 成功构造相机先验时生成
└── workspace/
    ├── database.db              # COLMAP 数据库
    ├── sparse/ 或 dense/        # 稀疏模型或稠密工作区
    ├── colmap.log               # 完整引擎日志与每条命令
    └── reconstruction.json      # 单位、点数、相机模式、坐标系
```

`reconstruction.json` 是客户端读取尺度的依据。`camera_mode` 为 `estimated` 时单位写 `arbitrary`；提供了位姿时写入 `--input-unit` 的值，未显式声明则为 `unknown`，客户端同样按任意尺度处理。后续重建 CAD 时，未知尺度必须由用户确认。

常见问题：

- **工作目录非空**：流程只允许空工作目录，请指定新的 `--workspace`；输出 PLY 已存在时也会被拒绝。
- **图像少于两张**：客户端和流程都要求至少两张可读图像。
- **匹配失败或点云稀疏**：优先检查纹理、视角重叠和运动模糊，再检查匹配器是否选错。
- **内参尺寸不符**：CameraInfo 中的宽高必须与 COLMAP 提取到的图像尺寸一致，否则流程报错。
- **外参方向错误**：确认 `world_to_camera` 与 `camera_to_world` 没有混用，并检查坐标轴变换。
- **稠密模式失败**：`--dense` 要求 COLMAP 编译时启用 CUDA 或 HIP；CPU 版只能完成稀疏流程。
- **找不到 COLMAP**：用 `--colmap` 指定可执行文件，或设置 `COLMAP_EXECUTABLE`，或把运行时放到 `deps/colmap/bin/colmap.exe`。

更多实现细节见 [COLMAP 图像重建](/modules/image-reconstruction) 与 [多视角重建与相机几何](/algorithms/multiview-geometry)。
