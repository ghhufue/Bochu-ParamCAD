# COLMAP 图像重建

本页描述普通照片的 COLMAP 后端。客户端已增加自动后端选择；编码采集目录走[结构光后端](/modules/structured-light)，不得将变化的编码帧直接用于 SfM 匹配。

## 职责与入口

本模块把多视角图片目录和可选的 `CameraInfo` 转换为彩色 PLY 点云。它是一个无界面的 Python 调度层，只负责校验输入、编排 COLMAP 命令行、维护工作目录和写出重建清单；它不解释点云中的 CAD 语义，也不直接操作桌面界面。

入口链路为：

- `core/reconstruction/reconstruct.py` 只是启动器，实际调用 `colmap_reconstruction.cli.main`；
- `cli.py` 定义并解析 `--images`、`--output`、`--camera-info`、`--workspace`、`--colmap`、`--dense`、`--matcher`、`--gpu` 和 `--input-unit`；
- `pipeline.reconstruct()` 完成全部阶段调度，并返回 `ReconstructionResult`。

桌面端通过 `ReconstructionController` 以 `QProcess` 启动同一脚本。控制器先向上查找 `core/reconstruction/reconstruct.py` 定位仓库根，再按 `QSettings`、`deps/python/python.exe`、`python`、`python3`、`py` 的顺序选择解释器；程序与参数分开设置，不经过 Shell。运行期间它轮询 `workspace/colmap.log` 的增量内容，把阶段和日志转发给界面；Python 层另写 `progress.json` 供无界面诊断。

## 输入与输出

输入：

- 图片目录，至少包含两张 COLMAP 可读取的图片；
- 输出 `.ply` 路径，文件必须不存在；
- 工作目录，必须为空目录或不存在的路径；
- 可选 `CameraInfo` JSON；其中的图片名必须与 `--images` 下 COLMAP 实际提取到的图片集合完全一致；
- 匹配方式 `exhaustive`（默认）或 `sequential`；
- `--input-unit unknown|mm|m`，只影响已提供位姿时的输出单位。

`CameraInfo` 的内参模型和参数顺序遵循 COLMAP 4.2，常用形式为 `SIMPLE_PINHOLE [f, cx, cy]`、`PINHOLE [fx, fy, cx, cy]` 和 `OPENCV [fx, fy, cx, cy, k1, k2, p1, p2]`。

位姿可按图片省略，但必须满足“全部省略”或“至少两张有值”。单张位姿只能使用以下三种写法之一：

- `world_to_camera`：Hamilton 四元数 `qvec` 与 `tvec`，满足 `x_camera = R * x_world + t`；
- `camera_to_world`：`qvec` 与相机中心 `position`，适配层会取其共轭并换算 `tvec`；
- 直接给出裸 `qvec` 与 `tvec`。

输出目录由客户端创建为 `outputs/reconstructions/<时间戳>/`：

```text
outputs/reconstructions/<时间戳>/
├── cloud.ply
├── camera_info.json          # 客户端成功构建先验时存在
└── workspace/                # database.db、colmap.log、progress.json、
                              # sparse/ 或 dense/、reconstruction.json
```

`reconstruction.json` 的 `schema_version` 为 1，记录 `engine`（固定 `COLMAP 4.2.0`）、`images`、`camera_info`、`camera_mode`、`point_cloud`、`point_count`、`density`、`coordinate_system` 和 `unit`。坐标系为右手系，使用 COLMAP 重建世界帧。

## 核心数据结构与依赖

- `Camera`：`key`、`model`、`width`、`height` 与参数元组，`model_id` 由 `CAMERA_MODELS` 查表得到。
- `Pose`：COLMAP 原生 world-to-camera 位姿，使用 Hamilton 四元数 `qvec` 与 `tvec`。
- `CameraImage`：图片名、相机键与可选位姿。
- `CameraInfo`：`cameras` 字典、`images` 字典与 `num_poses` 计数。
- `ReconstructionResult`：点云路径、工作目录、清单路径、点数、`camera_mode` 与 `dense` 标记。

`camera_mode` 有三个值：`estimated`、`provided`、`partial`。它决定清单中的 `unit`：`estimated` 一律写 `arbitrary`；有部分或全部位姿时写 `--input-unit` 的值。

依赖边界：

- **Python 3 标准库**：参数解析、子进程、SQLite、JSON、文件校验；
- **COLMAP 4.2.0**：特征提取、匹配、SfM、三角化和稠密重建，源码固定在提交 `be5e29168d4aff238409d60424812df66aac919f`；
- **CUDA 或 HIP**：只有稠密模式需要，客户端启用 `--dense` 时会同时传 `--gpu`；
- **Qt `QProcess`**：桌面端的启动、取消与日志轮询。

## 相机先验分支、稀疏与稠密重建

**特征提取。** `feature_extractor` 默认关闭 GPU，`--gpu` 同时开启 SIFT 特征提取与匹配的 GPU 路径。提供 `CameraInfo` 时额外传 `--camera_mode 3`，让每张图片拥有独立相机，便于写入任意给定内参。提取完成后脚本读取数据库，要求至少两张可读图片；再用 SQLite 更新 `cameras` 表的模型、宽高和参数，并设置 `prior_focal_length=1`。若 `CameraInfo` 的宽高与图片实际尺寸不一致，或图片名集合不匹配，立即报错。

**匹配。** 默认运行 `exhaustive_matcher`，`--matcher sequential` 改为 `sequential_matcher`，适用于按顺序采集的视频帧。

**四条相机先验分支：**

1. 不提供 `CameraInfo`：由 `mapper` 估计内参和外参，`camera_mode = estimated`。
2. 只提供内参、没有任何位姿：内参先写入数据库，再由 `mapper` 估计外参；`num_poses = 0`，`camera_mode = estimated`，输出单位仍为 `arbitrary`。
3. 全部图片都有位姿：把 `CameraInfo` 写成种子模型 `cameras.txt`、`images.txt` 和空 `points3D.txt`，固定位姿运行 `point_triangulator`，`camera_mode = provided`。
4. 部分图片有位姿（至少两张）：先三角化种子模型，再用 `mapper` 加 `--Mapper.fix_existing_frames 1` 注册其余图片，`camera_mode = partial`。

`mapper` 可能产生多个子模型，脚本按 `points3D.bin` 或 `points3D.txt` 的文件大小选择最大模型。

**稀疏输出。** 默认用 `model_converter --output_type PLY` 导出彩色点云。

**稠密输出。** `--dense` 依次运行：

1. `image_undistorter --output_type COLMAP`；
2. `patch_match_stereo --workspace_format COLMAP --PatchMatchStereo.geom_consistency 1`；
3. `stereo_fusion --workspace_format COLMAP --input_type geometric`，直接写出最终 PLY。

每条 COLMAP 命令及其参数、退出码和耗时都会追加到 `colmap.log`，同时以 `paramcad.colmap-progress` schema 更新 `progress.json`。任一命令失败、PLY 不存在或点数为 0 都会抛出 `ReconstructionError`。

## 源码位置

| 内容 | 路径 |
| --- | --- |
| 无界面启动器 | `core/reconstruction/reconstruct.py` |
| 命令行参数 | `core/reconstruction/colmap_reconstruction/cli.py` |
| 重建阶段编排 | `core/reconstruction/colmap_reconstruction/pipeline.py` |
| CameraInfo 校验与坐标转换 | `core/reconstruction/colmap_reconstruction/camera_info.py` |
| 使用约定与产物说明 | `core/reconstruction/README.md` |
| 客户端进程集成 | `apps/ParamCAD Studio/src/app/ReconstructionController.cpp` |
| 客户端请求与信号 | `apps/ParamCAD Studio/src/app/ReconstructionController.h` |
| 拍摄元数据转先验 | `apps/ParamCAD Studio/src/io/CameraInfoBuilder.cpp` |

## 测试与诊断

## 已知限制

- 没有提供外参时，SfM 结果只有相对尺度，清单单位为 `arbitrary`；不能在未确认尺度的情况下当作毫米输入下游。
- 输出文件必须不存在，工作目录必须为空；脚本拒绝覆盖，重复运行必须换新的 `--workspace` 和 `--output`。
- `CameraInfo` 的图片名集合必须与 COLMAP 实际提取的图片集合完全一致，宽高也必须匹配；多一张、少一张或尺寸不同都会整体失败。
- 只提供内参时虽然固定了标定，但相机位置仍由 `mapper` 估计，尺度依旧不确定，输出单位仍为 `arbitrary`。
- 现实照片缺少 manifest 或 sidecar、元数据部分缺失、或包含正交投影视角时，客户端会整体放弃相机先验。
- 稠密模式要求 COLMAP 在构建时启用 CUDA 或 HIP；当前引擎固定为 COLMAP 4.2.0，且不包含真实相机硬件采集。
