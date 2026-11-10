# 多视角重建与相机几何

多视角点云由 `core/reconstruction/colmap_reconstruction/pipeline.py` 以无界面方式驱动 COLMAP 4.2 生成，输出彩色 PLY 和记录坐标系、单位与点数的 `reconstruction.json`。本页说明该入口依赖的相机模型、位姿约定、匹配与三角化流程，以及由此产生的尺度与退化边界；COLMAP 内部的优化细节不在本文范围。

## 相机模型与坐标变换

相机内外参通过可选的 CameraInfo JSON 提供，校验实现在 `core/reconstruction/colmap_reconstruction/camera_info.py`。相机模型名与参数个数必须匹配 COLMAP 4.2 的注册表，例如 `SIMPLE_PINHOLE` 为 `[f, cx, cy]`，`PINHOLE` 为 `[fx, fy, cx, cy]`，`OPENCV` 为 `[fx, fy, cx, cy, k1, k2, p1, p2]`。图像名必须是相对输入目录的安全路径，集合必须与目录中可读图像完全一致，宽高也必须与 COLMAP 检测结果一致，否则直接报错而不是静默缩放。

相机位姿采用 COLMAP 的 world-to-camera 约定：

$$
\mathbf{x}_{cam} = \mathbf{R}\,\mathbf{x}_{world} + \mathbf{t},
\qquad
\mathbf{x}_{img} \sim \mathbf{K}\,\mathbf{x}_{cam}
$$

`world_to_camera` 直接给出 Hamilton 四元数 `qvec=[qw,qx,qy,qz]` 与 `tvec`；`camera_to_world` 给出相机朝向四元数 $q_{cw}$ 和世界坐标中的相机中心 `position`，适配层先取共轭得到 $q_{wc}$，再计算 $\mathbf{t}=-\mathbf{R}(q_{wc})\,\mathbf{position}$。每张图必须且只能指定其中一种，或使用裸 `qvec`/`tvec`；四元数会被归一化，零四元数被拒绝。

COLMAP 相机局部坐标固定为 X 向右、Y 向下、Z 向前，项目要求右手坐标系。来自 Y 向上或 Z 向上渲染器的相机变换，需要在生成 CameraInfo 之前完成轴变换；适配层不会猜测上游约定，也不会自动翻转坐标轴。

## 特征匹配与三角化

特征提取由 `feature_extractor` 完成，默认使用 COLMAP 的 SIFT 路径；`--gpu` 同时打开特征提取与匹配的 GPU 开关，默认关闭以保证 CPU 版可运行。提供 CameraInfo 时，提取阶段使用 `--camera_mode 3`，即每张图一个相机，随后把给定内参写入数据库并标记为先验焦距，这样任意标定参数都能安全应用。

匹配阶段只有 `exhaustive` 与 `sequential` 两种模式，分别对应无序图片集合和有序视频帧。项目不显式估计基础矩阵或本质矩阵；匹配与几何校验均交由 COLMAP 的 matcher 内部完成，不作为项目自己的输出。

位姿数量决定后续路径：

- 未提供任何位姿：运行 `mapper` 做增量式运动恢复结构，并从 `sparse` 中选择 `points3D` 文件最大的模型。
- 提供全部位姿：把给定外参写成种子模型，直接运行 `point_triangulator`，位姿保持固定。
- 提供部分位姿（至少两张）：先对种子模型三角化，再运行 `mapper` 并设置 `--Mapper.fix_existing_frames 1`，固定已有位姿继续注册其余图片。

少于两张可读图像直接失败；位姿必须全部省略或至少提供两张，只给一张会被拒绝。

## 稀疏与稠密重建

稀疏路径用 `model_converter` 把最终模型导出为 PLY，保留颜色。稠密路径先运行 `image_undistorter` 生成去畸变图像与 COLMAP 工作区，再运行 `patch_match_stereo`（开启几何一致性）和 `stereo_fusion`（`--input_type geometric`），直接融合出带法向的稠密点云。稠密模式要求 COLMAP 编译时启用 CUDA 或 HIP，CPU 构建只能在稀疏路径工作。

清单 `reconstruction.json` 记录 `camera_mode`（`estimated`、`provided` 或 `partial`）、点云路径、点数、`density` 和 `coordinate_system`。工作目录必须为空，输出文件不得已存在，避免误覆盖；中间数据库、稀疏模型与深度图都保留在工作目录中以便复现。

## 尺度与退化场景

没有给定外参时，单目 SfM 只能恢复相似变换，清单把单位记为 `arbitrary`。给定外参时，点云沿用 CameraInfo 平移的单位；若项目按毫米建模，应传 `--input-unit mm`。代码不会根据零件尺寸反推真实尺度，也不会把 arbitrary 静默当作毫米；进入几何算法前必须由用户确认单位假设。

常见退化条件及其表现：

| 条件 | 表现 | 原因 |
| --- | --- | --- |
| 纯旋转或基线过短 | 三角化点稀少或空洞 | 缺少视差，深度不可观测 |
| 低纹理表面 | 特征点不足，模型残缺 | SIFT 无可匹配描述子 |
| 重复纹理 | 位置错配，点云重影 | 描述子歧义导致错误对应 |
| 内参尺寸不符 | 直接报错 | 宽高与检测结果不一致 |
| 外参约定错误 | 点云扭曲或重建失败 | 坐标系或四元数方向不一致 |
| 图像少于两张 | 直接报错 | 无法同时估计尺度与位姿 |

重建为空点云同样视为失败。尺度、纹理与遮挡的限制不会因为提高匹配阈值而消失，它们决定了后续点云处理与识别模块能看到什么证据。
