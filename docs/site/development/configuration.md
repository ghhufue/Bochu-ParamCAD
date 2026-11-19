# 超参数配置

客户端 0.3.0 起把各阶段默认值从 `*Options` 结构体和实现常量集中到 `config/*.conf`，算法本身不变。客户端通过 **Settings → 超参数** 编辑这些文件；命令行可以复用同一份文件，并用重复的 `--set` 覆盖单项。配置键与分组的完整清单以 `apps/ParamCAD Studio/config/*.conf` 和加载代码为准。

## 配置文件与覆盖顺序

配置文件位于 `apps/ParamCAD Studio/config/`，构建或安装后复制到可执行文件旁的 `config/`。每个模块一份，模块内部按二级标题分组：

| 模块 | 文件 | 当前键数 | 二级分组 |
| --- | --- | --- | --- |
| 拍摄 | `capture.conf` | 16 | 输出、轨道环 |
| 点云预处理 | `preprocessing.conf` | 14 | 执行、降采样、离群过滤、法向与曲率、单位 |
| 曲面检测 | `detection.conf` | 67 | 通用、多尺度、尺度自适应、平面、圆柱、球、圆锥、圆环 |
| 表面分析与显示 | `analysis.conf` | 77 | 执行、连接、关系与局部恢复、表面支撑验证、显示网格及界面默认容差 |
| 常规识别 | `recognition.conf` | 122 | 分析配额、箱体、通孔、曲面实体、组合体、混合特征、引导模板 |
| 装配分解 | `assembly.conf` | 43 | 采样、容差、面片与闭合、局部恢复、投影回退 |
| 投影拟合 | `projection.conf` | 89 | 输入与候选轴、端面与草图、两轴求交、孔与分段、评分与搜索 |

文件支持 `#` 注释、空行和 UTF-8 BOM。对任意模块 `M`，实际生效值按以下顺序确定：

1. 命令行 `--set` 覆盖：模块 CLI 接受裸 `key=value`，客户端接受 `module.key=value`；
2. 命令行 `--config <file>` 显式指定的文件；
3. 环境变量 `PARAMCAD_<MODULE>_CONFIG`，例如 `PARAMCAD_DETECTION_CONFIG`；
4. 可执行文件旁的 `config/<module>.conf`；
5. 源码树 `apps/ParamCAD Studio/config/<module>.conf`，仅作为开发构建的兜底。

未提供的键使用 C++ 结构体默认值；所有已跟踪的 `.conf` 都写全了键，正常部署不依赖兜底。投影模块保留旧的裸键覆盖语义，例如 `--set maximum_axes=5` 仍只作用于 `projection.conf`；客户端新增覆盖必须写成 `模块名.键=值`，避免跨模块歧义。

加载时对未知键、重复键和越界值直接报错，错误信息包含文件名与行号；跨字段冲突在解析后统一校验并整体拒绝，例如投影端面点数与拟合点数不一致。设置界面保存时只写回被修改的模块，写回前对整份配置校验，并使用 `QSaveFile` 原子替换。构建目录中的配置只在目标文件不存在时复制，因此修改源码 `.conf` 后需删除 `build/.../Release/config/` 或手动复制。

## 线程与性能超参数

在 Studio 的 **设置 → 超参数** 中，线程项位于点云预处理或表面分析页的 **执行** 分组。下列值是源码默认值；已安装版本或本地配置可能不同。

| 配置 | 线程项 | 默认 / 范围 | 作用范围 |
| --- | --- | --- | --- |
| `preprocessing.conf` | `worker_count` | `0` / 0–32 | 离群过滤逐点 KNN 和 PCA/缺失法向；自动最多 8，小点集少用或不用辅助线程 |
| `analysis.conf` | `support_validation_worker_count` | `0` / 0–32 | 曲面面域构建与支撑复核；自动最多 8，实际受面片数限制 |
| `analysis.conf` | `family_detection_worker_count` | `0` / 0–5 | 独立曲面族检测；自动最多为启用族数，混合检测和多尺度残余恢复不受此项控制 |

`0` 表示自动，`1` 表示该阶段强制串行，不是整条重建的全局线程开关。装配分组、局部轮廓恢复和投影搜索目前没有对应的线程数配置。小输入回退条件见[并发与串行状态](/performance/execution-model#线程数与串行回退)。

以下是**工作量或搜索预算**，不是线程开关。调整可能改变检测结果、回退覆盖或显示质量，不应与线程数一起修改后归因于并行：

| 配置 | 当前预算示例 | 调整影响 |
| --- | --- | --- |
| `preprocessing.conf` | `voxel_size_mm=0`、`voxel_diagonal_fraction=0.002` | `0` 使用相对尺度；加粗体素通常减少工作点，也可能丢失小孔槽支撑 |
| `detection.conf` / `recognition.conf` | `multi_scale_rounds=3`、`analysis_curved_maximum_candidates=32` | 分别限制多尺度恢复轮次和常规识别的曲面候选配额 |
| `analysis.conf` | `mesh_point_budget=16000` | 显示网格的采样预算；不是有限面支撑验证的采样上限 |
| `assembly.conf` | `maximum_working_points=200000`、`maximum_local_profile_attempts=48`、`maximum_projection_parts=6` | 分别控制装配工作点、局部恢复尝试预算及投影回退部件数；减少预算可能漏掉可恢复部件 |
| `projection.conf` | `maximum_axes=12`、`maximum_fit_points=24000`、`maximum_score_points=1800`、`search_beam_width=4` | 改变候选搜索、拟合和评分规模，可能改变最终候选 |

要测纯并行收益，先维持预算不变，只改一项线程数。真实样本的测量方法和结果见 [Y373 对照](/performance/benchmarks#y373-全量点云重建-2026-11-18)。

## 与 CLI 配合

命令入口、在线/离线模式、返回值及工作流已拆到独立的 [CLI 章节](/cli/)。本页只维护配置格式、覆盖顺序和设置含义。

```powershell
pcad detect --offline --print-config
pcad detect --offline --in cloud.ply --unit mm --config custom-detection.conf --set plane_maximum_planes=3
pcad analyze --config-dir custom-config --in cloud.ply --unit mm
pcad analyze --offline --in cloud.ply --unit mm --set family_detection_worker_count=1 --set support_validation_worker_count=8
```

`--config` 只选择当前模块；上游仍按各自配置发现规则加载。`--config-dir` 对离线命令设置整套配置目录，环境变量仍可逐模块覆盖。在线命令使用已运行 Studio 的设置，不接受离线配置覆盖。

报告中的 settings 通常只记录当前模块；比较运行结果时还需固定上游配置。预览和成面参数详见[表面支持与显示参数](/validation/surface-support-settings)。
