# 超参数配置与 CLI

客户端 0.3.0 起把各阶段默认值从 `*Options` 结构体和实现常量集中到 `config/*.conf`，算法本身不变。客户端通过 **Settings → 超参数** 编辑这些文件；命令行可以复用同一份文件，并用重复的 `--set` 覆盖单项。配置键与分组的完整清单以 `apps/ParamCAD Studio/config/*.conf` 和加载代码为准。

## 配置文件与覆盖顺序

配置文件位于 `apps/ParamCAD Studio/config/`，构建或安装后复制到可执行文件旁的 `config/`。每个模块一份，模块内部按二级标题分组：

| 模块 | 文件 | 当前键数 | 二级分组 |
| --- | --- | --- | --- |
| 拍摄 | `capture.conf` | 16 | 输出、轨道环 |
| 点云预处理 | `preprocessing.conf` | 13 | 降采样、离群过滤、法向与曲率、单位 |
| 曲面检测 | `detection.conf` | 67 | 通用、多尺度、尺度自适应、平面、圆柱、球、圆锥、圆环 |
| 表面分析与显示 | `analysis.conf` | 57 | 最终成面支持、显示网格、局部密度支撑、相对密度孔洞、边界内部支撑、孔槽边界、连接、关系推断、关系优化、局部恢复、界面默认容差 |
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

## 模块 CLI

点云客户端新增的成面支持、孔洞与显示网格参数，详见[表面支持与显示参数](../validation/surface-support-settings.md)。`minimum_surface_points` 默认 12，约束最终点归属后的自动集合及显示网格连通片；它与候选检测阶段的最低支持点数分别生效。保存分析设置会立即重建当前显示网格，候选检测参数在下次识别时使用。CLI 的核心证据索引可以保留较小的候选片段；客户端最终发布与显示时还会执行该门槛。

与客户端一起构建 6 个独立可执行文件，受 `CAD_CLIENT_BUILD_MODULE_CLIS` 控制，默认 `ON`：

| 命令 | 作用 | 主要输出 |
| --- | --- | --- |
| `pcad_preprocess` | 读取 PLY，执行降采样、离群过滤、法向估计 | 处理后的 PLY + JSON 报告 |
| `pcad_detect` | 曲面基元检测 | 证据列表 JSON |
| `pcad_analyze` | 连通面片与关系推断 | 面片、关系 JSON |
| `pcad_recognize` | 常规参数化实体识别 | 候选统计 JSON |
| `pcad_assemble` | 装配面片分解、边界与闭合 | 部件与候选 JSON |
| `pcad_project` | 投影拉伸拟合，可选 STEP 导出 | 拟合质量与操作 JSON |

公共参数：

```text
--in <cloud.ply>          输入点云
--synthetic box           内置 60x40x20 mm 合成盒（冒烟测试）
--out <file.ply>          仅 pcad_preprocess 必需
--report <file.json>      报告路径；省略时输出到标准输出
--step <file.step>        仅 pcad_project
--unit mm|m|arbitrary     覆盖 PLY 单位推断
--config <file>           指定该模块配置文件
--set key=value           覆盖单项，可重复
--print-config            打印实际生效配置后退出
```

典型调用：

```powershell
$exe = ".\apps\ParamCAD Studio\build\msvc-debug\Release"

& "$exe\pcad_preprocess.exe" --synthetic box --out .\outputs\box.ply --report .\outputs\preprocess.json
& "$exe\pcad_detect.exe" --synthetic box --set plane_maximum_planes=3 --report .\outputs\detect.json
& "$exe\pcad_recognize.exe" --in .\outputs\box.ply --unit mm --set box_angular_tolerance_deg=7
& "$exe\pcad_project.exe" --synthetic box --path two-axis-intersection --report .\outputs\project.json
```

`pcad_project` 的 `--path` 接受 `auto`、`single-axis`、`two-axis-intersection`、`segmented`。`pcad_recognize` 与 `pcad_assemble` 要求毫米点云，无法从元数据确认单位时应显式传入 `--unit mm`；`pcad_assemble` 另外会加载投影配置作为回退。退出码为：`0` 成功，`1` 识别、投影或装配结果被判定为不支持，`2` 参数、读取或运行错误。

## 输出与诊断

- `--report` 使用 `QSaveFile` 原子写入；省略时 JSON 输出到标准输出，便于管道处理。
- 报告固定包含 `module`、`config`、`settings`、`input`、`unit`、`elapsed_ms` 和 `status`，各命令再附加自己的计数、覆盖率和评分。
- 识别与装配报告记录计数、覆盖率、部件状态和错误，**不包含可回放的完整特征参数**；需要完整结果时使用 `cad_reconstruction_validation` 的既有报告入口。
- `--print-config` 打印实际生效的完整配置；输出沿用 `.conf` 文本格式，可被自身重新加载，用于核对 `--set` 与环境变量是否生效。
- 拍摄模块没有独立 CLI。离屏拍摄依赖客户端 VTK 视口，入口是 `ParamCAD Studio.exe --performance-capture <step> --output <dir>`；62 视角计划和 `1920x1080` 输出尺寸读取 `capture.conf`，可用 `--config-dir` 指定目录，或用 `--set capture.image_width=...` 临时覆盖。
- `--set` 临时覆盖不会写回文件，也不会显示在设置界面，避免把一次调试覆盖固化成默认配置。
