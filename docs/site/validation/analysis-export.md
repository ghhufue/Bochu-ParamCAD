# 点云识别结果导出与 AI 排查

在点云面板的「更多 → 导出识别结果（AI 分析）…」中选择新目录名，生成 `.pcad-analysis` 目录。导出保存点击时刻的点云、分面、关系、选择及缓存显示网格，不会重新执行识别。写入在后台进行，完成前关闭客户端会取消写入。已有目录不覆盖；失败或取消不会留下半成品目录。

## 从面名称开始

可以向 AI 提供：「请检查这个分析包的平面 406，先看面信息，再按需要用脚本分析。」无需把整个项目、点坐标或映射列表贴进对话。

进入导出目录，使用 Python 3.9 或更新版本，无需第三方依赖：

```sh
python inspect_bundle.py list --filter 平面 --limit 10
python inspect_bundle.py show --name "平面 406"
python inspect_bundle.py show --id 182 --group
python inspect_bundle.py analyze --id 182
```

`list` 和 `show` 只读取 JSON 元数据，即使移走所有二进制文件也可使用。`--group` 额外显示共面分组、边界点与内部点占比及候选区域是否因内部支撑不足而被拒绝。这些是显示网格构建阶段的记录，没有相关记录的面不会凭空补算。

`analyze` 按映射读取指定面的原始观测和缓存网格，输出范围、质心、法向数量、平面残差、网格面积、最长边及使用其他集合点的三角形统计。最大的三个三角形仅输出面积和源点编号，不输出坐标。它可区分「自己的点太稀疏」与「显示网格借用了其他集合的点」。

名称可重名，集合 ID 在该快照内唯一，名称中的数字和 ID 不一定相同。重名时脚本要求改用 `--id`。不应跨不同快照直接比较点编号；先核对 `bundle_id` 和源点指纹。

## 分层文件

| 文件 | 内容 |
| --- | --- |
| `manifest.json` | 格式版本、快照 UUID、时间、源点数量与单位、文档版本、视图、资产校验信息 |
| `surfaces.jsonl` | 面的名称、ID、类型、点数及单面 JSON 路径与校验值 |
| `surfaces/00000182.json` | 单面参数、置信度、拟合残差、失效状态、包围盒/质心、二进制范围、共面组引用 |
| `plane-groups/*.json` | 共面成员和候选区域的边界/内部支撑检查 |
| `relations.jsonl` | 已保存的面间关系 |
| `point-map.bin` | 每个集合对应的源点索引，以及当前选择的索引段 |
| `points.bin` | 全部源点快照的位置、法向、颜色和删除标志 |
| `meshes.bin` | 缓存显示网格、三角形和每个网格顶点对应的源点索引 |
| `inspect_bundle.py`、`README.md` | 独立查询工具及使用说明 |

ID 0 表示未归属点。映射仅包含有效观测；原始点文件保留删除位置，防止删除操作改变后续索引。所有点索引从 0 开始，指向当前源点云快照的行，而非重新排列的有效点序号。若当前点云经过下采样，这里的源点云就是下采样后的工作点云。

## 用脚本进一步分析

```python
from inspect_bundle import Bundle

bundle = Bundle(".")
surface = bundle.surface(surface_id=182)
for point in bundle.points(surface):
    # point: source_index, position, normal, color
    # 在脚本内计算，仅打印需要的统计结果。
    pass
```

显式查看少量原始坐标：

```sh
python inspect_bundle.py points --id 182 --limit 5
```

该入口默认 10 点，单次上限 100 点。`analyze` 不会打印完整点列表。元数据校验和完整文件校验分别为：

```sh
python inspect_bundle.py verify
python inspect_bundle.py verify --full
```

二进制使用小端序。统一 48 字节头格式为 `<8sIIQ16sQ`：magic、版本 1、记录字节数、记录数量、UUID 原始 16 字节、保留 0。点记录 `<6f4BI` 为 xyz、法向 xyz、RGB、删除标志、保留 0；映射为 `<Q`。网格头的记录字节数/数量为 0，各面的 JSON 给出独立范围；顶点记录 `<3dQ` 为拟合位置与源点索引，三角形记录 `<3Q` 为该面网格中的局部顶点索引。

没有法向/颜色时对应字段填零，是否存在以 manifest 中的标记为准。UUID 和 SHA-256 用来检测误混文件和损坏。原始观测位置与拟合网格顶点位置属于不同数据，不能混用。此格式服务于诊断，不承担项目恢复，也不包含完整 CAD 拓扑或识别过程中的所有中间状态。
