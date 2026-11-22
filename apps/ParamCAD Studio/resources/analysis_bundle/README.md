# 点云识别结果：AI 查询指南

适用于排查集合拟合、共面分组、点归属和显示网格。此目录是导出时刻的不可变快照，不会重新识别，也不是项目恢复文件。

## 执行顺序

先确认快照、单位及数据身份，再按名称定位集合，最后按需要读取单面二进制。`pcad inspect` 只支持离线，不探测或启动 Studio。

```powershell
pcad inspect --bundle . manifest
pcad inspect --bundle . list --limit 10
pcad inspect --bundle . show --id 118 --group
pcad inspect --bundle . analyze --id 118
pcad inspect --bundle . explain --id 118
pcad inspect --bundle . compare --ids 118,119
pcad inspect --bundle . verify --full
```

示例 ID 仅示意，先用 list 查询本包实际 ID。显示名称中的数字与集合 ID 不相同；名称重名时必须用 ID。

使用已安装或构建的 pcad CLI 查询。导出包只包含数据和本指南，不附带 Python 程序；查询不依赖 Python 或正在运行的 Studio。

## 按需读取

- `list` / `show` 只读 JSON 元数据；`--group` 附带共面成员和边界/内部支撑记录。
- `analyze` 按映射读取指定面的观测及缓存网格，只输出统计。当前只有平面残差复算。
- `explain` 输出面来源、支持验证阶段、记录的关系、显示组和单面几何统计。
- `compare --ids A,B` 输出两平面的方程差、支撑重叠、包围盒间距和共同中面探针残差。
- `points --id ID --limit 5` 才显式输出原始坐标；默认 10 点，单次上限 100。
- `verify` 验证元数据；`verify --full` 追加所有资产的大小和 SHA-256 校验。

不要把完整点坐标、映射、二进制资产直接加入 AI 上下文。优先给出单面统计和必要证据。

## 字段与判断边界

`surfaces.jsonl` 是索引，`surfaces/*.json` 保存单面方程、摘要和范围；`plane-groups/*.json` 保存共面分组和成面支撑记录；`relations.jsonl` 保存已记录的关系。

`points.bin` 是原始观测快照，`point-map.bin` 是集合到源点映射，`meshes.bin` 是缓存显示网格及来源。原始观测位置与拟合网格顶点位置不能混用。

源点索引从 0 开始，指向本快照源点云的行；下采样后的源点云不再等同于下采样前点云。跨包比较先核对 bundle_id 和 positions_sha256。

显示网格可能引用其他集合的顶点，需结合 foreign_source_vertices 与 shared_plane 判断。confidence 是拟合置信度，covered_observations 是点数，均不能代替面积或内部支撑检查。没有记录的中间过程不能推断为已验证；arbitrary 单位不能直接当成毫米。

`explain` 与 `compare` 使用 `paramcad.face_diagnostics` schema v1。新导出的 bundle 会记录 Studio 原始点云支持恢复中的候选对、阶段、结果、拒绝原因及使用的门槛，不覆盖初始 RANSAC 或工作点云检测。`source_recovery_proposal_links` 将所有权前验证的局部 proposal ID 映射到恢复 proposal ID。`trace_available`、`candidate_linked`、`matching_attempt_found` 和 `trace_truncated` 分别表示 trace 存在、所选面关联恢复候选、该候选有合并尝试事件、事件是否因上限而截断。旧 bundle 可能没有 trace 或恢复候选映射。比较报告中的中面残差是对导出观测的事后诊断探针，不是检测器实际合并拟合。

`compare` 还会给出 `same_recovery_proposal` 和 `matching_attempt_found`。不同最终面若映射到同一恢复 proposal，不能归因成这两个最终面之间的一次合并拒绝；查看 `explain` 中按恢复 proposal 映射的前置验证决策。事件列表为空且 trace 已截断时，不能断言该尝试不存在。
