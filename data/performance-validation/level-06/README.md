# Level 06 · 多实体装配

本地压力测试样例 Y373 位于 `local/Y373/`，包含源模型 `Y373.STEP`、
扫描点云 `cloud.ply`、已有分析包和降采样数据。整个 `local/` 目录由 Git
忽略。ParamCAD Studio 读入源 STEP 后得到 95 个实体和 1,675 个面。

用源 STEP 生成的标准点云应单独保存在运行输出目录，已有扫描点云可用于回放。
