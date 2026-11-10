# 点云采样与法向估计

预处理负责把观测快照变成可计算的工作点，同时保留三样东西：源点 ID、受保护的人工观测、以及每个点是否具有可信材料方向。实现位于 `core/geometric_reconstruction/src/preprocessing`，组合入口是 `PointCloudPreprocessor.cpp` 中的 `preprocessPointCloud`。顺序固定为体素降采样、统计离群过滤、法向与曲率估计、方向传播。

## 体素降采样

`Downsampling.cpp` 使用均匀体素网格。对非保护点，体素键为：

$$
\mathbf{k} = \left(\left\lfloor \frac{x}{v} \right\rfloor, \left\lfloor \frac{y}{v} \right\rfloor, \left\lfloor \frac{z}{v} \right\rfloor\right),
\qquad
\mathbf{c} = (k_x+0.5,\;k_y+0.5,\;k_z+0.5)\,v
$$

每个体素只保留离中心平方距离最小的原始点；距离相同则保留源点 ID 更小者。输出按原始索引排序，因此代表点仍是原始观测，不是插值或平均点。保护点（模板点、人工选区点）不参与体素竞争，始终保留，保证人工约束在降采样后仍可按原 ID 找回。

体素尺寸为 0 时，客户端用 `voxel_diagonal_fraction = 0.002` 乘以包围盒对角线推导；包围尺度来自 1% 截尾后的坐标范围，避免少量远点把零件尺寸“撑大”。实现还会保留无法索引到 64 位体素键的远点，把它们交给离群过滤处理，而不是在浮点转整数时溢出或丢失 ID。

降采样不是特征保持简化：体素大于薄壁、孔壁或小凸台时，这些特征会被整体吞掉；代码不做边折叠、曲率自适应或点平均。

## 离群点判定

`OutlierFilter.cpp` 对每个点取 $k$ 近邻，$k=\min(\text{outlier\_neighbor\_count}, n-1)$，计算平均邻距 $d_i$。全局阈值为：

$$
\tau_{global} = \mu + m_\sigma \sigma,
\qquad
\tau = \min\left(\tau_{global},\; \max(m_m\, m,\; m + m_\sigma \cdot 1.4826 \cdot \mathrm{MAD})\right)
$$

其中 $m$ 是邻距中位数，$\mathrm{MAD}$ 是中位绝对偏差，默认 $m_\sigma=2.5$、$m_m=3$。加入稳健上限的原因是：远处离群点会同时抬高均值与标准差，使纯全局阈值失效；用中位数与 MAD 设一个上限，可以在不反复裁剪合法稀疏边缘的情况下抑制这种掩蔽效应。

判定为 $d_i > \tau$ 且不在保护列表中的点被删除。删除只改变工作点集合，不给剩余点重新编号；报告记录删除数量、平均与中位邻距、实际阈值。代码在点数少于 4 或邻域数为 0 时直接跳过整步过滤。

该判据只能处理稀疏飞点。密集离群块内部互为邻居，平均邻距不高，会被保留；合法的稀疏边缘或薄结构也可能被误删。没有半径滤波、聚类分割或基于颜色的过滤。

## 局部 PCA 与法向定向

`NormalEstimation.cpp` 先归一化输入法向，但不做符号规范化——输入法向携带外向/内向语义，取反会破坏材料占据判断。缺失法向只在 `estimate_missing_normals` 开启时估计；邻域数为 `normal_neighbor_count = 18`。

对点及其近邻构造协方差：

$$
\mathbf{C} = \frac{1}{N}\sum_{i=1}^{N}(\mathbf{x}_i-\bar{\mathbf{x}})(\mathbf{x}_i-\bar{\mathbf{x}})^{\mathsf{T}},
\qquad
\kappa = \frac{\lambda_0}{\lambda_0+\lambda_1+\lambda_2}
$$

其中 $0\le\lambda_0\le\lambda_1\le\lambda_2$。法向取最小特征值对应的特征向量，曲率取 $\kappa$。当 $\lambda_1 \le \max(10^{-18},\,10^{-8}\lambda_2)$ 时，邻域退化共线或重合；代码计数为退化邻域并跳过，不制造虚假法向。此时若已有输入法向则保留，否则该点无法向。

新估计的法向标记 `normalOrientationKnown = false`。方向传播是一个多源最短路径森林：

- 以所有已知方向的点为根，代价从 0 开始；
- 只有当 $|\mathbf{n}_i\cdot\mathbf{n}_j| \ge 0.75$、间距不超过 `4 × max(spacing)`、且位移在两个法向的切平面偏差均不超过 `0.55 × 距离` 时才允许扩展；
- 边代价为 $1-|\mathbf{n}_i\cdot\mathbf{n}_j| + 0.001 \cdot d / \max(s_i,s_j)$；
- 对齐为负时翻转目标法向，输入点的符号保持不变。

未被任何根到达的连通分量继续保持 `normalOrientationKnown = false`。常规实体识别会因此直接拒绝，而不是假设一个外向方向。尖锐边、薄壁两侧与遮挡边界处，PCA 法向本身会模糊，传播也可能跨越不该连接的邻域；门限只能降低概率，不能证明方向正确。
