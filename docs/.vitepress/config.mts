import { defineConfig } from 'vitepress'
import { withMermaid } from 'vitepress-plugin-mermaid'

const sections = [
  {
    text: '快速开始',
    link: '/getting-started/',
    collapsed: true,
    items: [
      { text: '项目介绍', link: '/getting-started/introduction' },
      { text: '安装与构建', link: '/getting-started/installation' },
      { text: '点云到 CAD 快速上手', link: '/getting-started/first-reconstruction' },
      { text: '从多视角图像生成点云', link: '/getting-started/images-to-cloud' },
    ],
  },
  {
    text: '系统架构',
    link: '/architecture/',
    collapsed: true,
    items: [
      { text: '系统分层与依赖', link: '/architecture/system-layers' },
      { text: '点云转模型的代码边界', link: '/architecture/point-cloud-code-boundaries' },
      { text: '端到端数据流', link: '/architecture/data-flow' },
      { text: '数据契约与坐标约定', link: '/architecture/data-contracts' },
      { text: '后台任务与结果提交', link: '/architecture/background-workflow' },
    ],
  },
  {
    text: '模块手册',
    link: '/modules/',
    collapsed: true,
    items: [
      {
        text: '一、多视角图片构建点云',
        collapsed: true,
        items: [
          { text: '虚拟拍摄与相机元数据', link: '/modules/capture' },
          { text: 'COLMAP 图像重建', link: '/modules/image-reconstruction' },
          { text: '编码结构光与投影拍摄', link: '/modules/structured-light' },
        ],
      },
      {
        text: '二、点云转参数化模型',
        collapsed: false,
        items: [
          { text: '五个模块总览', link: '/modules/point-cloud-to-cad' },
          {
            text: '1. 曲面分割',
            collapsed: true,
            items: [
              { text: '输入与预处理', link: '/modules/preprocessing' },
              { text: '分割与几何类型拟合', link: '/modules/surface-detection' },
            ],
          },
          { text: '2. 曲面拓展与验证', link: '/modules/surface-analysis' },
          { text: '3. 特征与实体识别', link: '/modules/feature-recognition' },
          {
            text: '4. 多体分组与闭合',
            collapsed: true,
            items: [
              { text: '部件分组与有限边界闭合', link: '/modules/assembly-reconstruction' },
              { text: '二维轮廓与投影回退', link: '/modules/projection-fitting' },
            ],
          },
          {
            text: '5. 参数化建模与交付',
            collapsed: true,
            items: [
              { text: '特征历史与 B-Rep', link: '/modules/parametric-modeling' },
              { text: '工作区与参数编辑', link: '/modules/studio' },
              { text: '工程保存与 STEP 导出', link: '/modules/persistence' },
            ],
          },
        ],
      },
    ],
  },
  {
    text: '算法原理',
    link: '/algorithms/',
    collapsed: true,
    items: [
      { text: '多视角重建与相机几何', link: '/algorithms/multiview-geometry' },
      { text: '点云采样与法向估计', link: '/algorithms/point-cloud-processing' },
      { text: '解析曲面拟合和连通扩展', link: '/algorithms/primitive-fitting' },
      { text: '面密度与曲面展开验证', link: '/algorithms/surface-support-validation' },
      { text: '特征识别与几何判定', link: '/algorithms/feature-inference' },
      { text: '表面场景与实体推断', link: '/algorithms/solid-inference' },
      { text: '装配分组与壳闭合', link: '/algorithms/shell-closure' },
      { text: '二维轮廓与投影评分', link: '/algorithms/projection-scoring' },
    ],
  },
  {
    text: '案例与验证',
    link: '/validation/',
    collapsed: true,
    items: [
      { text: '数据集与复现约定', link: '/validation/datasets' },
      { text: '贯穿孔与盲孔', link: '/validation/hole-cases' },
      { text: 'U 形槽与装配案例', link: '/validation/projection-assembly-cases' },
      { text: '阶段契约与回归测试', link: '/validation/contract-tests' },
      { text: '端到端精度与性能评测', link: '/validation/end-to-end' },
    ],
  },
  {
    text: '性能调优',
    link: '/performance/',
    collapsed: false,
    items: [
      { text: '优化总览', link: '/performance/' },
      { text: '并发与串行状态', link: '/performance/execution-model' },
      { text: '渲染与交互', link: '/performance/rendering' },
      { text: '预处理与曲面识别', link: '/performance/preprocessing-recognition' },
      { text: '结构光与 Python', link: '/performance/structured-light-python' },
      { text: '性能基准与正确性验证', link: '/performance/benchmarks' },
    ],
  },
  {
    text: 'CLI',
    link: '/cli/',
    collapsed: false,
    items: [
      { text: 'CLI 使用指南', link: '/cli/' },
      { text: '在线、离线与命令能力', link: '/cli/modes' },
      { text: '命令参考', link: '/cli/commands' },
      { text: 'AI 工作流与诊断', link: '/cli/workflows' },
    ],
  },
  {
    text: '开发维护',
    link: '/development/',
    collapsed: true,
    items: [
      { text: '开发环境与代码布局', link: '/development/developer-setup' },
      { text: '超参数配置', link: '/development/configuration' },
      { text: '贡献指南与文档维护', link: '/development/contributing' },
      { text: '设计决策', link: '/development/design-decisions' },
      {
        text: '实体推断（SurfaceScene）',
        collapsed: true,
        items: [
          { text: '实施方案（参考）', link: '/development/solid-inference-plan' },
          { text: '实现状态与差距', link: '/development/solid-inference-status' },
        ],
      },
      { text: '曲面支撑验证修改方案', link: '/development/surface-support-validation-plan' },
      { text: '多视角图片转点云扩展方案', link: '/development/multiview-reconstruction-extension' },
      { text: '结构光实现验证记录', link: '/development/structured-light-validation' },
      { text: '路线图与里程碑', link: '/development/roadmap' },
    ],
  },
]

export default withMermaid(defineConfig({
  lang: 'zh-CN',
  title: 'Bochu Capstone',
  description: 'ParamCAD Studio 项目技术文档',
  base: '/Bochu-ParamCAD/',
  srcDir: 'site',
  markdown: { math: true },
  themeConfig: {
    nav: sections.map(({ text, link }) => ({ text, link })),
    sidebar: sections,
    socialLinks: [{ icon: 'github', link: 'https://github.com/ghhufue/Bochu-ParamCAD' }],
    outline: { label: '本页目录' },
    docFooter: { prev: '上一页', next: '下一页' },
    search: {
      provider: 'local',
      options: {
        miniSearch: {
          options: {
            tokenize: (text) => {
              const segmenter = new Intl.Segmenter('zh-CN', { granularity: 'word' })
              return Array.from(segmenter.segment(text))
                .filter(({ isWordLike }) => isWordLike)
                .map(({ segment }) => segment)
            },
          },
        },
        translations: { button: { buttonText: '搜索文档', buttonAriaLabel: '搜索文档' } },
      },
    },
  },
}))
