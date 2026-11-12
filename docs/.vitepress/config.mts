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
      { text: '虚拟拍摄与相机元数据', link: '/modules/capture' },
      { text: 'COLMAP 图像重建', link: '/modules/image-reconstruction' },
      { text: '编码结构光与投影拍摄', link: '/modules/structured-light' },
      { text: '点云输入与预处理', link: '/modules/preprocessing' },
      { text: '曲面检测与证据', link: '/modules/surface-detection' },
      { text: '曲面分析与修复', link: '/modules/surface-analysis' },
      { text: '实体识别与人工引导', link: '/modules/feature-recognition' },
      { text: '装配分离与闭合', link: '/modules/assembly-reconstruction' },
      { text: '轮廓恢复与投影拟合', link: '/modules/projection-fitting' },
      { text: '参数化建模与 B-Rep', link: '/modules/parametric-modeling' },
      { text: '桌面工作区与参数编辑', link: '/modules/studio' },
      { text: '工程保存与 STEP 导出', link: '/modules/persistence' },
    ],
  },
  {
    text: '算法原理',
    link: '/algorithms/',
    collapsed: true,
    items: [
      { text: '多视角重建与相机几何', link: '/algorithms/multiview-geometry' },
      { text: '点云采样与法向估计', link: '/algorithms/point-cloud-processing' },
      { text: '解析曲面拟合', link: '/algorithms/primitive-fitting' },
      { text: '特征识别与几何判定', link: '/algorithms/feature-inference' },
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
    text: '开发维护',
    link: '/development/',
    collapsed: true,
    items: [
      { text: '开发环境与代码布局', link: '/development/developer-setup' },
      { text: '超参数配置与 CLI', link: '/development/configuration-cli' },
      { text: '贡献指南与文档维护', link: '/development/contributing' },
      { text: '设计决策', link: '/development/design-decisions' },
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
