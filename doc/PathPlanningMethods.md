# 路径规划方式对比

本文档梳理 MovementPath 当前已经实现或在 Workbench 中暴露的路径规划方式。
代码名、枚举名和界面文案保留英文，便于和源码、日志对应。

## 规划方式总览

| 方式 | Workbench 是否可选 | 当前状态 | 核心思路 | 主要用途 | 主要限制 |
|---|---:|---|---|---|---|
| `Voxel full bounds` | 是 | 当前正确性基准 | 对完整模型包围范围进行体素化，再运行 voxel A*。 | 验证正确性、路径质量和 benchmark baseline。 | 体素构建范围最大，耗时和内存成本最高。 |
| `Voxel lazy` | 是 | 实验性加速路径 | 初始化稀疏体素空间，A* 搜索访问到某个区域时再按 chunk 构建体素。 | 大模型、搜索区域较小的场景。 | 依赖 fallback 和 guardrail；路径质量与 chunk 配置有关。 |
| `StartGoalBox` 局部体素化 | Workbench 暂未直接暴露 | API 和测试中存在 | 只对起点-终点包围盒加 padding 后的局部区域体素化。 | 快速局部实验。 | 可能裁掉局部盒外的有效绕行路径，不能作为正确性基准。 |
| `Geometry query planner` | 下拉框可见，但未接入计算 | 查询基础和脚手架已完成 | 不完整预体素化，改用 mesh 空间索引、最近距离和 segment clearance 查询。 | 长期替代方向。 | `Plan()` 当前只构建 query context，并返回 `NotImplemented`。 |

## voxel 规划公共流程

| 步骤 | 目的 | Full bounds | Lazy chunks | StartGoalBox |
|---|---|---|---|---|
| 导入并三角化 | 将 BREP/VTK 输入转换为三角网格。 | 相同 | 相同 | 相同 |
| 构建三角形空间哈希 | 减少体素标记时需要测试的 triangle 数量。 | 使用 | 使用 | 使用 |
| 构建体素空间 | 标记 `Occupied`、`ClearanceBand`，并记录 `distanceToSurface`。 | 构建完整模型 bounds。 | 搜索访问到区域时由 chunk cache 按需构建。 | 只构建起终点局部盒。 |
| 端点映射 | 将原始起终点映射到可行/安全体素中心，作为 A* 的真实端点。 | 支持 | 支持 | 支持 |
| A* 搜索 | 在体素图上寻找离散路径。 | 搜索完整体素空间。 | 搜索过程中触发 lazy chunk 构建。 | 搜索局部体素空间。 |
| 连通性 fallback | 在配置允许时，用更宽邻域重试连通性问题。 | 支持 | fallback 前支持 | 支持 |
| 还原原始端点 | 将原始端点拼回最终输出路径。 | 输出形式为 `原始 start -> 映射 start -> ... -> 映射 goal -> 原始 goal`。 | 相同 | 相同 |
| 折线优化 | 删除共线体素点，并尝试 line-of-sight shortcut。 | 开启 | 开启 | 开启 |
| 非端点安全检查 | 避免中间路径经过有交或不安全体素。 | A* 和 shortcut 都会检查。 | 相同 | 相同 |
| 可选平滑 | 为显示和后续速度规划生成更密的曲线采样点。 | 仅勾选 `Smooth path` 时启用。 | 相同 | 相同 |
| 可视化/导出 | 显示最终路径、可选体素 overlay，并可导出 VTK。 | 支持 | 支持，包括 lazy chunk bounds。 | 支持 |

## 搜索模式

| 搜索模式 | UI 文案 | 可走状态 | 目的 | 说明 |
|---|---|---|---|---|
| `ClearanceBand` | Clearance band | `ClearanceBand` 体素，以及特殊的 start/goal/path 状态。 | 让路径靠近配置出的安全层。 | 中间路径仍会进行最小距离和有交体素检查。 |
| `FreeSpace` | Free space | 除 `Occupied` 以外的体素，包括 `Free` 和 `ClearanceBand`，以及特殊的 start/goal/path 状态。 | 只要求路径不经过有交体素。 | 不强制 `clearance` 最小距离；当 safety-band 搜索过于受限时可尝试。 |

## 邻域模式

| 邻域模式 | UI 文案 | 目的 | 路径特征 | 成本 |
|---|---|---|---|---|
| `Face6` | 6-face | 只允许轴向移动。 | 更接近曼哈顿路径，更保守。 | 分支数最低。 |
| `FaceEdge18` | 18-face-edge | 允许轴向和边对角移动。 | 比 6 邻域更短、更灵活。 | 分支数中等。 |
| `FaceEdgeVertex26` | 26-face-edge-vertex | 允许轴向、边对角和体对角移动。 | 离散路径通常最短，但更容易出现斜切。 | 分支数最高，需要 DDA 校验兜底。 |

## 路径后处理

| 阶段 | 默认状态 | 目的 | 输出变化 | 校验方式 |
|---|---:|---|---|---|
| 删除共线点 | 开启 | 删除方向不变的中间点。 | 减少控制体素数量。 | 保持原始 voxel chain 拓扑。 |
| line-of-sight shortcut | 开启 | 用更长直线段替代多段折线。 | `optimizedPath` 通常小于 `rawPath`。 | DDA 检查体素可走性和非端点安全性。 |
| 曲线平滑 | 关闭 | 生成更密、更平滑的显示/速度规划点。 | `pointPath` 可能变成曲线采样点。 | 采样点和采样段不得经过 `Occupied` 体素；失败时回退到优化折线。 |

## 使用建议

| 场景 | 推荐配置 |
|---|---|
| 验证正确性或路径质量 | 使用 `Voxel full bounds`、`Clearance band`，并采用较保守的 voxel size。 |
| 大模型性能实验 | 使用 `Voxel lazy`，并保留 full-bounds fallback。 |
| 排查端点映射问题 | 开启 key voxels、有交体素和安全层体素 overlay。 |
| 排查路径贴近障碍物 | 显示 occupied/safety voxels；如果路径过粗，减小 voxel size。 |
| 需要更平滑的显示路径 | 勾选 `Smooth path`，并查看日志中的 `smoothing=accepted/rejected`。 |
