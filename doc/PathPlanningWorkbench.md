# Path Planning Workbench 界面说明

`PathPlanningWorkbench` 是基于 Qt + OCCT 的交互式路径规划调试工具，用于加载几何模型、配置 voxel planner、计算路径并检查结果。

## 界面布局

| 区域 | 作用 |
|---|---|
| 3D viewport | 显示导入模型、起终点、方向线、计算出的路径，以及可选的体素 overlay。 |
| 右侧控制面板 | 包含导入、显示、离散化、端点、规划器、VTK 导出和日志控制。 |
| 日志面板 | 输出规划参数、成功/失败状态、路径点数、平滑状态和诊断信息。 |

## 导入控件

| 控件 | 作用 |
|---|---|
| `Import Model` | 加载当前 `ModelLoader` 支持的 BREP/STEP/VTK 类输入。 |
| 模型路径标签 | 显示当前输入文件路径。 |

## 显示控件

| 控件 | 作用 | 视觉约定 |
|---|---|---|
| `Mode` | 在 shaded、wireframe 和 mesh 显示模式之间切换。 | 使用 OCCT presentation mode。 |
| `Show key voxels` | 显示路径关键控制体素。 | 蓝色盒子。路径包含映射安全端点时，原始端点不作为 key voxel 显示。 |
| `Show occupied voxels` | 显示有交/阻塞体素。 | 红色盒子。 |
| `Show safety voxels` | 显示 clearance/safety layer 体素。 | 青绿色盒子。 |

体素 overlay 只会在对应复选框开启时随路径计算一起收集。路径规划成功时，Workbench 默认显示最终路径体素包围框内的体素，并向外扩 1 个 voxel。这样调试视图会聚焦于路径走廊，而不是显示全模型中与当前路径无关的体素。

## 离散化控件

| 控件 | 作用 | 影响 |
|---|---|---|
| `Linear deflection` | 控制形状三角化的线性误差。 | 数值越小，三角网格越密。 |
| `Angular deflection` | 控制三角化角度误差。 | 数值越小，曲面保真度越高。 |
| `Apply Discretization` | 按当前离散化参数重建显示 mesh。 | 后续路径规划使用更新后的 mesh。 |

## 端点控件

| 控件 | 作用 |
|---|---|
| `Start` | 用户指定的原始起点。 |
| `Start dir` | 起点映射/吸附时的偏好方向。 |
| `Goal` | 用户指定的原始终点。 |
| `Goal dir` | 终点映射/吸附时的偏好方向。 |
| `Pick start` / `Pick goal` | 在 3D viewport 中拾取起点或终点。 |

规划时会先把原始端点映射到安全/可走体素中心，再用映射点作为 A* 的真实端点。最终显示路径会把原始端点拼回去：

`原始 start -> 映射 start -> ... -> 映射 goal -> 原始 goal`

这样既能保持用户输入的端点契约，又能避免中间路径受非安全端点影响而穿过有交体素。

## 规划器控件

| 控件 | 作用 |
|---|---|
| `Method` | 选择 `Voxel full bounds`、`Voxel lazy` 或 `Geometry query`。其中 `Geometry query` 当前可见但尚未接入 Workbench 路径计算。 |
| `Search` | 选择 `Clearance band` 或 `Free space` 搜索模式。 |
| `Neighbors` | 选择 6、18 或 26 邻域扩展。 |
| `Voxel size` | 设置体素尺寸。数值越小，几何分辨率越高，但构建和搜索成本越高。 |
| `Clearance` | 设置安全距离，用于体素标记和非端点路径安全检查。 |
| `Snap radius` | 设置端点映射时的最大搜索半径，单位为 voxel 层数。 |
| `Smooth path` | 启用 shortcut 后的可选曲线平滑。 |
| `Realtime after input changes` | 预留功能，用于参数变化后自动重新计算。 |
| `VTK Export Settings` | 配置可选 VTK 输出路径。 |
| `Compute Path` | 开始路径计算。 |
| `Stop Computation` | 请求取消当前计算。 |

## 结果日志含义

| 日志字段 | 含义 |
|---|---|
| `rawPath` | A* 后处理之前的 voxel 路径点数。 |
| `optimizedPath` | 删除共线点和 shortcut 后的控制体素点数。 |
| `smoothedPath` | 平滑后的采样点数，仅在请求平滑时显示。 |
| `smoothing=accepted/rejected` | 平滑曲线是否通过 occupied voxel 校验。 |
| `cost` | A* 路径代价，不包含原始端点拼接和平滑后的曲线长度。 |

## 调试流程建议

1. 加载模型，必要时先调整并应用离散化。
2. 设置 start/goal 以及对应方向。
3. 正确性检查优先使用 `Voxel full bounds`。
4. 排查 clearance 或端点映射问题时，开启 `Show occupied voxels` 和 `Show safety voxels`。
5. 使用 `Show key voxels` 检查映射后的路径控制体素。
6. 离散路径确认有效后，再开启 `Smooth path` 检查平滑效果。

