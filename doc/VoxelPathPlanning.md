# 基于三角网格与体素空间的激光头路径规划方案

## 1. 目标

给定：

- 起点 `startPoint`
- 终点 `goalPoint`
- 障碍物 `TopoDS_Shape`
- 最小安全距离 `clearance`
- 体素尺寸 `voxelSize`

计算一条从起点到终点的路径，使激光头在运动过程中尽量沿障碍物表面附近的安全距离层移动，并避免与障碍物表面相交。

当前阶段不考虑机械臂运动学，将激光头抽象为一个点。

---

## 2. 当前算法总体流程

```text
TopoDS_Shape
    -> BRepMesh_IncrementalMesh 三角化
    -> MeshTriangle 集合
    -> VoxelSpace 初始化
    -> 标记 Occupied / ClearanceBand
    -> A* 在 ClearanceBand 中搜索
    -> 输出体素路径 voxelPath
    -> 转换为三维点路径 pointPath
    -> 路径简化
    -> VTK 可视化调试
```

---

## 3. 核心数据结构

### 3.1 VoxelIndex

表示体素在体素空间中的整数索引。

```cpp
struct VoxelIndex
{
    int x = 0;
    int y = 0;
    int z = 0;
};
```

---

### 3.2 VoxelState

当前体素状态定义如下：

```cpp
enum class VoxelState
{
    Free = 0,

    // 与障碍物表面三角片相交，不可通行
    Occupied,

    // 安全距离候选运动区
    // 在 ClearanceBand 模式下，A* 只允许在该区域中搜索
    ClearanceBand,

    Start,
    Goal,
    Path
};
```

语义说明：

| 状态 | 含义 | ClearanceBand 模式下是否可走 |
|---|---|---|
| `Free` | 远离障碍物表面的自由区域 | 否 |
| `Occupied` | 与障碍物表面相交的体素 | 否 |
| `ClearanceBand` | 表面附近的安全距离层 | 是 |
| `Start` | 搜索起点 | 是 |
| `Goal` | 搜索终点 | 是 |
| `Path` | 搜索得到的路径体素 | 是 |

---

### 3.3 VoxelCell

单个体素信息。

```cpp
struct VoxelCell
{
    VoxelIndex index;
    VoxelState state = VoxelState::Free;

    double distanceToSurface = std::numeric_limits<double>::max();

    double gCost = std::numeric_limits<double>::max();
    double hCost = 0.0;
    double fCost = std::numeric_limits<double>::max();

    bool opened = false;
    bool closed = false;

    bool hasParent = false;
    VoxelIndex parent;
};
```

字段说明：

| 字段 | 用途 |
|---|---|
| `index` | 体素索引 |
| `state` | 体素状态 |
| `distanceToSurface` | 体素中心到最近三角面的距离 |
| `gCost/hCost/fCost` | A* 搜索代价 |
| `opened/closed` | A* 搜索状态 |
| `parent` | A* 回溯路径使用 |

---

### 3.4 VoxelSpace

`VoxelSpace` 使用稀疏存储：

```cpp
std::unordered_map<VoxelIndex, VoxelCell, VoxelIndexHash> m_cells;
```

未存储体素默认视为：

```cpp
VoxelState::Free
```

这样可以避免显式存储大量自由空间体素。

---

## 4. 三角网格构建

使用 OCCT：

```cpp
BRepMesh_IncrementalMesh
```

将 `TopoDS_Shape` 离散成三角形集合。

每个三角形结构为：

```cpp
struct MeshTriangle
{
    gp_Pnt p0;
    gp_Pnt p1;
    gp_Pnt p2;
};
```

当前三角化参数：

```cpp
meshDeflection = 0.25;
angularDeflection = 0.5;
```

建议：

```text
meshDeflection <= voxelSize * 0.25 ~ voxelSize * 0.5
```

否则三角网格过粗，会影响体素化精度。

---

## 5. 体素空间初始化

### 5.1 基本流程

对每个三角形：

```text
1. 计算三角形 AABB
2. 按 clearance + halfVoxelDiagonal 扩展 AABB
3. 将扩展 AABB 映射到体素索引范围
4. 遍历候选体素
5. 计算体素中心到三角形距离
6. 根据距离标记 Occupied / ClearanceBand
```

---

### 5.2 距离判断规则

设：

```cpp
halfDiag = 0.5 * sqrt(3.0) * voxelSize;
effectiveClearance = clearance + halfDiag;
```

当前规则：

```text
distanceToSurface <= halfDiag
    -> Occupied

halfDiag < distanceToSurface <= clearance + halfDiag
    -> ClearanceBand

distanceToSurface > clearance + halfDiag
    -> Free
```

注意：

`ClearanceBand` 不是严格的固定 offset 面，而是一个有厚度的候选安全通道。

---

## 6. 为什么不填充 Solid 内部

当前算法只关心障碍物表面附近体素，不关心 Solid 内部是否全部标记为障碍。

原因：

```text
1. 激光头被抽象为点
2. 运动约束主要来自与表面的安全距离
3. Solid 内部可能存在内腔
4. 当前只需构造表面附近不可通行/候选通行区域
5. 避免复杂的 inside/outside 判断
```

因此当前不使用：

```cpp
BRepClass3d_SolidClassifier
```

也不做内部 flood fill。

---

## 7. A* 搜索模式

当前定义：

```cpp
enum class VoxelAStarSearchMode
{
    FreeSpace,
    ClearanceBand
};
```

### 7.1 ClearanceBand 模式

当前主要使用该模式。

```text
可走：
    ClearanceBand
    Start
    Goal
    Path

不可走：
    Occupied
    Free
```

适用于：

```text
路径应沿障碍物表面附近安全层移动
```

---

### 7.2 FreeSpace 模式

当前仅初步定义，后续需要完善。

当前语义：

```text
可走：
    Free

不可走：
    Occupied
    ClearanceBand
```

后续可能扩展为：

```cpp
enum class VoxelAStarSearchMode
{
    FreeSpaceStrict,        // 只走 Free
    FreeSpaceAvoidSurface,  // Free + ClearanceBand 可走，Occupied 不走
    ClearanceBand           // 只走 ClearanceBand
};
```

---

## 8. 起点终点吸附

当起点或终点位于 `Occupied` 或 `Free` 上时，在 `ClearanceBand` 模式下需要吸附到最近的 `ClearanceBand` 体素。

当前支持：

```cpp
snapStartGoalToWalkable = true;
snapMaxRadius = 20;
```

对于封闭曲面，例如球体，可能存在内外两层 `ClearanceBand`。

因此支持方向吸附：

```cpp
useStartSnapDirection = true;
useGoalSnapDirection = true;

startSnapDirection = center -> startPoint;
goalSnapDirection  = center -> goalPoint;
```

用于控制起点终点吸附到外侧还是内侧。

---

## 9. A* 搜索

### 9.1 当前设置

球体单测中使用：

```cpp
searchMode = VoxelAStarSearchMode::ClearanceBand;
neighborType = VoxelNeighborType::FaceEdgeVertex26;
heuristicWeight = 1.0;
turnPenalty = voxelSize * 0.1 ~ voxelSize * 0.3;
```

### 9.2 邻接类型

```cpp
enum class VoxelNeighborType
{
    Face6,
    FaceEdge18,
    FaceEdgeVertex26
};
```

说明：

| 邻接 | 含义 | 特点 |
|---|---|---|
| `Face6` | 共享面 | 最保守，但曲面壳层可能断连 |
| `FaceEdge18` | 共享面或边 | 折中 |
| `FaceEdgeVertex26` | 共享面、边或点 | 连通性最好，但需防穿角 |

当前球体单测建议使用 `FaceEdgeVertex26`。

---

## 10. 球体单测结果

测试模型：

```text
球心: (0, 0, 0)
半径: 50
起点: (0, 0, 50)
终点: (0, 0, -50)
```

最新日志：

```text
Debug around index: 63, 63, 14
radius: 10
Free: 5815
Occupied: 758
ClearanceBand: 2688
Start: 0
Goal: 0
Path: 0
OutOfBounds: 0

Debug around index: 63, 63, 114
radius: 10
Free: 5818
Occupied: 757
ClearanceBand: 2686
Start: 0
Goal: 0
Path: 0
OutOfBounds: 0

A* success.
Visited count: 87378
Path voxel count: 144
Total cost: 169.881
```

结论：

```text
1. ClearanceBand 生成正常
2. 起点终点附近存在候选安全体素
3. A* 能够在 ClearanceBand 中找到路径
4. 球体北极到南极单测通过
```

---

## 11. 路径优化

当前计划增加两层优化。

---

### 11.1 去除共线点

将连续同方向的体素点删除。

优点：

```text
1. 安全
2. 快速
3. 不改变路径可通行性
```

示意：

```text
A -> B -> C
```

如果 `A-B` 与 `B-C` 方向一致，则删除 `B`。

---

### 11.2 直线可通行检测

对路径做跳点压缩：

```text
如果 path[i] 到 path[j] 的直线穿过体素都可通行，
则删除中间点。
```

当前使用 3D DDA 算法遍历线段穿过的体素。

判断逻辑复用：

```cpp
VoxelWalkability::IsIndexWalkable(...)
```

这样 `A*` 和路径优化的可通行规则保持一致。

---

### 11.3 VoxelPathOptimizeResult

```cpp
struct VoxelPathOptimizeResult
{
    std::vector<VoxelIndex> voxelPath;
    std::vector<gp_Pnt> pointPath;

    std::size_t inputCount = 0;
    std::size_t afterCollinearCount = 0;
    std::size_t outputCount = 0;

    int lineCheckCount = 0;
};
```

其中：

```cpp
pointPath[i] = voxelSpace.IndexToCenter(voxelPath[i]);
```

`pointPath` 是最终用于运动控制或折线导出的三维点路径。

---

## 12. VTK 可视化

当前支持三类 VTK 输出。

---

### 12.1 三角网格输出

用于检查：

```text
TopoDS_Shape -> MeshTriangle
```

是否正常。

输出：

```text
shape_mesh.vtk
```

类型：

```text
DATASET POLYDATA
POLYGONS
```

---

### 12.2 体素六面体输出

用于检查：

```text
Occupied / ClearanceBand / Start / Goal / Path
```

输出：

```text
voxel_background.vtk
astar_path_voxels.vtk
optimized_path_key_voxels.vtk
```

类型：

```text
DATASET UNSTRUCTURED_GRID
VTK_HEXAHEDRON
```

---

### 12.3 路径折线输出

用于检查原始路径和优化后路径。

输出：

```text
raw_path_polyline.vtk
optimized_path_polyline.vtk
```

类型：

```text
DATASET POLYDATA
LINES
```

ParaView 中可使用 `Tube` 过滤器让路径更明显。

---

## 13. 当前代码模块

当前已整理或计划整理的模块：

```text
VoxelSpace.h
VoxelMeshBuilder.h
VoxelMeshBuilder.cpp
VoxelAStar.h
VoxelAStar.cpp
VoxelWalkability.h
VoxelPathOptimizer.h
VoxelPathOptimizer.cpp
VoxelVtkExporter.h
VoxelVtkExporter.cpp
MeshVtkExporter.h
MeshVtkExporter.cpp
```

---

# 14. 距离最终目标剩余的开发任务

## 14.1 短期任务

### 任务 1：路径优化跑通

目标：

```text
A* 原始路径 -> 去共线 -> 直线可通行压缩 -> 输出优化路径折线
```

需要完成：

```text
1. 接入 VoxelPathOptimizer
2. 输出 raw_path_polyline.vtk
3. 输出 optimized_path_polyline.vtk
4. 对比优化前后点数
5. 检查优化后路径是否仍在 ClearanceBand 中
```

验收标准：

```text
1. 优化后路径点数明显减少
2. 路径不穿过 Occupied
3. 路径视觉上仍处于 ClearanceBand 中
```

---

### 任务 2：完善 VoxelAStarSearchMode::FreeSpace

当前 `FreeSpace` 只做了基本定义。

需要补充：

```text
1. FreeSpaceStrict
    只允许 Free

2. FreeSpaceAvoidSurface
    允许 Free + ClearanceBand
    禁止 Occupied

3. ClearanceBand
    只允许 ClearanceBand
```

建议修改为：

```cpp
enum class VoxelAStarSearchMode
{
    FreeSpaceStrict,
    FreeSpaceAvoidSurface,
    ClearanceBand
};
```

验收标准：

```text
1. 三种模式均可跑通单测
2. VoxelWalkability 统一管理所有模式判断
3. A* 和路径优化共用同一套可通行规则
```

---

### 任务 3：路径线段体素采样输出

当前优化后路径只输出关键点折线。

需要补充：

```text
1. CollectLineVoxels
2. 将优化路径中每段线经过的体素收集出来
3. 标记为 Path
4. 输出优化路径完整体素覆盖
```

用途：

```text
检查优化后直线段是否真的全部位于可通行区域
```

---

### 任务 4：失败原因与调试日志标准化

当前已有部分失败原因设计。

需要补充：

```text
1. VoxelAStarFailReason 转字符串
2. 打印 inputIndex / snappedIndex / failReason
3. 打印起点终点附近状态统计
4. A* 失败自动导出 debug VTK
```

验收标准：

```text
A* 失败时可以快速知道失败发生在：
    搜索范围外
    吸附失败
    起点不可走
    终点不可走
    open set 为空
    maxVisited 超限
```

---

## 14.2 中期任务

### 任务 5：防穿角逻辑

使用 `FaceEdgeVertex26` 时，路径可能从障碍体素边角之间穿过。

需要补充：

```text
1. 26 邻接下的 corner cutting 检查
2. 对角移动时检查相关中间体素是否可通行
3. 支持开关配置
```

验收标准：

```text
1. 路径不从两个 Occupied 体素之间的斜角穿过
2. 26 邻接仍能保持较好连通性
```

---

### 任务 6：路径代价模型优化

当前代价：

```text
moveCost + turnPenalty
```

后续建议加入：

```text
1. 距离表面的偏好代价
2. 转弯角度惩罚
3. 路径靠近 Occupied 的惩罚
4. 路径过度远离 ClearanceBand 中心的惩罚
```

示例：

```cpp
cost =
    moveCost
  + turnPenalty
  + surfaceDistancePenalty
  + nearOccupiedPenalty;
```

目标：

```text
1. 路径更平滑
2. 转折更少
3. 距离障碍表面更合理
4. 减少贴面或过远路径
```

---

### 任务 7：路径平滑

当前路径是体素中心折线。

后续需要：

```text
1. 折线平滑
2. B 样条 / 圆角过渡
3. 平滑后路径安全性验证
4. 平滑后重新采样
```

验收标准：

```text
1. 平滑路径不穿过 Occupied
2. 平滑路径尽量保持在 ClearanceBand 或允许区域中
3. 输出适合运动控制的点序列
```

---

### 任务 8：更准确的 ClearanceBand 构建

当前使用：

```text
体素中心到三角形距离
```

后续可优化为：

```text
1. Triangle-Box SAT 判断 Occupied
2. 体素盒到三角形距离判断 ClearanceBand
3. 避免大体素中心距离导致的误差
```

目标：

```text
1. Occupied 更准确
2. ClearanceBand 更稳定
3. 减少漏标和过度膨胀
```

---

## 14.3 长期任务

### 任务 9：自适应体素

当前是统一体素尺寸。

目标：

```text
非关键区域用粗体素
障碍附近、起点终点附近、窄通道附近用细体素
```

建议数据结构：

```cpp
struct AdaptiveVoxelKey
{
    int level = 0;
    int x = 0;
    int y = 0;
    int z = 0;
};
```

需要解决：

```text
1. 粗细体素索引映射
2. 父子体素关系
3. 粗细体素邻接关系
4. A* 跨层搜索
5. 路径从粗层进入细层的连接
6. VTK 输出多尺寸体素
```

验收标准：

```text
1. 障碍附近精细
2. 远离障碍区域粗略
3. 搜索节点数量明显下降
4. 路径仍满足安全距离约束
```

---

### 任务 10：多障碍物支持

当前可以传入一个 `TopoDS_Shape`，但后续需要支持多个邻近障碍。

需要支持：

```cpp
struct NearShapes
{
    std::vector<TopoDS_Shape> shapes;
    double minDis = 0.001;
};
```

需要解决：

```text
1. 多 Shape 三角网格合并
2. 每个障碍单独 clearance
3. 记录最近障碍来源
4. 多障碍之间 ClearanceBand 连通性分析
```

---

### 任务 11：路径质量评价

需要增加路径评价指标：

```text
1. 总长度
2. 转折数量
3. 最大转角
4. 最小表面距离
5. 平均表面距离
6. 是否穿过 Occupied
7. 是否离开允许搜索区域
8. 路径点数量
```

用于比较：

```text
1. 不同 voxelSize
2. 不同 clearance
3. 不同 neighborType
4. 不同 heuristicWeight
5. 不同 turnPenalty
6. 不同优化策略
```

---

### 任务 12：工程接口封装

最终需要封装成类似接口：

```cpp
bool CalculateMovementPathByVoxel(
    const TopoDS_Shape& obstacleShape,
    const gp_Pnt& startPoint,
    const gp_Pnt& goalPoint,
    const VoxelMovementPathOptions& options,
    VoxelMovementPathResult& result);
```

建议结果包含：

```cpp
struct VoxelMovementPathResult
{
    bool success = false;

    std::vector<VoxelIndex> rawVoxelPath;
    std::vector<gp_Pnt> rawPointPath;

    std::vector<VoxelIndex> optimizedVoxelPath;
    std::vector<gp_Pnt> optimizedPointPath;

    double totalCost = 0.0;
    int visitedCount = 0;

    VoxelAStarFailReason failReason;
};
```

---

# 15. 后续优化目标

## 15.1 鲁棒性目标

```text
1. 起点终点在 Occupied / Free / ClearanceBand 中均可合理处理
2. 封闭曲面内外侧吸附可控
3. ClearanceBand 局部断裂时有诊断能力
4. A* 失败时可以快速定位原因
5. 26 邻接下不发生穿角
6. 路径优化后仍满足可通行约束
```

---

## 15.2 性能目标

```text
1. 减少 A* visitedCount
2. 降低 VoxelSpace 存储数量
3. 避免无意义创建 Free 节点
4. 路径优化避免 O(n^2) 过大开销
5. 使用 maxShortcutLookAhead 控制 line-of-sight 成本
6. 后续通过自适应体素减少搜索规模
```

---

## 15.3 路径质量目标

```text
1. 路径长度接近合理最短路径
2. 转折数量尽量少
3. 路径平滑
4. 不贴近 Occupied 表面
5. 不离开期望施工区域
6. 可输出给后续运动控制模块
```

---

## 15.4 可视化调试目标

```text
1. 可导出原始三角网格
2. 可导出 Occupied / ClearanceBand 体素
3. 可导出原始 A* 路径折线
4. 可导出优化后路径折线
5. 可导出路径经过的体素
6. 可在 ParaView 中直观看到失败原因
```

---

# 16. 推荐下一步执行顺序

```text
1. 接入 VoxelPathOptimizer
2. 输出 raw_path_polyline.vtk
3. 输出 optimized_path_polyline.vtk
4. 增加 CollectLineVoxels，用于标记优化后线段覆盖体素
5. 完善 VoxelAStarSearchMode::FreeSpace
6. 增加 26 邻接防穿角
7. 增加路径质量评价
8. 再进入自适应体素设计
```

当前最优先任务：

```text
路径简化 + 直线可通行检测 + VTK 折线验证
```

这一步完成后，才能更准确判断后续是否需要：

```text
1. 更厚 ClearanceBand
2. 更细 voxelSize
3. 更强 turnPenalty
4. 自适应体素
5. 路径平滑
```