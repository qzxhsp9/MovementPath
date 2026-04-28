# 基于源码的 VoxelPathPlanning 说明

本文档根据 `MovementPath` 当前源码整理，描述已经实现的体素路径规划流程、关键数据结构、当前测试入口以及仍待完善的部分。文中不再把“已实现”和“设想方案”混写。

## 1. 模块概览

当前体素路径规划核心源码位于 `src/MovementPathCore/`，并由 `MovementPathCore` 静态库统一管理：

- `VoxelPathPlanner.h/.cpp`：规划主接口，封装三角化、空间索引、体素构建、A*、路径优化、统计和可选 VTK 导出
- `VoxelSpace.h`：体素索引、状态、搜索边界、稀疏体素存储
- `VoxelMeshBuilder.h/.cpp`：`TopoDS_Shape -> 三角网格 -> VoxelSpace`，支持全局、局部 box 和追加构建
- `TriangleSpatialHash.h/.cpp`：`MeshTriangle` 空间哈希索引，服务局部体素化和 lazy chunk 构建
- `VoxelChunkCache.h/.cpp`：按 chunk 追加生成体素并缓存已生成 chunk
- `VoxelWalkability.h`：不同搜索模式下的可通行判定
- `VoxelAStar.h/.cpp`：A* 搜索、起终点吸附、路径回溯，支持 `ensureCellBuilt` 钩子供 lazy 构建接入
- `VoxelPathOptimizer.h/.cpp`：共线点删除、3D DDA 直连压缩
- `VoxelVtkExporter.h/.cpp`：体素和折线路径导出为 VTK
- `MeshVtkExporter.h/.cpp`：三角网格导出为 VTK
- `src/main.cpp`：示例入口，只负责构造场景、设置选项并调用 `VoxelPathPlanner::Plan()`

当前主流程是：

```text
TopoDS_Shape
  -> BRepMesh_IncrementalMesh 三角化
  -> MeshTriangle 集合
  -> 构建 TriangleSpatialHash
  -> 根据 FullMeshBounds / StartGoalBox / LazyChunks 构建或按需追加 VoxelSpace
  -> 标记 Occupied / ClearanceBand
  -> A* 搜索 voxelPath
  -> VoxelPathOptimizer 优化
  -> 导出 VTK 结果
```

## 2. 关键数据结构

### 2.1 Vec

项目没有直接在核心逻辑中依赖 `gp_Pnt`/`gp_Vec`，而是定义了一个轻量三维向量：

```cpp
struct Vec
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};
```

它提供了点积、归一化、距离和基础加减乘，供体素化、A* 启发式和方向吸附使用。

### 2.2 VoxelIndex

体素整数索引：

```cpp
struct VoxelIndex
{
    int x = 0;
    int y = 0;
    int z = 0;
};
```

同时定义了 `VoxelIndexHash`，可作为 `unordered_map` 的 key。

### 2.3 VoxelState

当前源码中的状态如下：

```cpp
enum class VoxelState
{
    Free = 0,
    Occupied,
    ClearanceBand,
    Start,
    Goal,
    Path
};
```

状态含义：

| 状态 | 含义 |
|---|---|
| `Free` | 默认自由空间；对稀疏体素场来说，未显式存储的索引也视为 `Free` |
| `Occupied` | 体素中心到三角形距离小于等于半体素对角线，认为可能与表面相交 |
| `ClearanceBand` | 位于安全层候选区域中的体素 |
| `Start` | A* 搜索起点或最终导出时的起点 |
| `Goal` | A* 搜索终点或最终导出时的终点 |
| `Path` | 路径中间体素 |

### 2.4 VoxelCell

单个体素单元结构：

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

其中：

- `distanceToSurface` 记录该体素中心到最近三角面的最小距离
- `g/h/f`、`opened/closed`、`parent` 只在 A* 搜索期间使用

### 2.5 VoxelBounds

搜索边界：

```cpp
struct VoxelBounds
{
    VoxelIndex minIndex;
    VoxelIndex maxIndex;
};
```

`VoxelSpace` 可以设置一个可选搜索边界。A* 和路径优化的 walkability 检查都会先判断索引是否在该边界内。

### 2.6 VoxelSpace

`VoxelSpace` 是稀疏体素场，核心存储为：

```cpp
std::unordered_map<VoxelIndex, VoxelCell, VoxelIndexHash> m_cells;
```

特点：

- 未存储体素默认视为 `Free`
- 支持 `WorldToIndex`、`IndexToCenter`、`IndexToMinCorner`、`IndexToMaxCorner`
- 支持 6/18/26 邻接索引生成
- 支持记录并重置 A* 搜索数据
- 支持设置搜索边界 `VoxelBounds`

## 3. 网格体素化实现

### 3.1 三角化

`VoxelMeshBuilder::BuildShapeTriangulation()` 使用：

```cpp
BRepMesh_IncrementalMesh
```

把 `TopoDS_Shape` 离散为：

```cpp
struct MeshTriangle
{
    Vec p0;
    Vec p1;
    Vec p2;
};
```

每个 `TopoDS_Face` 的 `Poly_Triangulation` 会被读取出来，并应用 `TopLoc_Location` 的变换。

### 3.2 构建参数

当前参数结构：

```cpp
struct VoxelMeshBuildOptions
{
    double voxelSize = 1.0;
    double meshDeflection = 0.1;
    double angularDeflection = 0.5;
    double clearance = 0.0;
    double bboxPadding = 0.0;
    bool conservativeClearance = true;
    bool storeFreeCells = false;
};
```

语义：

- `voxelSize`：体素边长
- `meshDeflection` / `angularDeflection`：OCCT 三角化参数
- `clearance`：期望安全层厚度
- `bboxPadding`：整体包围盒额外扩展
- `conservativeClearance`：若为 `true`，会把半体素对角线加到安全距离中
- `storeFreeCells`：是否把边界内的 `Free` 体素全部显式写入稀疏场

### 3.3 体素化流程

`BuildVoxelSpaceFromShapeMesh()` 的实际流程如下：

```text
1. 三角化 shape
2. 计算所有三角形的整体 AABB
3. 用 bboxPadding + clearance + voxelSize 扩大整体 AABB
4. 用 globalBox.minP 作为 VoxelSpace.origin
5. 计算整体搜索边界并写入 VoxelSpace
6. 如有需要，先显式填充边界内全部 Free 体素
7. 对每个三角形执行 MarkTriangleToVoxelSpace()
8. 统计 Occupied / ClearanceBand 数量
```

### 3.4 单三角形写入规则

对每个三角形：

```text
1. 计算三角形 AABB
2. 计算 effectiveClearance
3. 按 effectiveClearance 扩展该 AABB
4. 遍历扩展 AABB 覆盖到的候选体素
5. 计算体素中心到三角形的距离
6. 更新 distanceToSurface
7. 按阈值写入 Occupied 或 ClearanceBand
```

其中：

```cpp
halfDiag = 0.5 * sqrt(3.0) * voxelSize;
effectiveClearance = clearance + halfDiag; // conservativeClearance = true 时
```

若 `conservativeClearance = false`，则：

```cpp
effectiveClearance = clearance;
```

源码中的三角形候选体素范围也是按 `effectiveClearance` 扩展的。因此在关闭保守模式时，如果 `clearance` 很小，候选范围可能不足以覆盖所有“中心距离三角形小于 `halfDiag`”的体素。当前默认值为 `true`，也是更稳妥的设置。

判定规则：

```text
distance <= halfDiag
    -> Occupied

distance > halfDiag && distance <= effectiveClearance
    -> ClearanceBand
```

注意：

- 当前 `ClearanceBand` 是一个“有厚度的可搜索带”，不是严格意义上的 offset 曲面
- 当前距离是“体素中心到三角形距离”，不是 box-triangle 精确距离
- 当前没有做 solid 内外判定，也没有填充实体内部

## 4. 搜索模式与可通行规则

### 4.1 当前搜索模式

源码中只有两种模式：

```cpp
enum class VoxelAStarSearchMode
{
    FreeSpace,
    ClearanceBand
};
```

### 4.2 当前 walkability 实现

`VoxelWalkability::IsStateWalkable()` 的当前规则：

```text
Start / Goal / Path
    -> 始终可走

FreeSpace
    -> 只有 Free 可走

ClearanceBand
    -> 只有 ClearanceBand 可走
```

这意味着：

- `FreeSpace` 还不是“避开表面但可经过安全层”的模式
- `ClearanceBand` 模式下，普通 `Free` 体素不可走
- 路径优化的直连检测与 A* 共用同一套 walkability 规则

## 5. A* 搜索实现

### 5.1 选项结构

```cpp
struct VoxelAStarOptions
{
    VoxelNeighborType neighborType = VoxelNeighborType::Face6;
    VoxelAStarSearchMode searchMode = VoxelAStarSearchMode::ClearanceBand;
    double heuristicWeight = 1.0;
    double turnPenalty = 0.0;
    bool snapStartGoalToWalkable = true;
    int snapMaxRadius = 20;
    bool useStartSnapDirection = false;
    bool useGoalSnapDirection = false;
    Vec startSnapDirection;
    Vec goalSnapDirection;
    int maxVisitedCount = 0;
    bool markPathToVoxelSpace = true;
};
```

### 5.2 邻接方式

```cpp
enum class VoxelNeighborType
{
    Face6,
    FaceEdge18,
    FaceEdgeVertex26
};
```

当前 `VoxelSpace::GetNeighborIndices()` 已完整支持 6/18/26 邻接。

### 5.3 起终点吸附

当起点或终点不在可行走体素上时，A* 可以先吸附到最近可行走体素。

支持两种吸附方式：

- `FindNearestWalkableIndex()`：按半径层扩张，选择距离最近候选体素
- `FindNearestWalkableIndexWithDirection()`：在候选体素位于首选方向半空间内时优先吸附

方向吸附的用途是：对于封闭曲面，尽量控制起终点被吸到外侧还是内侧的 `ClearanceBand`。

### 5.4 搜索流程

`VoxelAStar::Search()` 的当前行为：

```text
1. 检查 VoxelSpace 是否有效
2. 将起终点世界坐标映射到体素索引
3. 检查索引是否落在搜索边界内
4. 根据设置执行起终点吸附，或直接验证起终点可走性
5. 重置所有已存储体素的 A* 字段
6. 初始化起点 g/h/f
7. 使用 priority_queue 执行 A*
8. 对每个邻居按 moveCost + turnPenalty 更新代价
9. 到达终点后回溯 voxelPath，并转换为 pointPath
10. 可选地把路径写回 VoxelSpace 为 Start / Goal / Path
```

当 `markPathToVoxelSpace = true` 时，A* 成功后会把起点、终点和路径中间点写回 `VoxelSpace`。后续路径优化如果复用这个 `VoxelSpace`，这些 `Start` / `Goal` / `Path` 状态会被 `VoxelWalkability` 视为可走。

### 5.5 代价函数

当前移动代价：

```text
moveCost = voxelSize * 欧氏步长
总代价 = moveCost + 可选 turnPenalty
```

启发式：

```text
h = heuristicWeight * 当前体素到目标体素的欧氏距离
```

转向惩罚通过比较 `(prev -> curr)` 与 `(curr -> next)` 的离散方向是否一致来决定是否加罚。

### 5.6 失败原因

当前失败原因枚举已经实现：

```cpp
enum class VoxelAStarFailReason
{
    None = 0,
    InvalidVoxelSpace,
    StartOrGoalOutsideBounds,
    SnapStartFailed,
    SnapGoalFailed,
    StartNotWalkable,
    GoalNotWalkable,
    MaxVisitedExceeded,
    OpenSetEmpty
};
```

但当前还没有“失败原因转字符串”的统一接口。

## 6. 路径优化实现

`VoxelPathOptimizer` 已经接入，当前做两层优化。

### 6.1 共线点删除

`RemoveCollinearVoxels()` 会删除离散方向完全相同的中间点。

例如：

```text
A -> B -> C
```

若 `A-B` 和 `B-C` 的索引增量完全一致，则删除 `B`。

### 6.2 直连压缩

`IsLineWalkable()` 使用 3D DDA 检查两体素中心连线经过的所有体素是否都可走。

特点：

- 先检查 `from` 和 `to` 本身是否可走
- 沿 x/y/z 同步推进，处理直线恰好穿过边/角时的并列最小 `t`
- 使用 `VoxelWalkability::IsIndexWalkable()`，因此与 A* 搜索模式保持一致

该检查仍然是离散体素层面的 line-of-sight，不是连续几何层面的精确碰撞检测。它只能保证采样穿过的体素满足当前搜索模式的可通行规则。

`Optimize()` 会：

```text
1. 可选删除共线点
2. 从当前点开始，尽量寻找最远可直连点
3. 用该点替代中间整段折线
4. 输出优化后的 voxelPath 和 pointPath
```

优化参数：

```cpp
struct VoxelPathOptimizeOptions
{
    VoxelAStarSearchMode searchMode = VoxelAStarSearchMode::ClearanceBand;
    bool allowSpecialStates = true;
    int maxShortcutLookAhead = 200;
    bool removeCollinear = true;
    bool enableLineOfSightShortcut = true;
};
```

需要注意的一点：

- `allowSpecialStates` 当前定义了，但在实现中没有单独使用；实际可通行性仍完全由 `VoxelWalkability` 决定

## 7. VTK 导出能力

### 7.1 三角网格导出

`MeshVtkExporter` 支持：

- 直接导出 `std::vector<MeshTriangle>`
- 直接从 `TopoDS_Shape` 三角化后导出

输出格式：

```text
DATASET POLYDATA
POLYGONS
```

每个三角形单独写 3 个点，不复用顶点。

### 7.2 体素导出

`VoxelVtkExporter::ExportVoxelSpaceToVtk()` 导出指定状态的体素为六面体：

```text
DATASET UNSTRUCTURED_GRID
VTK_HEXAHEDRON
```

默认导出状态：

- `Occupied`
- `ClearanceBand`

同时会写出以下 `CELL_DATA`：

- `voxel_state`
- `distance_to_surface`
- `voxel_x`
- `voxel_y`
- `voxel_z`

### 7.3 路径折线导出

`ExportPathPolylineToVtk()` 支持两种输入：

- `std::vector<VoxelIndex>`
- `std::vector<Vec>`

输出格式：

```text
DATASET POLYDATA
LINES
```

并附带 `POINT_DATA path_index`。

## 8. 当前 main.cpp 的测试入口

`main.cpp` 目前的主要测试函数是 `TestVoxelAStar()`。

### 8.1 当前测试模型

`main()` 中当前实际执行的是球体示例：

```cpp
const double R = 50.0;
TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), R).Shape();

TestVoxelAStar(
    sphere,
    gp_Pnt(0, 0, -50),
    gp_Vec(0, 0, -1),
    gp_Pnt(0, 0, 50),
    gp_Vec(0, 0, 1));
```

即：

- 障碍物：半径 50 的球
- 起点：球面南极附近 `(0, 0, -50)`
- 终点：球面北极附近 `(0, 0, 50)`
- 起终点都启用了方向吸附

### 8.2 当前构建参数

测试中实际使用：

```cpp
buildOptions.voxelSize = 1.0;
buildOptions.meshDeflection = 0.25;
buildOptions.angularDeflection = 0.3;
buildOptions.clearance = 3.0;
buildOptions.bboxPadding = 10.0;
buildOptions.conservativeClearance = true;
buildOptions.storeFreeCells = false;
```

### 8.3 当前 A* 参数

测试中实际使用：

```cpp
astarOptions.searchMode = VoxelAStarSearchMode::ClearanceBand;
astarOptions.neighborType = VoxelNeighborType::Face6;
astarOptions.heuristicWeight = 1.0;
astarOptions.turnPenalty = voxelSize * 0.1;
astarOptions.snapStartGoalToWalkable = true;
astarOptions.snapMaxRadius = 20;
astarOptions.useStartSnapDirection = true;
astarOptions.useGoalSnapDirection = true;
astarOptions.maxVisitedCount = 0;
astarOptions.markPathToVoxelSpace = true;
```

这里需要特别注意：当前实际运行的是 `Face6`，不是 `FaceEdgeVertex26`。

### 8.4 当前导出文件

示例代码会导出到 `D:/`：

- `shape_mesh.vtk`
- `astar_failed.vtk`（仅失败时）
- `astar_path.vtk`
- `optimized_path_voxels.vtk`
- `optimized_path_polyline.vtk`

当前示例没有导出 `raw_path_polyline.vtk`。

## 9. 当前实现边界与未完成项

结合源码，当前仍存在这些明确边界：

### 9.1 已知未实现或未完善

- `VoxelAStarSearchMode::FreeSpace` 只有单一语义，没有拆分成更细模式
- 没有 26 邻接下的防穿角检查
- 没有 solid 内外判定，也没有实体内部填充
- `distanceToSurface` 基于“体素中心到三角形距离”，不是 box-triangle 精确距离
- `allowSpecialStates` 选项当前未被单独使用
- `conservativeClearance = false` 时，单三角形候选范围不会额外按 `halfDiag` 扩展，极小 `clearance` 下可能漏标部分近表面体素
- 缺少统一的 `VoxelAStarFailReason -> string`
- 缺少优化前原始折线单独导出
- 缺少“优化后线段经过体素集合”的完整采样与可视化

### 9.2 当前实现中已具备的能力

- 起终点自动吸附到最近可走体素
- 封闭曲面场景下按方向控制吸附侧
- A* 与路径优化共用同一套 walkability 判定
- 可导出三角网格、关键体素、优化后折线
- 可在 `VoxelCell` 中保留 `distanceToSurface` 供后续代价函数扩展

## 10. 建议的文档维护原则

后续如果继续演进该模块，建议按下面的方式维护本文档：

- “当前实现”只写源码中已经存在的行为
- “待实现”单独成节，不和主流程混排
- 参数示例尽量标明来自哪个入口，例如 `main.cpp`、单元测试或业务接口
- 若运行参数变更，优先同步 `8.2`、`8.3` 两节

这样文档可以继续作为源码阅读入口，而不是方案草稿。

## 11. 当前测试入口

当前测试不再通过 `main.cpp` 间接覆盖规划流程，而是直接链接 `MovementPathCore` 并调用核心接口：

- `TriangleSpatialHashTests`：覆盖空间哈希查询必须包含 brute-force AABB 命中的三角形。
- `VoxelChunkCacheTests`：覆盖 chunk 配置失败、重复命中、负坐标 chunk、bounds 扩展、padding 覆盖和追加体素化行为。
- `VoxelPathPlannerTests`：覆盖默认全局构建、显式 `StartGoalBox` 局限性对照、`ensureCellBuilt` 钩子、lazy guardrail fallback、lazy 成功路径和 `maxCostRegressionRatio` 质量回退。

构建与测试命令：

```text
cmake --build out\build\x64-Debug --config Debug
ctest --test-dir out\build\x64-Debug --output-on-failure
```

## 12. 当前优化实现状态

- 默认规划仍使用 `FullMeshBounds`，这是 correctness baseline。
- `StartGoalBox` 是显式 opt-in 的局部预体素化模式，只适合短距离或调用方能接受质量回退的场景；它可能裁剪真实绕行路径。
- `TriangleSpatialHash` 已接入局部构建和 chunk 构建，用于减少候选三角形扫描。
- `LazyChunks` 已作为实验模式接入 planner，默认关闭；开启后 A* 通过 `VoxelChunkCache` 按需生成 chunk。
- lazy 模式支持 `maxChunkBuildCount` guardrail，失败或触发 guardrail 时可按 `FullMeshBoundsOnFailure` 回退。
- `maxCostRegressionRatio` 已实现为质量回退阈值；lazy 成功后可与 full-bounds baseline 对照，超过阈值时返回 full-bounds 结果并保留 lazy 尝试统计。
- `VoxelPathPlannerResult` 已暴露 `executionMode`、`fallbackExecutionMode`、`lazyAttemptCost`、`fallbackCost`、`optimizeResult` 和最终搜索边界，便于测试直接断言行为。
