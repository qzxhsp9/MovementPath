# VoxelPathPlanning 性能优化方案与开发计划

本文档作为 `MovementPath` 体素路径规划后续性能优化的开发基准。目标是在保持路径正确性和可调试性的前提下，减少不必要的全局体素化、降低内存占用，并为按需体素生成和缓存预留演进路径。

## 1. 背景与问题

当前主流程是：

```text
TopoDS_Shape
  -> 三角化为 MeshTriangle 集合
  -> 对整个模型包围盒范围内的三角形影响区做体素化
  -> 生成 VoxelSpace
  -> A* 在 VoxelSpace 中搜索路径
  -> 路径优化与 VTK 导出
```

该流程实现简单，`VoxelWalkability` 查询也直接。但它会提前支付全局体素化成本：

- 起点和终点距离较近时，最终路径可能只使用很小一部分体素。
- 复杂模型中，很多 `Occupied` / `ClearanceBand` 体素不会参与 A*。
- `ClearanceBand` 越厚、`voxelSize` 越小，全局体素数量增长越快。
- 当前每个三角形在自己的扩展 AABB 内遍历候选体素，模型越复杂，重复写入和距离计算越多。

因此，性能优化的核心方向是：

```text
不要一开始生成所有可能体素；
优先生成路径搜索实际需要的局部体素；
必要时再扩张范围或按需生成。
```

## 2. 优化目标

### 2.1 性能目标

- 减少体素化耗时。
- 减少 `VoxelSpace::CellCount()`。
- 减少无关三角形参与距离计算。
- 避免 A* 查询时退化为每个体素扫描全量三角形。
- 支持大模型、细体素、小范围路径场景。

### 2.2 正确性目标

- A* 与路径优化继续共用同一套 walkability 规则。
- `Occupied` / `ClearanceBand` 判定结果与当前全局体素化保持可解释的一致性。
- 局部范围过小时能够诊断并扩张重试。
- 失败时能够导出足够的调试 VTK。

### 2.3 工程目标

- 优先做低风险、可验证的局部优化。
- 保留当前全局体素化作为基线和回退路径。
- 每个阶段都能独立验收，避免一次性重写主流程。
- 性能收益必须通过统计数据证明，而不是只凭视觉结果。

## 3. 候选方案分析

### 3.1 局部范围预体素化

思路：

```text
根据 startPoint / goalPoint 先构造一个局部搜索包围盒
只处理该包围盒附近的三角形和体素
如果 A* 失败，则扩大范围重试
```

初始范围可以来自：

```text
startPoint 和 goalPoint 的 AABB
  + clearance
  + halfVoxelDiagonal
  + searchPadding
```

优点：

- 改动小，和当前 `VoxelSpace` / `VoxelMeshBuilder` 结构兼容。
- 容易保留全局体素化作为对照。
- 适合起终点距离较短的典型场景。

风险：

- 局部范围太小会漏掉真实可行绕行路径。
- 需要设计失败后的扩张策略。
- 如果仍然遍历全量三角形，复杂模型下收益有限。

结论：

这是第一阶段最适合落地的优化。

### 3.2 三角形空间索引

思路：

```text
为 MeshTriangle 建立空间索引
局部体素化或按需体素查询时，只取相关三角形
```

可选索引：

- Uniform Grid / Spatial Hash
- BVH
- AABB Tree

短期建议使用 Uniform Grid 或 Spatial Hash，原因是实现成本较低，和体素空间天然匹配。后续如果模型尺度差异很大，再考虑 BVH。

优点：

- 避免局部体素化仍扫描全部三角形。
- 是 lazy voxelization 和 chunk 缓存的基础设施。
- 能复用三角形 AABB。

风险：

- 需要选择合理的索引 cellSize。
- 三角形跨多个 cell 时会重复登记。
- 查询半径过大时仍可能返回较多候选三角形。

结论：

这是第二阶段核心工作。没有三角形空间索引，不建议直接做逐体素按需判断。

### 3.3 按需体素化 Lazy Voxelization

思路：

```text
A* 查询某个 VoxelIndex 的状态
  -> 缓存命中：直接返回
  -> 缓存未命中：查询附近三角形
  -> 计算该体素状态
  -> 写入缓存
  -> 返回状态
```

优点：

- 只生成 A* 实际触达的体素。
- 对短路径、小范围搜索非常节省。
- 可以自然统计缓存命中率和实际访问区域。

风险：

- 单体素查询粒度太细，可能频繁触发空间索引查询。
- A* 主循环中 walkability 查询变重。
- 调试复杂度高于预体素化。

结论：

理论上可行，但不建议作为第一步。应在三角形空间索引完成后再推进。

### 3.4 Chunk 级按需生成

思路：

```text
将体素空间划分为固定大小 chunk
例如 16 x 16 x 16 或 32 x 32 x 32

A* 查询某体素
  -> 找到所属 chunk
  -> chunk 已生成：直接查缓存
  -> chunk 未生成：一次性生成整个 chunk
  -> 返回体素状态
```

优点：

- 比单体素 lazy 查询更少重复计算。
- 比全局体素化更节省内存和时间。
- 与 A* 局部扩展行为匹配。
- 便于统计、导出和调试。

风险：

- chunk 过小会频繁生成，过大会接近局部全量体素化。
- 需要维护 chunk 状态、缓存和边界。
- 需要处理搜索范围外 chunk 是否允许生成。

结论：

这是长期推荐形态。第二阶段完成空间索引后，第三阶段优先实现 chunk lazy，而不是逐体素 lazy。

### 3.5 粗到细双阶段搜索

思路：

```text
1. 使用较大 voxelSize 建立粗体素场
2. 在粗体素场上搜索粗路径
3. 沿粗路径生成细体素走廊
4. 在细体素走廊中重新 A*
```

优点：

- 大模型路径搜索可明显降维。
- 适合远距离起终点。
- 可和局部体素化结合。

风险：

- 粗路径可能误导细路径。
- 需要设计粗细路径映射和走廊宽度。
- 开发复杂度高于局部体素化和空间索引。

结论：

作为中长期优化方向，不放在首轮实现。

## 4. 推荐路线

推荐按以下顺序推进：

```text
Phase 0: 建立性能基线与统计输出
Phase 1: 局部范围预体素化 + 失败扩张重试
Phase 2: MeshTriangle 空间索引
Phase 3: Chunk 级按需体素生成与缓存
Phase 4: 粗到细搜索与更高级代价模型
```

这条路线的原则是：

- 每一阶段都能独立带来收益或为下一阶段铺路。
- 始终保留全局体素化作为回退和对照。
- 先解决“生成范围太大”，再解决“查询三角形太多”，最后做“实时按需生成”。

## 5. Phase 0：性能基线与统计输出

### 5.1 目标

在改算法前，先量化当前流程的成本。

### 5.2 建议新增统计

```cpp
struct VoxelPlanningProfile
{
    double triangulationMs = 0.0;
    double voxelBuildMs = 0.0;
    double astarMs = 0.0;
    double optimizeMs = 0.0;

    std::size_t triangleCount = 0;
    std::size_t storedCellCount = 0;
    std::size_t occupiedCount = 0;
    std::size_t clearanceBandCount = 0;

    int astarVisitedCount = 0;
    std::size_t rawPathCount = 0;
    std::size_t optimizedPathCount = 0;
};
```

### 5.3 验收标准

- 能输出三角化、体素化、A*、路径优化耗时。
- 能输出 `CellCount`、`Occupied`、`ClearanceBand`、`visitedCount`。
- 能计算路径体素数占总存储体素数的比例。
- 当前全局流程结果不变。

## 6. Phase 1：局部范围预体素化

### 6.1 目标

只为起终点附近的候选搜索范围生成体素，减少全局预处理成本。

### 6.2 设计

新增选项：

```cpp
enum class VoxelBuildRegionMode
{
    FullMeshBounds,
    StartGoalBox
};

struct VoxelLocalBuildOptions
{
    VoxelBuildRegionMode regionMode = VoxelBuildRegionMode::FullMeshBounds;
    double searchPadding = 20.0;
    int maxRetryCount = 3;
    double retryExpandFactor = 2.0;
};
```

`StartGoalBox` 的适用边界：

- 它是性能优化/实验模式，不是 correctness baseline。
- 构建范围来自起点和终点的 AABB 加 padding，因此只保证覆盖这条直连盒附近的体素。
- 如果真实更短路径需要绕到该盒外，例如球体两极路径绕外侧走，局部盒可能裁掉有效 `ClearanceBand`，导致路径变长或失败。
- A* 失败后的扩张重试只能缓解“范围略小”的情况，不能证明局部路径质量等价于全局构建。
- 需要路径质量保证时必须使用 `FullMeshBounds`，或使用 `StartGoalBox` 后与全局/更大范围结果对比并具备回退策略。

局部范围计算：

```text
localBox = AABB(startPoint, goalPoint)
expand = clearance + halfVoxelDiagonal + searchPadding
localBox.expand(expand)
```

失败扩张：

```text
第 0 次: searchPadding
第 1 次: searchPadding * retryExpandFactor
第 2 次: searchPadding * retryExpandFactor^2
...
```

### 6.3 实现建议

- 不直接替换当前 `BuildVoxelSpaceFromShapeMesh()`。
- 新增重载或新增 builder 方法，例如 `BuildVoxelSpaceInWorldBox()`。
- 局部模式下，`VoxelSpace.origin` 使用局部盒 `minP`。
- `VoxelSpace.searchBounds` 使用局部盒对应的体素范围。
- A* 失败时导出该次局部范围的 debug VTK。

### 6.4 验收标准

- `FullMeshBounds` 模式与当前行为保持一致。
- `StartGoalBox` 模式在球体测试上能成功找到路径。
- 起终点距离较短时，`voxelBuildMs` 和 `storedCellCount` 明显下降。
- 局部范围过小时，能扩张重试并最终成功或给出明确失败原因。

## 7. Phase 2：MeshTriangle 空间索引

### 7.1 目标

减少局部体素化和后续 lazy 查询时参与距离计算的三角形数量。

### 7.2 建议数据结构

短期使用空间哈希：

```cpp
struct TriangleAabbRecord
{
    MeshTriangle triangle;
    MeshAABB aabb;
    int triangleId = -1;
};

struct TriangleSpatialHashOptions
{
    double cellSize = 10.0;
};
```

索引 key 可以使用整数三维索引：

```cpp
struct SpatialCellIndex
{
    int x = 0;
    int y = 0;
    int z = 0;
};
```

### 7.3 查询方式

体素化某个局部盒或 chunk 时：

```text
1. 构造 queryBox
2. queryBox 按 clearance + halfDiag 扩展
3. 从空间索引取候选三角形 id
4. 对候选三角形执行当前距离计算和状态标记
```

### 7.4 验收标准

- 对同一输入，使用空间索引和全量三角形遍历的体素状态结果一致或差异可解释。
- 输出候选三角形数量统计。
- 大模型或多三角形模型中，局部构建耗时下降。
- 空间索引可被 Phase 3 复用。

## 8. Phase 3：Chunk 级按需体素生成

### 8.1 目标

将体素生成从“搜索前全部完成”改为“A* 搜索过程中按 chunk 生成并缓存”。

### 8.2 核心接口设想

```cpp
class IVoxelStateProvider
{
public:
    virtual ~IVoxelStateProvider() = default;

    virtual VoxelState GetState(const VoxelIndex& index) = 0;
    virtual bool IsInsideSearchBounds(const VoxelIndex& index) const = 0;
};
```

当前 `VoxelSpace` 可以视为 eager provider；新的 chunk 缓存可以作为 lazy provider。

Chunk 缓存结构：

```cpp
struct VoxelChunkKey
{
    int x = 0;
    int y = 0;
    int z = 0;
};

struct VoxelChunk
{
    VoxelChunkKey key;
    bool generated = false;
    std::vector<VoxelCell> cells;
};
```

### 8.3 生成流程

```text
A* 查询 VoxelIndex
  -> 计算 VoxelChunkKey
  -> chunk 已存在：返回 cell state
  -> chunk 不存在：生成 chunk
      -> 计算 chunk world AABB
      -> 通过 TriangleSpatialHash 查询候选三角形
      -> 标记 Occupied / ClearanceBand
      -> 写入缓存
  -> 返回 cell state
```

### 8.4 关键决策

- chunk size 初始建议 `16` 或 `32`。
- lazy provider 初期不做 LRU 删除，只做缓存增长。
- A* 仍应受 `VoxelBounds` 约束，避免无限生成。
- VTK 导出需要支持导出已生成 chunk。

### 8.5 验收标准

- A* 能在 lazy chunk provider 上运行。
- 访问过的 chunk 数量、生成耗时、缓存命中率可统计。
- 与 eager 局部体素化结果路径一致或差异可解释。
- 短路径场景下，生成体素数显著低于局部预体素化。

## 9. Phase 4：中长期优化

### 9.1 粗到细路径搜索

目标：

```text
粗体素快速找到全局路径趋势
细体素只在粗路径走廊内精细搜索
```

适用场景：

- 起终点距离远。
- 模型范围大。
- 全局 `ClearanceBand` 很大但实际路径局部。

### 9.2 更准确的体素标记

当前使用体素中心到三角形距离。后续可改进为：

- `Occupied` 使用 triangle-box SAT 相交判断。
- `ClearanceBand` 使用 box-triangle 距离。
- 减少中心点采样导致的漏标和过度膨胀。

### 9.3 路径质量代价

可在 A* 中加入：

- 距离表面的偏好代价。
- 远离 `Occupied` 的安全代价。
- 转弯角度惩罚。
- 路径过度远离目标安全层的惩罚。

## 10. 推荐优先开发任务

### 任务 1：统计与基线

交付：

- 增加计时与数量统计。
- 输出全局体素化基线数据。
- 记录路径体素占比。

验收：

- 能回答“时间主要花在哪里”和“路径实际用了多少体素”。

### 任务 2：局部搜索盒体素化

交付：

- 新增局部 build box。
- 支持 `FullMeshBounds` / `StartGoalBox` 两种模式。
- A* 失败时扩张重试。

验收：

- 小范围路径构建体素数量下降。
- 局部失败可诊断、可扩张。

### 任务 3：三角形空间索引

交付：

- 建立 `TriangleSpatialHash`。
- 局部体素化只处理候选三角形。
- 输出候选三角形统计。

验收：

- 局部构建不再遍历全部三角形。
- 结果与全量遍历一致或差异可解释。

### 任务 4：Chunk Lazy Voxelization 原型

交付：

- 定义 state provider 抽象。
- 新增 chunk 缓存 provider。
- A* 支持从 provider 查询状态。
- 导出已生成 chunk VTK。

验收：

- 搜索过程中按需生成 chunk。
- 短路径场景生成体素数明显低于 eager 模式。

## 11. 风险与控制策略

### 11.1 局部范围漏掉可行路径

控制策略：

- 保留扩张重试。
- 记录每次局部 box。
- 失败时导出局部 `Occupied` / `ClearanceBand` / 起终点吸附结果。

### 11.2 Lazy 查询变慢

控制策略：

- 不做无索引的逐体素 lazy。
- 先做三角形空间索引。
- 优先 chunk 级生成。
- 输出缓存命中率和 chunk 生成耗时。

### 11.3 路径结果与全局体素化不一致

控制策略：

- 保留 full build 对照模式。
- 固定测试模型和参数。
- 输出差异体素和路径差异 VTK。
- 对局部边界附近路径进行额外检查。

### 11.4 调试复杂度升高

控制策略：

- 每个阶段保留 VTK 导出。
- 输出 build region / chunk bounds。
- 输出候选三角形数量。
- 输出 A* fail reason 和 snapped index。

## 12. 评价指标

每次优化都应记录：

| 指标 | 含义 |
|---|---|
| `triangleCount` | 三角形数量 |
| `candidateTriangleCount` | 实际参与局部体素化或 chunk 生成的候选三角形数量 |
| `storedCellCount` | `VoxelSpace` 或 chunk 缓存中实际存储体素数 |
| `occupiedCount` | `Occupied` 体素数 |
| `clearanceBandCount` | `ClearanceBand` 体素数 |
| `astarVisitedCount` | A* 访问节点数 |
| `rawPathCount` | 原始路径体素数 |
| `optimizedPathCount` | 优化后路径点数 |
| `voxelBuildMs` | 体素化耗时 |
| `astarMs` | A* 耗时 |
| `chunkGeneratedCount` | lazy 模式下生成的 chunk 数量 |
| `chunkCacheHitRate` | lazy 模式下 chunk 缓存命中率 |

重要派生指标：

```text
pathUsageRatio = rawPathCount / storedCellCount
visitedRatio = astarVisitedCount / storedCellCount
candidateTriangleRatio = candidateTriangleCount / triangleCount
```

这些指标用于判断优化是否真正减少了无效体素和无关三角形计算。

## 13. 当前推荐结论

短期不要直接把 A* 改成逐体素实时判断。没有三角形空间索引时，每个 walkability 查询都可能退化为扫描全量三角形，性能风险很高。

当前最稳妥路线是：

```text
先做统计基线
再做局部范围预体素化
再做三角形空间索引
最后做 chunk 级 lazy voxelization
```

这样可以逐步降低体素化范围和三角形计算规模，同时保留现有全局体素化流程作为正确性对照。

## 14. 推进状态

### 2026-04-28：Phase 0 性能基线已接入

已完成：

- 在 `main.cpp` 示例流程中增加 `VoxelPlanningProfile`。
- 增加 `ScopedTimer`，统计 shape mesh 导出、体素构建、A*、路径优化、VTK 导出耗时。
- 输出 `triangleCount`、`storedCellCount`、`occupiedCount`、`clearanceBandCount`。
- 输出 `astarVisitedCount`、`rawPathCount`、`optimizedPathCount`、`lineCheckCount`、`totalCost`。
- 输出派生指标 `pathUsageRatio` 和 `visitedRatio`。
- 保持当前全局体素化、A*、路径优化算法不变。

验证结果：

```text
Build: 在 VS x64 开发环境中执行 cmake --build out\build\x64-Debug --config Debug
Run: out\build\x64-Debug\MovementPath.exe
Result: success
```

当前球体示例基线：

| 指标 | 数值 |
|---|---:|
| `triangleCount` | 2004 |
| `storedCellCount` | 529418 |
| `occupiedCount` | 54144 |
| `clearanceBandCount` | 188200 |
| `astarVisitedCount` | 87318 |
| `rawPathCount` | 206 |
| `optimizedPathCount` | 14 |
| `pathUsageRatio` | 0.000389 |
| `visitedRatio` | 0.164932 |
| `voxelBuildMs` | 约 1.8 s |
| `astarMs` | 约 0.33 s |
| `vtkExportMs` | 约 15.8 s |

结论：

- 原始路径只占已存储体素的约 `0.039%`，说明“全局生成大量未被路径使用的体素”确实存在。
- A* 访问体素约占已存储体素的 `16.5%`，仍有较大局部化空间。
- 当前示例中 VTK 导出耗时显著高于体素构建和 A*，后续性能测试应区分“算法耗时”和“调试导出耗时”。

下一步：

- 推进 Phase 1：局部范围预体素化。
- 保留当前全局流程作为 baseline。
- 新增局部搜索盒模式，并在失败时支持扩张重试。

### 2026-04-28：Phase 1 局部范围预体素化已接入

已完成：

- 在 `VoxelMeshBuilder` 中新增 `BuildVoxelSpaceFromShapeMeshInBox()`。
- 新增 `candidateTriangleCount`，用于统计局部构建实际处理的三角形数量。
- 单三角形体素写入时增加 `IsInsideSearchBounds()` 检查，避免局部构建把盒外体素写入 `VoxelSpace`。
- 在 `main.cpp` 示例流程中新增 `VoxelBuildRegionMode::StartGoalBox`。
- 增加局部搜索盒扩张重试：`searchPadding = 20, 40, 80, 160`。
- 保留 `FullMeshBounds` 作为代码层面的回退模式。
- profile 输出新增构建模式、尝试次数、最终 padding、构建成功状态、A* 成功状态、候选三角形比例。

验证结果：

```text
Build: 在 VS x64 开发环境中执行 cmake --build out\build\x64-Debug --config Debug
Run: out\build\x64-Debug\MovementPath.exe
Result: success
```

当前球体示例 Phase 1 结果：

| 指标 | Phase 0 全局构建 | Phase 1 局部构建 |
|---|---:|---:|
| `buildRegionMode` | FullMeshBounds | StartGoalBox |
| `buildAttemptCount` | 1 | 2 |
| `finalSearchPadding` | - | 40 |
| `triangleCount` | 2004 | 2004 |
| `candidateTriangleCount` | 2004 | 1931 |
| `storedCellCount` | 529418 | 403714 |
| `occupiedCount` | 54144 | 41374 |
| `clearanceBandCount` | 188200 | 143158 |
| `astarVisitedCount` | 87318 | 69609 |
| `rawPathCount` | 206 | 242 |
| `optimizedPathCount` | 14 | 14 |
| `pathUsageRatio` | 0.000389 | 0.000599 |
| `visitedRatio` | 0.164932 | 0.172422 |
| `candidateTriangleRatio` | 1.0 | 0.963573 |
| `voxelBuildMs` | 约 1.8 s | 约 1.7 s，包含 2 次构建 |
| `astarMs` | 约 0.33 s | 约 0.27 s，包含 2 次搜索 |
| `vtkExportMs` | 约 15.8 s | 约 12.4 s |

结论：

- 局部搜索盒第一轮 padding `20` 不足，A* 失败；第二轮 padding `40` 成功，说明扩张重试机制有效。
- `storedCellCount` 从 `529418` 降到 `403714`，下降约 `23.7%`。
- `astarVisitedCount` 从 `87318` 降到 `69609`，下降约 `20.3%`。
- `candidateTriangleRatio` 仍高达 `96.4%`，说明仅靠局部盒无法显著减少三角形遍历，Phase 2 的 `TriangleSpatialHash` 仍是必要优化。
- 当前 `voxelBuildMs` 包含失败重试成本，而且每次重试都会重新三角化 shape；后续 Phase 2 或重构时应考虑复用 `MeshTriangle` 集合。

下一步：

- 推进 Phase 2：`MeshTriangle` 空间索引。
- 首先抽出可复用的三角形集合，避免局部重试时重复三角化。
- 再实现 `TriangleSpatialHash`，让局部构建只遍历空间相关三角形。

### 2026-04-28：Phase 2 MeshTriangle 空间索引基础设施已接入

已完成：

- 新增 `TriangleSpatialHash`，使用三角形 AABB 写入空间哈希网格。
- 新增 `TriangleSpatialHashOptions`，当前默认 `cellSize = max(10.0, voxelSize * 10.0)`。
- 新增 `BuildVoxelSpaceFromTriangles()`，支持复用已三角化的 `MeshTriangle` 集合执行全局构建。
- 新增 `BuildVoxelSpaceFromTrianglesInBox()`，支持复用 `MeshTriangle` 集合和可选 `TriangleSpatialHash` 执行局部构建。
- `main.cpp` 示例流程改为只三角化一次，并复用同一份 `MeshTriangle` 做 VTK 导出、空间索引构建和多次局部重试。
- profile 新增 `triangulationMs` 和 `spatialIndexBuildMs`。

验证结果：

```text
Build: 在 VS x64 开发环境中执行 cmake --build out\build\x64-Debug --config Debug
Run: out\build\x64-Debug\MovementPath.exe
Result: success
```

当前球体示例 Phase 2 结果：

| 指标 | Phase 1 局部构建 | Phase 2 空间索引 |
|---|---:|---:|
| `buildRegionMode` | StartGoalBox | StartGoalBox |
| `buildAttemptCount` | 2 | 2 |
| `finalSearchPadding` | 40 | 40 |
| `triangleCount` | 2004 | 2004 |
| `candidateTriangleCount` | 1931 | 1931 |
| `storedCellCount` | 403714 | 403714 |
| `occupiedCount` | 41374 | 41374 |
| `clearanceBandCount` | 143158 | 143158 |
| `astarVisitedCount` | 69609 | 69609 |
| `rawPathCount` | 242 | 242 |
| `optimizedPathCount` | 14 | 14 |
| `candidateTriangleRatio` | 0.963573 | 0.963573 |
| `triangulationMs` | 重试中重复执行 | 约 0.03 s，一次执行 |
| `spatialIndexBuildMs` | - | 约 0.0015 s |
| `voxelBuildMs` | 约 1.7 s，包含 2 次构建 | 约 1.7 s，包含 2 次构建 |

结论：

- Phase 2 已完成空间索引和三角形集合复用的代码基础。
- 当前球体测试中局部搜索盒仍覆盖球体大部分表面，因此 `candidateTriangleRatio` 没有下降；这是用例特性，不代表空间索引无效。
- 路径、体素数量和 A* 访问数量与 Phase 1 保持一致，说明空间索引筛选没有改变当前结果。
- 后续需要增加更能体现局部裁剪收益的测试，例如大模型中的短距离局部路径，或多个相距较远的障碍物。

下一步：

- 继续完善 Phase 2：增加局部短路径测试模型，验证 `TriangleSpatialHash` 的候选裁剪收益。
- 考虑输出空间哈希统计，例如 cell 数量、平均每 cell 三角形数量、查询 cell 数量。
- 在进入 Phase 3 前，将 profiling 和局部构建选项从 `main.cpp` 示例中整理为更正式的接口结构。

### 2026-04-28：按 4 步计划推进后的状态

本轮执行了上一节列出的 4 个方向：补充验证场景、补充空间索引统计、整理主流程结构，并为 Phase 3 增加默认关闭的通用扩展点。扩展点不在当前主流程中启用，不用于让某个测例通过或短暂优化某个场景指标。

已完成：

- 在 `main.cpp` 中增加 `VoxelPlanningScenario` 和 `VoxelPlanningRunOptions`，将单一示例流程改为可重复运行的场景驱动流程。
- 增加 `sphere_local_short_path` 局部短路径场景，用于验证空间索引在起终点较近时的裁剪收益。
- `TriangleSpatialHash` 增加 `TriangleSpatialHashStats`，输出 hash cell 数、entry 数、查询 cell 数、hash 原始命中三角形数。
- `VoxelMeshBuildResult` 和 profile 增加 `rawCandidateTriangleCount`、`hashQueryCellCount`、`hashRawTriangleCount`。
- `VoxelMeshBuilder` 增加 `AppendVoxelSpaceFromTrianglesInBox()`，用于后续向已有 `VoxelSpace` 追加构建局部 chunk；当前主流程未调用。
- `VoxelAStarOptions` 增加默认空的 `ensureCellBuilt` 回调，作为未来 lazy voxelization 的通用入口；未设置回调时 A* 行为保持不变。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
Run: out\build\x64-Debug\MovementPath.exe
Result: success
Warning: 仅存在既有 MSVC C4819 编码警告
```

当前两组场景数据：

| 指标 | `sphere_pole_to_pole` | `sphere_local_short_path` |
|---|---:|---:|
| `buildAttemptCount` | 2 | 1 |
| `finalSearchPadding` | 40 | 20 |
| `triangleCount` | 2004 | 2004 |
| `candidateTriangleCount` | 1931 | 512 |
| `rawCandidateTriangleCount` | 2004 | 554 |
| `candidateTriangleRatio` | 0.963573 | 0.255489 |
| `hashCellCount` | 554 | 554 |
| `hashEntryCount` | 6113 | 6113 |
| `hashQueryCellCount` | 2000 | 252 |
| `hashRawTriangleCount` | 6113 | 1429 |
| `storedCellCount` | 403714 | 48715 |
| `astarVisitedCount` | 69609 | 46 |
| `rawPathCount` | 242 | 14 |
| `voxelBuildMs` | 约 1.70 s | 约 0.22 s |
| `astarMs` | 约 0.28 s | 约 0.0013 s |

结论：

- Phase 2 的方向没有偏离。极点到极点路径覆盖球体大范围表面，因此候选三角形比例仍高；局部短路径场景中候选比例降到约 `25.55%`，证明空间索引对局部路径有效。
- 局部短路径中 `storedCellCount` 从 `403714` 降到 `48715`，A* 访问数从 `69609` 降到 `46`，说明“起终点附近局部体素化”是有效优化方向。
- 当前阶段不能为了让某个测例结果更好而修补 A*、walkability 判定或体素写入语义；但允许增加通用扩展点和单测，以支持后续架构演进。
- Phase 3 仍未替换默认主流程。`ensureCellBuilt` 和追加构建接口只是通用前置能力，后续必须在单测、基线和回退策略明确后，才能接入 chunk cache 或 lazy 模式。

下一步计划：

- 优先补充单测或可重复验证场景，覆盖全局构建、局部构建、空间索引候选筛选、A* 结果一致性和默认空 `ensureCellBuilt` 不改变行为。
- 将当前 profile 输出整理为测试可断言的数据结构，避免只依赖人工查看控制台日志。
- Phase 3 接入 chunk cache 前，先验证追加构建不会破坏已有 `VoxelSpace` 状态、搜索边界和路径一致性。

### 2026-04-28：规划流程接口下沉与 smoke test

本轮目标是减少 `main.cpp` 中的流程逻辑，让示例和后续测例都通过统一接口调用规划流程。

已完成：

- 新增 `VoxelPathPlanner.h/.cpp`，封装场景输入、运行选项、profile、构建重试、A*、路径优化和可选 VTK 导出流程。
- `main.cpp` 已精简为薄示例层，只负责构造场景、设置选项并调用 `VoxelPathPlanner::Plan()`。
- 新增 `VoxelPathPlannerOptions::MakeDefaultOptions()`，集中维护当前默认体素构建、局部构建和 A* 参数。
- CMake 拆分出 `MovementPathCore` 静态库，示例程序和测试程序都链接该核心库。
- 新增 `tests/VoxelPathPlannerSmokeTests.cpp`，直接调用 `VoxelPathPlanner::Plan()`，不经过 `main.cpp`。
- smoke test 覆盖局部短路径成功规划、局部候选三角形裁剪，以及 no-op `ensureCellBuilt` 不改变路径结果。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
CTest: ctest --test-dir out\build\x64-Debug --output-on-failure
Run: out\build\x64-Debug\MovementPath.exe
Result: success
Warning: 仅存在既有 MSVC C4819 编码警告
```

结论：

- 当前测例已经可以只调用 `VoxelPathPlanner` 接口，不需要复制 `main.cpp` 中的流程逻辑。
- `main.cpp` 现在只承担示例入口职责，不再作为算法流程的唯一承载点。
- 后续应继续把可断言指标沉淀到 `VoxelPathPlannerResult` / `VoxelPlanningProfile`，新增测试优先基于这些结构断言，而不是解析控制台输出。

下一步：

- 补充全局构建与局部构建结果一致性测试，重点比较路径成功性、起终点吸附、路径成本和候选三角形范围。
- 为 `AppendVoxelSpaceFromTrianglesInBox()` 增加独立测试，确认追加构建不会破坏已有体素状态和搜索边界。
- 再评估 Phase 3 chunk cache 的最小实验入口，保持默认主流程不变。

### 2026-04-28：MovementPathCore 目录化管理

本轮目标是让核心库有独立源码目录和 CMake 管理边界，避免 `src/CMakeLists.txt` 同时承担核心库、示例程序和测试目标的所有职责。

已完成：

- 新增 `src/MovementPathCore/`，将 `VoxelPathPlanner`、`VoxelMeshBuilder`、`VoxelAStar`、`VoxelSpace`、VTK 导出、路径优化等核心文件移动到该目录。
- 新增 `src/MovementPathCore/CMakeLists.txt`，在该目录内定义 `MovementPathCore` 静态库、源码列表、公开 include 路径和依赖库。
- `src/CMakeLists.txt` 改为只负责加载 OCCT 依赖、`add_subdirectory(MovementPathCore)`、创建示例程序 `MovementPath` 和 smoke test 目标。
- `main.cpp` 和 `tests/VoxelPathPlannerSmokeTests.cpp` 都通过链接 `MovementPathCore` 使用公开接口。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
CTest: ctest --test-dir out\build\x64-Debug --output-on-failure
Run: out\build\x64-Debug\MovementPath.exe
Result: success
Warning: 仅存在既有 MSVC C4819 编码警告
```

结论：

- `MovementPathCore` 已从示例入口中分离，后续核心算法、规划接口和单测可以围绕该目录演进。
- `main.cpp` 继续保持薄示例层，不应再承载新的规划流程逻辑。
- 后续新增核心模块时，应优先放入 `src/MovementPathCore/` 并在该目录 CMake 中登记。

### 2026-04-28：修复默认局部裁剪导致的路径退化

问题：

- `VoxelPathPlanner::MakeDefaultOptions()` 曾默认使用 `StartGoalBox`。
- 对 `sphere_pole_to_pole` 这类起终点距离远、最优路径需要绕过几何外侧的场景，`StartGoalBox` 只按起终点连线 AABB 加 padding 生成体素。
- 在球体两极场景中，该局部盒会裁掉部分有效 `ClearanceBand`，导致 A* 只能在被截断的候选带中找路，路径 cost 从全局构建下的约 `209.2` 退化到局部构建下的约 `244.9`。

修复：

- `VoxelPathPlanner::MakeDefaultOptions()` 默认构建模式改回 `FullMeshBounds`。
- `StartGoalBox` 保留为显式实验/优化选项，由调用方针对局部短路径等场景主动开启。
- `main.cpp` 中 `sphere_pole_to_pole` 使用默认全局构建；`sphere_local_short_path` 显式启用 `StartGoalBox`。
- smoke test 增加极点到极点对照：默认全局构建必须成功，且 cost 应低于显式局部构建结果，防止局部裁剪再次成为默认行为。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
CTest: ctest --test-dir out\build\x64-Debug --output-on-failure
Run: out\build\x64-Debug\MovementPath.exe
Result: success
```

当前 `sphere_pole_to_pole` 结果：

| 指标 | 修复前默认局部构建 | 修复后默认全局构建 |
|---|---:|---:|
| `buildRegionMode` | StartGoalBox | FullMeshBounds |
| `buildAttemptCount` | 2 | 1 |
| `rawPathCount` | 242 | 206 |
| `totalCost` | 244.9 | 209.2 |
| `storedCellCount` | 403714 | 529418 |
| `candidateTriangleRatio` | 0.963573 | 1.0 |

结论：

- 局部范围预体素化不能作为默认规划语义；它是性能优化选项，必须在路径质量可接受或有回退策略时启用。
- 当前阶段应继续保留全局构建作为 correctness baseline。
- 后续 Phase 3 或局部构建优化必须增加“路径质量不退化或可回退”的测试约束，而不能只看 candidate ratio 或 voxel build time。
- 对外接口中 `StartGoalBox` 必须被视为显式 opt-in 模式；调用方开启它时应记录原因、适用场景和回退策略。

### 2026-04-28：测试基线补强、结果结构完善与 Phase 3 原型

本轮按“先完成步骤 1、2，再推进 Phase 3”的顺序执行。

步骤 1：补强测试基线

- smoke test 继续覆盖局部短路径成功规划和候选三角形裁剪。
- 增加 `FullMeshBounds` 与 `StartGoalBox` 的极点到极点路径质量对照，防止局部盒裁剪再次成为默认行为。
- 增加 `TriangleSpatialHash` 查询测试：hash 查询结果必须覆盖 brute-force AABB 命中的三角形。
- 增加 `AppendVoxelSpaceFromTrianglesInBox()` 测试：追加构建必须扩展 bounds、填充原 bounds 外的新体素，并保持已有体素状态。

步骤 2：提炼可断言结果

- `VoxelPathPlannerResult` 新增 `optimizeResult`，测试可直接断言优化后 voxel path，而不是只看控制台输出。
- `VoxelPathPlannerResult` 新增 `finalSearchBounds` 和 `hasFinalSearchBounds`，用于后续测试搜索边界行为。

Phase 3：chunk cache 最小原型

- 新增 `VoxelChunkCache.h/.cpp`。
- `VoxelChunkCache` 以 chunk index 缓存已构建 chunk，内部调用 `AppendVoxelSpaceFromTrianglesInBox()` 追加构建。
- 当前只作为独立实验组件存在，没有接入 `VoxelPathPlanner` 默认主流程。
- 新增测试验证同一 chunk 只构建一次，第二次 `EnsureChunkForIndex()` 命中缓存，并且构建后的目标体素非 `Free`。

同时修复：

- `AppendVoxelSpaceFromTrianglesInBox()` 原先会把 append bounds 裁剪到原始 bounds 内，导致无法向外追加 chunk。
- 现在 append 时临时用 append bounds 限制写入，完成后将 `VoxelSpace` search bounds 扩展为原始 bounds 与 append bounds 的并集。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
CTest: ctest --test-dir out\build\x64-Debug --output-on-failure
Run: out\build\x64-Debug\MovementPath.exe
Result: success
Warning: 仅存在既有 MSVC C4819 编码警告
```

下一步：

- 不要立即把 `VoxelChunkCache` 接入默认规划流程。
- 先设计 lazy 模式的启用选项、统计字段和失败回退策略。
- lazy 模式接入前必须新增路径质量对照测试，至少保证默认 `FullMeshBounds` 结果仍作为回退 baseline。

### 2026-04-28：Lazy 模式选项、统计字段与回退策略设计

本轮只完成 lazy 模式的接口设计和默认关闭验证，尚未把 `VoxelChunkCache` 接入 `VoxelPathPlanner` 主搜索流程。

新增启用选项：

```cpp
enum class VoxelLazyFallbackPolicy
{
    None,
    FullMeshBoundsOnFailure
};

struct VoxelLazyBuildOptions
{
    bool enabled = false;
    VoxelChunkCacheOptions chunkCacheOptions;
    std::size_t maxChunkBuildCount = 0;
    double maxCostRegressionRatio = 0.0;
    VoxelLazyFallbackPolicy fallbackPolicy =
        VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure;
};
```

设计约束：

- `enabled = false` 是默认值，默认规划仍走 `FullMeshBounds` 或显式 `StartGoalBox`。
- `chunkCacheOptions` 只描述 chunk 尺寸和构建 padding，不直接改变当前 planner 行为。
- `maxChunkBuildCount = 0` 表示不限制；非 0 时作为防止 lazy 模式退化为大量 chunk 生成的保护阈值。
- `maxCostRegressionRatio = 0.0` 表示暂不启用路径质量阈值；未来可用于拒绝明显劣于 baseline 的 lazy 路径。
- `FullMeshBoundsOnFailure` 是默认回退策略，表示 lazy 模式失败或触发 guardrail 时应回退到全局构建。

新增 profile 统计字段：

```cpp
bool lazyBuildEnabled;
bool lazyFallbackTriggered;
std::string lazyFallbackReason;
std::size_t lazyChunkBuildCount;
std::size_t lazyCacheHitCount;
std::size_t lazyFailedBuildCount;
std::size_t lazyCandidateTriangleCount;
std::size_t lazyRawCandidateTriangleCount;
```

这些字段用于后续判断 lazy 模式是否真的减少构建范围、是否频繁 miss、是否退化为大量 chunk 构建，以及是否触发回退。

当前验证：

- smoke test 断言 `VoxelPathPlanner::MakeDefaultOptions()` 下 lazy 默认关闭。
- smoke test 断言默认 fallback policy 为 `FullMeshBoundsOnFailure`。
- 示例输出中 `Lazy build enabled = false`，chunk build/cache/fail/candidate 统计均为 `0`。
- build、CTest、示例运行均通过。

下一步接入前必须完成：

- 在 `VoxelPathPlanner` 中增加显式 lazy planning 分支，不能替换默认分支。
- lazy 分支必须把 `VoxelChunkCacheStats` 汇总到 `VoxelPlanningProfile`。
- lazy 分支必须实现 `maxChunkBuildCount` guardrail。
- lazy 分支失败或触发 guardrail 时，按 `fallbackPolicy` 回退到 `FullMeshBounds`。
- 接入后新增测试：lazy 关闭结果不变、lazy 失败可回退、chunk 统计正确、路径质量不劣于允许阈值。

### 2026-04-28：Lazy 分支接入 planner 并实现 guardrail 回退

本轮完成 lazy 模式的显式接入，但默认仍关闭。

已完成：

- `VoxelPathPlanner::Plan()` 在 `lazyBuildOptions.enabled = true` 时进入独立 lazy 分支。
- lazy 分支使用空 `VoxelSpace` 初始化完整搜索 bounds，并通过 `VoxelAStarOptions::ensureCellBuilt` 调用 `VoxelChunkCache::EnsureChunkForIndex()`。
- lazy 分支将 `VoxelChunkCacheStats` 汇总到 `VoxelPlanningProfile`。
- 实现 `maxChunkBuildCount` guardrail；超过限制时停止继续构建新 chunk，并标记 fallback reason。
- 实现 `FullMeshBoundsOnFailure` 回退：lazy A* 失败、chunk cache 配置失败、初始化失败或 guardrail 触发时，重新执行 `FullMeshBounds` 规划。
- 回退结果保留 lazy 统计字段和 fallback 状态，便于判断是否发生过 lazy 尝试。

当前未完成/刻意未做：

- `maxCostRegressionRatio` 仅作为接口保留，尚未启用路径质量阈值判断。
- lazy 模式未设为默认，也未替换 `FullMeshBounds` baseline。
- lazy 模式成功路径的 VTK 导出还不是重点验证目标，当前测试主要覆盖回退和统计。

新增测试：

- 默认选项下 lazy 必须关闭。
- 默认 fallback policy 必须是 `FullMeshBoundsOnFailure`。
- lazy guardrail 场景中，`maxChunkBuildCount = 1` 会触发 fallback。
- fallback 后规划必须成功，`buildRegionMode` 应回到 `FullMeshBounds`，并保留 lazy 统计。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
CTest: ctest --test-dir out\build\x64-Debug --output-on-failure
Run: out\build\x64-Debug\MovementPath.exe
Result: success
Warning: 仅存在既有 MSVC C4819 编码警告
```

下一步：

- 为 lazy 成功路径增加专门测试，选择足够小且局部 chunk 可覆盖的场景。
- 启用 `maxCostRegressionRatio`，实现 lazy 成功后与 fallback/baseline 的路径质量对照。
- 增加 lazy VTK/debug 输出验证，便于观察 chunk 覆盖范围。

### 2026-04-28：Chunk 相关测试覆盖补强

本轮重点补充 `VoxelChunkCache` 与 chunk 构建相关测试，尽量覆盖核心边界行为。

新增测试覆盖：

- 配置失败：拒绝 `nullptr` triangles、空 triangles、非正 `voxelSize`、非正 `chunkVoxelSize`。
- 未配置调用：`EnsureChunkForIndex()` 必须失败，并增加 `failedBuildCount`。
- 无空间索引路径：`spatialHash = nullptr` 时仍可通过 brute-force 三角形候选追加构建。
- `Clear()`：必须重置 configured 状态、统计字段和 built chunk 记录。
- 同 chunk 缓存：同一 chunk 第二次 `EnsureChunkForIndex()` 不应重复构建，应增加 `cacheHitCount`。
- 负坐标 chunk：验证 floor division 语义，`-1` 与 `-4` 属于同一 chunk，`-5` 属于相邻负 chunk。
- 跨 chunk 构建：负 chunk、相邻负 chunk、正 chunk 应分别计为不同构建。
- search bounds 扩展：追加多个 chunk 后，`VoxelSpace` bounds 应覆盖已构建负/正 chunk。
- padding 覆盖：`buildPadding` 应扩展 chunk build box，使邻近 chunk 边界外的三角形影响可以被写入。
- 统计累计：验证 `totalRawCandidateTriangleCount` 等候选统计会随 chunk 构建累积。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
CTest: ctest --test-dir out\build\x64-Debug --output-on-failure
Run: out\build\x64-Debug\MovementPath.exe
Result: success
Warning: 仅存在既有 MSVC C4819 编码警告
```

结论：

- chunk cache 的基础行为已经有单测覆盖，后续可以更安全地推进 lazy 成功路径和质量回退。
- 仍需补充更高层的 planner 级 lazy 成功测试，而不仅是 chunk cache 单元行为。

### 2026-04-28：Planner 级 Lazy 成功路径测试

本轮补充高层 planner 级 lazy 成功测试，验证 `VoxelChunkCache` 不只是单元可用，也能通过 `VoxelPathPlanner::Plan()` 的 lazy 分支完成路径规划。

新增测试：

- 新增 `MakeLazySmokeOptions()`，显式开启 `lazyBuildOptions.enabled`。
- 使用局部短路径球体场景作为 lazy 成功测试对象。
- 对同一场景先运行 `FullMeshBounds` baseline，再运行 lazy planner。
- 断言 lazy planner：
  - `success == true`。
  - `lazyBuildEnabled == true`。
  - `lazyFallbackTriggered == false`。
  - `buildRegionMode == "LazyChunks"`。
  - `lazyChunkBuildCount > 0`。
  - `lazyCandidateTriangleCount > 0`。
  - `lazyFailedBuildCount == 0`。
  - `rawPathCount > 0`。
  - `optimizeResult.voxelPath.size()` 与 `optimizedPathCount` 一致。
  - `totalCost` 不超过 `FullMeshBounds` baseline 的 `1.25` 倍。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
CTest: ctest --test-dir out\build\x64-Debug --output-on-failure
Run: out\build\x64-Debug\MovementPath.exe
Result: success
Warning: 仅存在既有 MSVC C4819 编码警告
```

结论：

- lazy 模式已具备一个 planner 级成功路径测试，不再只依赖 chunk cache 单元测试。
- 默认示例仍保持 lazy 关闭，`sphere_pole_to_pole` 继续使用 `FullMeshBounds` correctness baseline。
- 后续可以继续完善 `maxCostRegressionRatio` 的正式实现，把当前测试中的固定 `1.25` 倍质量约束下沉到 planner 选项。

### 2026-04-28：Lazy 质量回退实现与测试拆分

本轮按“下一步建议”继续执行，重点是把 lazy 质量约束下沉到 planner 选项，并拆分测试目标，避免所有行为堆在单一 smoke test 中。

已完成：

- `maxCostRegressionRatio` 已正式接入 `VoxelPathPlanner::Plan()`：lazy 成功后会在启用该阈值时运行 `FullMeshBounds` baseline 对照。
- 当 lazy 路径 cost 超过 `baselineCost * maxCostRegressionRatio` 时，planner 按 `FullMeshBoundsOnFailure` 返回 full-bounds 结果，并保留 lazy 尝试统计。
- `VoxelPathPlannerResult` 中的 `lazyAttemptCost`、`fallbackCost`、`executionMode`、`fallbackExecutionMode` 可用于断言质量回退行为。
- 原 `VoxelPathPlannerSmokeTests.cpp` 拆分为 `TriangleSpatialHashTests`、`VoxelChunkCacheTests`、`VoxelPathPlannerTests`。
- 新增 planner 级 `maxCostRegressionRatio` 回退测试，验证 lazy 尝试成功但质量阈值不满足时会回退到 `FullMeshBounds`。
- `src/CMakeLists.txt` 改为通过 `add_movement_path_test()` 注册多个独立测试目标。
- `doc/VoxelPathPlanning.md` 同步当前架构、测试入口、lazy/chunk/质量回退状态。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
CTest: ctest --test-dir out\build\x64-Debug --output-on-failure
Result: success, 3/3 tests passed
Warning: 仅存在既有 MSVC C4819 编码警告
```

当前结论：

- lazy/chunk 已经具备“可显式开启、可统计、可 guardrail、可质量回退”的最小闭环。
- 默认主流程没有被 lazy 替换，仍保持 `FullMeshBounds` 作为正确性基线。
- 测试结构已从单 smoke test 过渡到按模块分层，后续新增边界行为应优先放入对应测试目标。

下一步建议：

- 增加 lazy debug/VTK 导出或 chunk 覆盖范围导出，便于定位 lazy 生成区域与路径质量差异。
- 增加更贴近真实复杂模型的 benchmark 场景，记录 `lazyChunkBuildCount`、候选三角形比例、cost regression 和总耗时。
- 评估 `maxCostRegressionRatio` 是否需要默认推荐值；在没有足够 benchmark 前，不建议默认开启 lazy。

### 2026-04-28：Lazy chunk 覆盖范围 VTK 导出

本轮继续执行上一节“下一步建议”中的第一项，增加 lazy debug/VTK 导出能力，用于观察 lazy chunk 实际覆盖范围。

已完成：

- `VoxelChunkCache` 新增 `GetBuiltChunks()`，可返回已生成 chunk 列表。
- `VoxelVtkExporter` 新增 `ExportChunkBoundsToVtk()`，将 chunk world-space 包围盒导出为 `UNSTRUCTURED_GRID` 六面体。
- `VoxelPlanningRunOptions` 新增 `lazyChunkBoundsVtkPath`，默认值为 `D:/lazy_chunk_bounds.vtk`。
- `VoxelPathPlanner` 在 lazy 分支中，当 `runOptions.exportVtk = true` 时导出已生成 chunk 覆盖范围。
- `VoxelChunkCacheTests` 增加 chunk bounds VTK 文件结构测试。
- `VoxelPathPlannerTests` 增加 planner 级 lazy chunk bounds 导出测试，验证该能力不是孤立工具函数。
- `doc/VoxelPathPlanning.md` 同步 VTK 导出能力说明。

设计约束：

- 该导出只输出 chunk 外包围盒，不输出 chunk 内全部体素，目的是快速观察 lazy 覆盖范围。
- 默认规划仍不启用 lazy；默认示例也不会额外导出 lazy chunk bounds。
- 该能力仅在 `lazyBuildOptions.enabled = true` 且 `runOptions.exportVtk = true` 时生效。

验证结果：

```text
Build: cmake --build out\build\x64-Debug --config Debug
CTest: ctest --test-dir out\build\x64-Debug --output-on-failure
Run: out\build\x64-Debug\MovementPath.exe
Result: success, 3/3 tests passed
Warning: 仅存在既有 MSVC C4819 编码警告
```

下一步建议：

- 设计 benchmark 数据结构和输出格式，先不优化算法，只记录 full/local/lazy 在同一场景下的耗时、候选三角形、chunk 数和路径 cost。
- 增加一个非球体的稳定 benchmark 场景，避免后续优化只围绕 `sphere_pole_to_pole` 调参。
- 在 benchmark 数据足够前，继续保持 lazy 默认关闭，`maxCostRegressionRatio` 不设置默认推荐值。

### 2026-04-28：Benchmark 数据结构、CSV 输出与非球体场景

本轮落实 benchmark 基础设施，目标是先记录数据，不调整算法策略。

已完成：

- 新增 `benchmarks/VoxelPlannerBenchmark.cpp`。
- 新增可执行目标 `MovementPathBenchmark`。
- 设计 `BenchmarkCase` 和 `BenchmarkRow` 两层数据结构：
  - `BenchmarkCase` 描述场景、模式和 planner options。
  - `BenchmarkRow` 承载一次规划后的 profile/result 指标。
- 输出 CSV 文件 `voxel_planner_benchmark.csv`。
- 每个场景固定运行三种模式：
  - `full`：`FullMeshBounds`。
  - `local`：显式 `StartGoalBox`。
  - `lazy`：显式 `LazyChunks`。
- 新增非球体稳定场景 `box_long_face_to_face`：
  - 几何体为 `BRepPrimAPI_MakeBox(gp_Pnt(-30,-10,-10), gp_Pnt(30,10,10))`。
  - 起点位于左侧面中心，终点位于右侧面中心。
  - 起终点方向分别指向外侧，用于稳定吸附到外侧 `ClearanceBand`。
- 保持默认 planner 行为不变：
  - `lazyBuildOptions.enabled = false`。
  - `maxCostRegressionRatio = 0.0`。
  - benchmark 中的 lazy 仅作为显式模式运行。

CSV 字段：

```text
scene,mode,success,buildRegionMode,lazyFallbackTriggered,
lazyFallbackReason,triangulationMs,spatialIndexBuildMs,
voxelBuildMs,astarMs,lazyChunkBuildMs,
lazyCandidateQueryMs,lazyCandidateFilterMs,lazyVoxelMarkMs,
lazyStateCountMs,astarNonChunkMs,
optimizeMs,totalMeasuredMs,
triangleCount,candidateTriangleCount,rawCandidateTriangleCount,
storedCellCount,occupiedCount,clearanceBandCount,
lazyEnsureCallCount,lazyChunkBuildCount,
lazyCacheHitCount,lazyFailedBuildCount,
lazyCandidateTriangleCount,lazyRawCandidateTriangleCount,
astarVisitedCount,rawPathCount,optimizedPathCount,totalCost,
lazyAttemptCost,fallbackCost
```

本轮实测结果摘要：

```text
sphere_pole_to_pole / full  success=true cost=209.2 chunks=0   measuredMs≈2389
sphere_pole_to_pole / local success=true cost=244.9 chunks=0   measuredMs≈2022
sphere_pole_to_pole / lazy  success=true cost=209.2 chunks=236 measuredMs≈10211
box_long_face_to_face / full  success=true cost=85.2 chunks=0  measuredMs≈128
box_long_face_to_face / local success=true cost=85.2 chunks=0  measuredMs≈105
box_long_face_to_face / lazy  success=true cost=85.2 chunks=35 measuredMs≈248
```

当前结论：

- benchmark 已能稳定生成 full/local/lazy 对比数据。
- 当前 lazy 在这两个场景下并不天然更快，特别是 `sphere_pole_to_pole` 中 chunk 构建嵌入 A* 主循环后耗时明显偏高。
- 这支持继续保持 lazy 默认关闭，也支持暂不为 `maxCostRegressionRatio` 设置默认推荐值。

下一步建议：

- 将 benchmark 输出扩展为可选 JSON，方便后续自动比较多轮运行结果。
- 增加至少一个“短路径、小局部范围”的非球体 benchmark，用于观察 lazy/local 的潜在收益边界。
- 针对 lazy 耗时高的问题先做 profile 分解，不直接改算法：区分 A* 本身耗时、chunk 构建耗时和 chunk cache 命中开销。

### 2026-04-28：Lazy benchmark 耗时拆分与初步判断

针对 benchmark 中 lazy 模式路径正确但未加速的问题，本轮先补统计，不改变默认算法策略。

新增统计：

- `VoxelChunkCacheStats::ensureCallCount`：记录 A* 过程中调用 chunk ensure 的次数。
- `VoxelChunkCacheStats::totalBuildMs`：记录实际追加构建 chunk 的累计耗时。
- `VoxelPlanningProfile::lazyEnsureCallCount`。
- `VoxelPlanningProfile::lazyChunkBuildMs`。
- `VoxelPlanningProfile` 新增 `lazyCandidateQueryMs`、`lazyCandidateFilterMs`、`lazyVoxelMarkMs`、`lazyStateCountMs`。
- benchmark CSV 新增 `lazyChunkBuildMs`、`lazyCandidateQueryMs`、`lazyCandidateFilterMs`、`lazyVoxelMarkMs`、`lazyStateCountMs` 和 `astarNonChunkMs`。
- `VoxelChunkCache` 增加 last-chunk 快路径，减少连续访问同一 chunk 时的 unordered_set 查询成本。

本轮实测结果摘要：

```text
sphere_pole_to_pole / lazy:
  totalMeasuredMs≈10174
  astarMs≈10794
  lazyChunkBuildMs≈10359
  lazyCandidateQueryMs≈25
  lazyCandidateFilterMs≈5
  lazyVoxelMarkMs≈4709
  lazyStateCountMs≈5620
  astarNonChunkMs≈434
  lazyEnsureCallCount=524054
  lazyChunkBuildCount=236
  lazyCacheHitCount=523818

box_long_face_to_face / lazy:
  totalMeasuredMs≈238
  astarMs≈233
  lazyChunkBuildMs≈193
  lazyCandidateQueryMs≈1
  lazyCandidateFilterMs≈0
  lazyVoxelMarkMs≈171
  lazyStateCountMs≈20
  astarNonChunkMs≈41
  lazyEnsureCallCount=60830
  lazyChunkBuildCount=35
  lazyCacheHitCount=60795
```

判断：

- chunk cache 不是没有命中；命中次数很高，说明 chunk 缓存机制确实在工作。
- lazy 未加速的主要原因是 chunk 构建本身太贵，且当前 chunk 构建被计入 A* 阶段。
- 在当前 benchmark 中，候选查询和候选过滤不是主要瓶颈；主要耗时来自 `voxelMarkMs` 和每次 append 后对 `VoxelSpace::Cells()` 的状态统计。
- `sphere_pole_to_pole` 中 236 个 chunk 累计候选三角形数远高于全局三角形数，说明相邻 chunk 重复处理三角形影响显著。
- 对当前实现而言，lazy 更像“推迟并分批支付体素化成本”，还没有做到“显著减少总体体素化成本”。

后续优化方向必须基于这些数据推进，避免只看 `storedCellCount` 或路径正确性：

- 优先处理 append 后全量状态统计的重复扫描；该统计目前对每个 chunk 都扫描 `VoxelSpace::Cells()`。
- 再处理 `MarkTriangleToVoxelSpace()` 的重复体素标记和距离计算成本。
- 评估 chunk size 对 `lazyChunkBuildCount`、重复候选三角形和总构建耗时的影响。
- 评估是否需要对 chunk build result 做更粗粒度的 triangle influence 缓存，避免相邻 chunk 重复处理同一批三角形。
- 在这些数据收敛前，继续保持 lazy 默认关闭。

### 2026-04-29：移除 append 后全量状态统计重复扫描

本轮优先处理已确认的 lazy chunk 重复统计问题，不调整 lazy 参数。

修改：

- `AppendVoxelSpaceFromTrianglesInBox()` 不再在每个 chunk append 后扫描整个 `VoxelSpace::Cells()` 统计 `Occupied` / `ClearanceBand`。
- `VoxelMeshBuildResult` 注释明确：full/local 构建会报告当前空间状态总数，append 构建不做全局状态计数；需要全局总数时由调用方最终统一统计。
- lazy planner 原本已在 A* 结束后调用一次 `CountVoxelStates()`，因此最终 profile 的 `occupiedCount` / `clearanceBandCount` 仍保持可用。
- 保留 `lazyStateCountMs` 字段，用于确认 append 阶段是否仍存在重复状态统计。

实测结果：

```text
sphere_pole_to_pole / lazy:
  before: totalMeasuredMs≈11231, lazyStateCountMs≈5761
  after : totalMeasuredMs≈4886,  lazyStateCountMs=0

box_long_face_to_face / lazy:
  before: totalMeasuredMs≈268, lazyStateCountMs≈28
  after : totalMeasuredMs≈221, lazyStateCountMs=0
```

结论：

- append 后全量状态统计是 lazy 慢的主要浪费之一，移除后 `sphere_pole_to_pole` lazy 耗时下降明显。
- lazy 仍慢于 full，剩余主要瓶颈是 `lazyVoxelMarkMs`，也就是 `MarkTriangleToVoxelSpace()` 中的体素遍历和点到三角形距离计算。
- 下一步应分析体素标记重复计算：包括相邻 chunk 重复处理同一 triangle influence、同一 voxel 多次距离计算，以及是否可以对 triangle influence 的 voxel index range 做缓存。

### 2026-04-29：拆分 chunk 查询范围与写入范围

本轮继续处理 `MarkTriangleToVoxelSpace()` 相关重复计算。核心问题是此前 `VoxelChunkCacheOptions::buildPadding` 同时扩大了候选查询范围和 voxel 写入范围，导致相邻 chunk 会重复写入 padding 区域内的体素。

修改：

- `AppendVoxelSpaceFromTrianglesInBox()` 新增 `extraQueryPadding` 参数。
- `VoxelChunkCache::MakeChunkBuildBox()` 只返回核心 chunk world box，不再把 `buildPadding` 扩入写入 bounds。
- `VoxelChunkCache` 调用 append 时把 `buildPadding` 作为额外查询 padding 传入 builder。
- chunk max corner 使用半开区间语义处理，避免 `WorldToIndex(maxCorner)` 落到相邻 chunk 的第一个体素。
- 单测更新为验证：query padding 能让相邻三角形影响核心 chunk 边界体素，但不会扩展 voxel write bounds。

实测结果：

```text
sphere_pole_to_pole / lazy:
  before state-count fix: totalMeasuredMs≈11231, lazyVoxelMarkMs≈4986
  after state-count fix : totalMeasuredMs≈4886,  lazyVoxelMarkMs≈4440
  after write-bound fix : totalMeasuredMs≈2182,  lazyVoxelMarkMs≈1746

box_long_face_to_face / lazy:
  before state-count fix: totalMeasuredMs≈268, lazyVoxelMarkMs≈191
  after state-count fix : totalMeasuredMs≈221, lazyVoxelMarkMs≈173
  after write-bound fix : totalMeasuredMs≈113, lazyVoxelMarkMs≈68
```

结论：

- 相邻 chunk 重复写 padding 区域是 `voxelMarkMs` 高的重要原因。
- 拆分查询范围与写入范围后，lazy 在 `box_long_face_to_face` 已接近/略优于 full，在 `sphere_pole_to_pole` 中也接近 full。
- lazy 仍有大量 ensure/cache hit，`astarNonChunkMs` 约 400ms；后续可继续优化 A* lazy hook 调用频率或 chunk 命中路径，但这已经不是主要构建瓶颈。

下一步建议：

- 继续分析 `MarkTriangleToVoxelSpace()` 内部：统计被遍历 voxel 数、实际写入 voxel 数、距离计算次数。
- 对同一 triangle 在多个 chunk 中的 influence index range 做缓存，避免重复计算 triangle influence AABB 和 index range。
- 评估是否需要在 `VoxelSpace::SetCellDistanceIfSmaller()` 层增加“距离没有变小则少写状态”的细粒度统计，而不是立即改变行为。
