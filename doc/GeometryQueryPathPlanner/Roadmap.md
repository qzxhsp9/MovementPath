# GeometryQueryPathPlanner 开发计划

本文档维护 `src/Algorithms/GeometryQueryPathPlanner/` 的长期开发计划。该模块用于验证“基于几何查询的路径规划”是否能在长期性能和路径质量上优于当前体素 baseline。

体素 baseline 的状态和计划见 `doc/VoxelPathPlanningBaseline/`。全项目修改记录见 `doc/ChangeLog.md`。

## 模块定位

`GeometryQueryPathPlanner` 不做完整预体素化，而是围绕 mesh 空间索引、按需最近距离查询、碰撞/clearance 判断、局部缓存和连续路径优化构建路径规划能力。

当前原则：

- 与 `VoxelPathPlanner` 源码隔离，不直接 include 体素模块头文件。
- 当前体素方案继续作为 correctness、性能和路径质量 baseline。
- 新模块先建立独立测试和 benchmark，再考虑接入 `main.cpp`。
- 不为了单个场景调参或写特例逻辑。
- 每一步先补统计、测试或 benchmark，再决定是否改变算法行为。

## 总体技术路线

目标不是简单替换 A*，而是减少“全局预计算”和“大量无效 triangle-voxel 距离计算”。

推荐路线：

1. 建立 mesh 空间索引，支持快速查询点/线段附近三角形。
2. 提供统一的 clearance/collision 几何查询接口。
3. 基于查询接口构建按需搜索图，节点和边只在搜索前沿附近触发几何判断。
4. 对离散路径做连续空间 shortcut/smoothing，并用 clearance 查询保护路径质量。
5. 与体素 baseline 做同场景对比，包括耗时、查询次数、路径长度、clearance 违规次数和失败原因。

## Phase 0：模块边界与基础类型

状态：已完成。

目标：

- 保持 `GeometryQueryPathPlanner` 独立 CMake target。
- 保持独立 `Vec3`、`Triangle`、`Aabb` 等基础类型。
- 保持 `GeometryQueryPathPlanner::Plan()` 作为对外入口。

已完成：

- 新增 `src/Algorithms/GeometryQueryPathPlanner/`。
- 新增 `GeometryPrimitives.h`。
- 新增 `GeometryQueryPathPlanner.h/.cpp`。
- 新增独立 CMake target `GeometryQueryPathPlanner`。
- 新增 `GeometryQueryPathPlannerTests` 测试 target。
- 已验证空 mesh 返回 `InvalidInput`。
- 已验证非空 mesh 当前返回 `NotImplemented`。

模块约束：

- 当前模块仍不链接、不 include `VoxelPathPlanner`。

## Phase 1：几何查询基础

状态：进行中。

目标：

- 建立不依赖体素的 mesh 查询能力。
- 先保证几何查询正确，再考虑搜索算法。

建议新增组件：

- `TriangleMesh`：持有三角形集合和 mesh AABB。
- `TriangleAabbTree`：三角形 AABB tree 空间索引，已完成第一版。
- `GeometryQueries`：点到三角形距离、暴力 closest-point mesh 查询，已完成第一版。
- `GeometryQueryContext`：组合 mesh、AABB tree 和 query profile，已完成第一版。
- `GeometryCollisionQuery`：线段/胶囊体与 mesh 的 clearance 检查；当前已完成 segment clearance 暴力 oracle、radius 语义与 AABB tree 保守遍历版本。
- `GeometryQueryProfile`：记录 query 次数、candidate triangle 数、耗时。

关键接口建议：

```cpp
struct ClosestPointResult
{
    bool hit = false;
    double distance = 0.0;
    Vec3 closestPoint;
    int triangleId = -1;
};

struct SegmentClearanceResult
{
    bool pass = false;
    double minDistance = 0.0;
    std::size_t testedTriangleCount = 0;
};
```

验收标准：

- 点到单三角形距离测试覆盖面内、边上、顶点附近、退化三角形。已完成。
- AABB tree 查询候选集合不漏三角形。已完成基础测试。
- AABB tree closest-point 查询结果与暴力全量扫描一致。已完成基础测试。
- 线段 clearance 查询结果与暴力全量扫描一致。已完成基础测试。
- 线段 radius/capsule clearance 语义已完成基础测试。
- segment clearance dry-run 剪枝统计已接入 profile，但尚未改变遍历行为。
- segment clearance 剪枝评估已升级为独立 dry-run 模拟，可估算启用剪枝后的访问节点数、测试三角形数、跳过三角形数和剪枝来源。
- segment-triangle 边界测试已覆盖共面穿越、端点接触、边重叠、近平行、退化线段和退化三角形。
- profile 能输出查询次数、候选数、耗时。已完成 `GeometryQueryContext` 基础统计。

## Phase 2：按需搜索图

状态：接口设计已开始，搜索算法未开始。

目标：

- 用几何查询替代大范围预体素化。
- 搜索过程中只判断当前需要扩展的节点和边。

可选搜索策略：

- 规则网格节点，但不预先标记所有 voxel，只在访问节点时做 clearance 查询。
- 自适应采样图，节点密度随障碍物距离和路径复杂度调整。
- 双向 A* / ARA* / Lazy PRM 作为后续候选，不在第一版直接引入复杂算法。

第一版建议：

- 使用轻量规则网格或步长采样作为搜索图，便于和 voxel baseline 对比。
- 节点可行性：点到 mesh 距离 >= clearance。
- 边可行性：segment/capsule clearance 通过。
- 节点和边查询结果使用局部 cache。
- 当前已新增 `GeometrySearchGraph` 接口，包含节点、边、边 clearance 结果和基础 profile；暂不执行图搜索。

需要记录的 profile：

- expandedNodeCount
- generatedEdgeCount
- nodeClearanceQueryCount
- edgeClearanceQueryCount
- geometryCacheHitCount
- geometryCacheMissCount
- averageCandidateTriangleCount
- maxCandidateTriangleCount
- searchMs

验收标准：

- 简单无障碍场景返回 start-goal 直线路径。
- 单盒体绕行场景能找到绕行路径。
- 失败时返回明确失败原因，而不是空路径。
- 同一场景下路径 cost、查询次数和耗时可与 `VoxelPathPlanner` benchmark 对比。

## Phase 3：局部缓存与性能稳定化

状态：未开始。

目标：

- 避免搜索过程中重复查询同一空间区域。
- 控制几何查询索引和 cache 的构建成本。

候选方向：

- 点查询 cache：按空间 cell 缓存 clearance 结果。
- 边查询 cache：按端点 cell pair 缓存 segment clearance 结果。
- candidate triangle cache：缓存 AABB/BVH 查询结果。
- query budget：限制单次搜索最大查询数，失败时返回可诊断原因。

必须避免：

- 重新演变成隐式全局体素化。
- 为 `sphere_pole_to_pole` 等单一场景写专用策略。
- 只优化耗时而不检查路径 clearance 和 cost。

验收标准：

- cache 命中率和 miss 成本进入 profile。
- cache 开关前后路径结果一致或差异可解释。
- 多轮 benchmark 输出稳定，能和 baseline 同表对比。

## Phase 4：连续路径质量优化

状态：未开始。

目标：

- 不只得到离散可行路径，还要提升路径长度、平滑度和 clearance 稳定性。

候选方法：

- line-of-sight shortcut。
- 基于 clearance query 的 segment replacement。
- 路径点平滑，但每次移动后检查 clearance。
- 必要时加入最大曲率、方向约束或端点切向约束。

验收标准：

- 优化后路径 cost 不高于原始搜索路径。
- 优化后所有 segment 满足 clearance。
- 起点和终点保持接口入参位置。
- 输出 path profile，包括 shortcut 次数、失败次数、优化耗时。

## Phase 5：接入与对比

状态：未开始。

目标：

- 在新模块有独立测试和 benchmark 后，再接入上层示例和统一 benchmark。

接入顺序：

1. 保持并扩展 `GeometryQueryPathPlannerTests`。
2. 新增独立 benchmark 场景，不复用体素内部类型。
3. 增加 adapter，将 OCCT triangulation 结果转换为 `geometry::Triangle`。
4. 在 benchmark 中同场景输出 `voxel_full`、`voxel_lazy`、`geometry_query`。
5. 最后再考虑 `main.cpp` 示例入口。

对比指标：

- totalMeasuredMs
- spatialIndexBuildMs
- searchMs
- optimizeMs
- geometryQueryCount
- averageCandidateTriangleCount
- pathCost
- minClearance
- failureReason

第一版 geometry benchmark 字段设计：

- `scene`：场景名。
- `mode`：固定为 `geometry_query`，后续可扩展为 `geometry_query_direct`、`geometry_query_grid`。
- `runIndex` / `runCount`：多轮 benchmark 标识。
- `success`：是否成功返回可行路径。
- `status`：`GeometryPathStatus` 字符串。
- `failureReason`：失败或未实现原因。
- `totalMeasuredMs`：外层总耗时。
- `spatialIndexBuildMs`：mesh/query context 构建耗时。
- `searchMs`：搜索耗时。
- `optimizeMs`：连续路径优化耗时。
- `triangleCount`：输入三角形数。
- `spatialIndexNodeCount`：AABB tree 节点数。
- `spatialIndexVisitedNodeCount`：查询访问节点总数。
- `spatialIndexDryRunPrunableNodeCount`：dry-run 评估可剪枝节点数。
- `spatialIndexDryRunClearanceSafeNodeCount`：dry-run 评估可直接判定 clearance 安全的节点数。
- `geometryQueryCount`：点/最近距离查询次数。
- `collisionQueryCount`：segment/capsule clearance 查询次数。
- `geometryCandidateTriangleCount`：实际测试 triangle 次数累计。
- `maxCandidateTriangleCount`：单次 query 最大测试 triangle 数。
- `searchNodeCount`：搜索图节点数。
- `searchEdgeCount`：搜索图边数。
- `feasibleSearchEdgeCount`：clearance 通过的边数。
- `blockedSearchEdgeCount`：clearance 未通过的边数。
- `rawPathCount`：原始路径点数。
- `optimizedPathCount`：优化后路径点数。
- `pathCost`：路径长度。
- `minClearance`：路径最小 clearance。

## 当前下一步

优先执行：

1. 给 `GeometrySearchGraph` 增加第一版 graph builder 草案，生成 start/goal 节点和候选直连边，但仍不做复杂搜索。
2. 设计节点可行性 query 接口：点到 mesh clearance >= clearance。
3. 基于 dry-run 数据决定是否增加可开关的 segment clearance AABB 剪枝实现。
4. 继续保持 `Plan()` 返回 `NotImplemented`，直到 graph builder 和 benchmark 字段稳定。
5. 后续新增 geometry benchmark 时输出 dry-run 剪枝评估字段。
