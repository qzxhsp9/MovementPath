# MovementPath 修改日志

本文档记录 MovementPath 项目的所有重要代码修改。模块文档分别放在 `doc/VoxelPathPlanningBaseline/` 和 `doc/GeometryQueryPathPlanner/`。

## 2026-04-30

### Lazy 统计与 benchmark 增强

- 细分状态未变化写入来源，新增 `lazyOccupiedUnchangedWriteCount` 和 `lazyClearanceUnchangedWriteCount`。
- `MovementPathBenchmark.exe` 支持 `--runs N` 多轮运行。
- benchmark CSV 增加 `runIndex` / `runCount`。
- 三轮数据表明状态未变化写入主要来自 `ClearanceBand -> ClearanceBand` 重复写入。
- 对 chunk-local 二次过滤、triangle-to-voxel 和 voxel-to-triangle 查询方式完成第一轮评估，暂不改变算法行为。
- 增加 chunk-local 二次过滤 dry-run 字段，验证 filtered candidate 基本全部 active，整 triangle 级二次过滤收益有限。

## 2026-04-29

### Lazy 统计增强

- 新增 `distanceNotImprovedCount`，用于判断距离计算中有多少没有改善 cell 最近表面距离。
- 新增 `stateUnchangedWriteCount`，用于判断状态写入中有多少没有改变原状态。
- 新增 lazy chunk candidate triangle min/max 分布统计。
- benchmark CSV 同步输出新增字段。
- 补充 chunk cache 和 planner 测试，验证统计字段自洽且不改变路径行为。

### 文档与文件结构整理

- 将 `doc/` 下文档归纳为固定文档：
- `doc/VoxelPathPlanningBaseline/ImplementationStatus.md`：体素 baseline 当前实现状态。
- `doc/VoxelPathPlanningBaseline/Roadmap.md`：体素 baseline 进度计划。
- `doc/VoxelPathPlanningBaseline/OptimizationPlan.md`：体素 baseline 性能优化计划。
- `doc/ChangeLog.md`：全项目每次重要修改后的日志。
- benchmark CSV 默认输出从仓库根目录改为 `doc/VoxelPathPlanningBaseline/voxel_planner_benchmark.csv`。

### 路径显示端点对齐接口入参

- A* 和 optimizer 的内部 `voxelPath` 保持 snapped voxel 序列。
- planner 输出的 `pointPath` 首尾替换为接口入参 `startPoint` / `goalPoint`。
- `optimized_path_polyline.vtk` 改为导出修正后的 `pointPath`。
- 补充 local/lazy planner 测试，验证 A* 和 optimized 点路径首尾均等于接口入参。

### Lazy VTK 导出补齐

- lazy 成功分支补齐 `astarPathVtkPath` 导出。
- lazy 优化后补齐 `optimizedPathVoxelsVtkPath` 和 `optimizedPathPolylineVtkPath` 导出。
- 扩展 lazy export 测试，确保 `exportVtk=true` 时路径和 chunk bounds 都会生成。
- 明确 `lazy_chunk_bounds.vtk` 表示实际构建的 chunk 集合，不表示封闭 mesh 包壳。

### Mark loop 按写入 bounds 裁剪

- `MarkTriangleToVoxelSpace()` 增加 `markBounds`。
- full/local 传构建 bounds，lazy append 传 chunk append bounds。
- 消除 lazy chunk 外 out-of-bounds voxel visit。
- 增加 append 裁剪测试，验证裁剪前后 voxel state/distance 一致。

### Triangle influence range 缓存

- 增加 `VoxelTriangleInfluenceRange` 和 `VoxelMeshBuildCache`。
- lazy chunk build 按 triangle id 缓存 influence AABB 和 index range。
- 增加 cache hit/miss 统计。
- 增加 chunk cache 测试，验证跨 chunk 复用 influence range。

### Benchmark 结构与统计字段

- 增加 benchmark 数据结构和 CSV 输出。
- 同一场景下记录 full/local/lazy 的耗时、候选三角形、chunk 数和路径 cost。
- 增加 `box_long_face_to_face` 非球体 benchmark 场景。
- 增加 lazy 内部统计字段：voxel visit、out-of-bounds visit、distance calculation、distance improved、state write、influence cache hit/miss。

### Append 状态统计去重

- `AppendVoxelSpaceFromTrianglesInBox()` 不再每个 chunk 后扫描整个 `VoxelSpace::Cells()`。
- lazy planner 在 A* 后统一执行一次最终状态统计。
- 保留 `lazyStateCountMs` 用于确认 append 阶段没有重复状态扫描。

### Chunk 查询范围与写入范围拆分

- `AppendVoxelSpaceFromTrianglesInBox()` 增加 `extraQueryPadding`。
- `VoxelChunkCache::MakeChunkBuildBox()` 只返回 chunk core write box。
- `buildPadding` 只扩展候选查询，不扩展写入范围。
- 修复 chunk max corner 半开区间问题，避免写入相邻 chunk。

### VoxelPathPlanner 模块拆分

- 将核心规划逻辑整理到 `src/VoxelPathPlanner/`。
- `main.cpp` 逐步简化为示例调用入口。
- 新增/调整 `src/VoxelPathPlanner/CMakeLists.txt`，由体素 baseline 库统一管理 planner、builder、A*、optimizer、exporter 等文件。

## 2026-04-28

### Lazy 模式接入前设计

- 设计 lazy 模式启用选项。
- 设计 lazy fallback 策略。
- 增加 profile 统计字段。
- 初步实现 `VoxelChunkCache` 并增加单元测试。

### 局部构建与空间索引

- 推进 `StartGoalBox` 局部范围预体素化。
- 记录 `StartGoalBox` 对长绕行路径的局限性。
- 增加 `TriangleSpatialHash` 降低候选三角形扫描成本。

## 验证命令

常用命令：

```powershell
cmd.exe /c "call ""D:\software\ide\vs\Microsoft Visual Studio2022\Community\VC\Auxiliary\Build\vcvars64.bat"" && cmake --build out\build\x64-Debug --config Debug --target VoxelPathPlannerTests MovementPathBenchmark"
ctest --test-dir out\build\x64-Debug --output-on-failure
out\build\x64-Debug\MovementPathBenchmark.exe
```

### 新长期规划模块

- 将现有 `VoxelPathPlanner` 体素路径规划定位为 baseline。
- 新增 `src/GeometryQueryPathPlanner/` 独立目录。
- 新增独立 library target `GeometryQueryPathPlanner`。
- 新模块当前不链接 `main.cpp`、benchmark 或既有 voxel 测试，避免影响 baseline。
- 新增 `GeometryPrimitives.h`、`GeometryQueryPathPlanner.h/.cpp` 和模块 README，作为后续 BVH/按需几何查询/连续路径优化的实现边界。

### 模块命名与文档拆分

- 将新模块源码目录从 `src/MovementPathGeometryPlanner/` 调整为 `src/GeometryQueryPathPlanner/`。
- 将新模块 planner 文件从 `GeometryQueryPlanner.*` 调整为 `GeometryQueryPathPlanner.*`。
- 将新模块 CMake target 调整为 `GeometryQueryPathPlanner`。
- 将体素 baseline 文档移动到 `doc/VoxelPathPlanningBaseline/`。
- 新增 `doc/GeometryQueryPathPlanner/Roadmap.md` 管理长期几何查询规划器计划。
- `doc/ChangeLog.md` 保持为全项目唯一变更日志。

### 体素 baseline 模块重命名

- 将源码目录从 `src/MovementPathCore/` 重命名为 `src/VoxelPathPlanner/`。
- 将 CMake library target 从 `MovementPathCore` 重命名为 `VoxelPathPlanner`。
- 更新 `main.cpp`、benchmark 和测试 target 的链接依赖。
- 更新文档和模块 README 中的 baseline 模块名称。

### 模块 README 归位

- 删除 `doc/VoxelPathPlanningBaseline/README.md`。
- 删除 `doc/GeometryQueryPathPlanner/README.md`。
- 模块说明统一放在各自源码目录下的 README：
- `src/VoxelPathPlanner/README.md`
- `src/GeometryQueryPathPlanner/README.md`
- `doc/` 仅保留模块设计/计划/状态文档和全项目 `ChangeLog.md`。

### GeometryQueryPathPlanner 几何查询基础

- 新增 `GeometryQueryPathPlannerTests` 测试 target。
- 补充 planner scaffold 行为测试：空 mesh 返回 `InvalidInput`，非空 mesh 当前返回 `NotImplemented`。
- 新增 `GeometryQueries.h/.cpp`，实现点到三角形最近点、点到三角形距离、三角形 AABB、点到 AABB 距离。
- 新增暴力 `ClosestPointToMeshBruteForce()`，作为后续空间索引 correctness oracle。
- 新增 `TriangleAabbTree.h/.cpp`，实现第一版三角形 AABB tree、AABB 查询和 closest-point 查询。
- 单测覆盖点到三角形面内、边、顶点、退化三角形，AABB tree 查询与暴力结果对照。
- 新增 `TriangleMesh.h/.cpp`，统一三角形集合和 mesh AABB 管理。
- 新增 segment-to-triangle 距离和 segment clearance 暴力查询。
- `TriangleAabbTree` 新增 segment clearance 查询，当前采用保守全遍历，保证与暴力 oracle 一致。
- 扩展几何测试，覆盖空树、单三角形、多层树、边界相交和 segment clearance 对照。
- 新增 `GeometryQueryContext.h/.cpp`，组合 `TriangleMesh`、`TriangleAabbTree` 和 query profile。
- `GeometryQueryPathPlanner::Plan()` 当前会构建 query context 并记录 `spatialIndexBuildMs`、`spatialIndexNodeCount`，但仍返回 `NotImplemented`。
- `GeometryPathProfile` 增加候选三角形、最大候选数、空间索引节点数和访问节点数统计字段。
- `SegmentClearance` 增加 `radius` 参数，支持线段安全半径/capsule clearance 语义。
- `SegmentClearanceResult` 增加 `clearance`、`radius` 和 `requiredDistance` 字段。
- `TriangleAabbTreeStats` 与 `GeometryPathProfile` 增加 segment clearance dry-run 剪枝统计字段，当前只统计不改变遍历行为。
- 补充 radius 与 dry-run 统计单测。
