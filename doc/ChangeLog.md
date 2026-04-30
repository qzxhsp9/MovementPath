# MovementPath 修改日志

本文档记录重要开发修改。当前实现状态见 `ProjectImplementationStatus.md`，总体计划见 `ProjectRoadmap.md`，性能优化计划见 `VoxelPathPlanningOptimizationPlan.md`。

## 2026-04-29

### Lazy 统计增强

- 新增 `distanceNotImprovedCount`，用于判断距离计算中有多少没有改善 cell 最近表面距离。
- 新增 `stateUnchangedWriteCount`，用于判断状态写入中有多少没有改变原状态。
- 新增 lazy chunk candidate triangle min/max 分布统计。
- benchmark CSV 同步输出新增字段。
- 补充 chunk cache 和 planner 测试，验证统计字段自洽且不改变路径行为。

### 文档与文件结构整理

- 将 `doc/` 下文档归纳为四份固定文档：
- `ProjectImplementationStatus.md`：当前实现状态。
- `ProjectRoadmap.md`：当前与最终进度计划。
- `VoxelPathPlanningOptimizationPlan.md`：性能优化计划。
- `ChangeLog.md`：每次重要修改后的日志。
- benchmark CSV 默认输出从仓库根目录改为 `doc/voxel_planner_benchmark.csv`。

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

### MovementPathCore 拆分

- 将核心规划逻辑整理到 `src/MovementPathCore/`。
- `main.cpp` 逐步简化为示例调用入口。
- 新增/调整 `src/MovementPathCore/CMakeLists.txt`，由核心库统一管理 planner、builder、A*、optimizer、exporter 等文件。

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
