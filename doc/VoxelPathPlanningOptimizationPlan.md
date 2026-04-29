# Voxel Path Planning 性能优化计划

本文档只记录性能优化相关的设计、数据和下一步实验。项目实现状态见 `ProjectImplementationStatus.md`，总体阶段计划见 `ProjectRoadmap.md`，逐次修改记录见 `ChangeLog.md`。

## 优化原则

- 不为单个测例硬编码逻辑。
- 不为了让测试短期通过而掩盖算法缺陷。
- lazy/local 优化必须以 full build 作为路径质量基线。
- 在 benchmark 数据不足前，lazy 默认关闭。
- `maxCostRegressionRatio` 不设置默认推荐值，只由调用方显式开启。
- 先补统计和测试，再改变算法行为。

## Benchmark 基准

CSV 输出：

```text
doc/voxel_planner_benchmark.csv
```

当前 benchmark 场景：

- `sphere_pole_to_pole`
- `box_long_face_to_face`

关键字段：

- 耗时：`triangulationMs`、`spatialIndexBuildMs`、`voxelBuildMs`、`astarMs`、`lazyChunkBuildMs`、`lazyVoxelMarkMs`、`optimizeMs`、`totalMeasuredMs`
- 规模：`triangleCount`、`candidateTriangleCount`、`storedCellCount`
- lazy：`lazyEnsureCallCount`、`lazyChunkBuildCount`、`lazyCacheHitCount`
- 标记：`lazyVoxelVisitCount`、`lazyOutOfBoundsVoxelCount`、`lazyDistanceCalculationCount`、`lazyDistanceImprovedCount`、`lazyStateWriteCount`
- 质量：`totalCost`、`lazyAttemptCost`、`fallbackCost`

## 已完成优化

### 1. Full/local/lazy benchmark 结构

状态：完成。

- 增加 benchmark 数据结构和 CSV 输出。
- 同一场景下运行 full/local/lazy。
- 增加非球体场景，避免只围绕 `sphere_pole_to_pole` 调参。

### 2. Append 后全量状态统计去重

状态：完成。

问题：lazy 每次 append chunk 后扫描整个 `VoxelSpace::Cells()` 统计状态，导致重复全量扫描。

处理：

- append build 不再统计全局 `Occupied` / `ClearanceBand`。
- planner 在 lazy A* 结束后统一统计一次最终状态。

效果：

- `lazyStateCountMs` 降为 0。
- lazy 总耗时明显下降。

### 3. Chunk 查询范围与写入范围拆分

状态：完成。

问题：`buildPadding` 同时扩大候选查询范围和 voxel 写入范围，导致相邻 chunk 重复写 padding 区域。

处理：

- `AppendVoxelSpaceFromTrianglesInBox()` 增加 `extraQueryPadding`。
- `VoxelChunkCache::MakeChunkBuildBox()` 只返回 chunk core write box。
- `buildPadding` 只用于候选 query，不扩大 write bounds。
- chunk max corner 使用半开区间语义，避免落入相邻 chunk。

效果：

- 减少相邻 chunk 重复写入。
- `box_long_face_to_face` lazy 接近或优于 full/local。

### 4. Triangle influence range 缓存

状态：完成。

问题：同一 triangle 在多个 chunk 中重复计算 influence AABB 和 voxel index range。

处理：

- 增加 `VoxelTriangleInfluenceRange`。
- 增加 `VoxelMeshBuildCache::triangleInfluence`。
- lazy chunk build 按 triangle id 缓存 influence box 和 index range。

结论：

- cache hit/miss 统计正常。
- 该优化减少重复基础计算，但主要瓶颈仍在 voxel 遍历和距离计算。

### 5. Mark loop 按写入 bounds 裁剪

状态：完成。

问题：`MarkTriangleToVoxelSpace()` 原先遍历完整 triangle influence range，再通过 `IsInsideSearchBounds()` 丢弃 chunk 外 voxel。

处理：

- `MarkTriangleToVoxelSpace()` 增加 `markBounds`。
- 循环前把 influence range 与当前 write bounds 求交集。
- full/local 传自身 bounds，lazy append 传当前 chunk append bounds。

效果：

```text
sphere_pole_to_pole / lazy:
  lazyOutOfBoundsVoxelCount: 21893788 -> 0
  lazyVoxelVisitCount:      27034963 -> 5141175

box_long_face_to_face / lazy:
  lazyOutOfBoundsVoxelCount: 3077852 -> 0
  lazyVoxelVisitCount:       3213056 -> 135204
```

结论：

- chunk 外无效遍历已消除。
- sphere 场景剩余耗时主要来自有效范围内的点到三角形距离计算和重复状态写入。

## 当前瓶颈判断

当前主要瓶颈不是 chunk cache 没有命中，而是有效 chunk 内仍有大量 triangle/voxel 组合需要计算距离。

需要继续观察：

- `distanceCalculationCount` 与 `distanceImprovedCount` 的比例。
- `stateWriteCount` 与最终 `storedCellCount` 的比例。
- 单个 chunk 内是否存在大量同一 voxel 被多个 triangle 重复计算但没有改善距离。
- triangle 候选过滤是否仍过宽。

## 下一步优化计划

### Step 1：补充更细粒度统计

- 统计距离未改善次数。
- 统计状态写入前后状态是否变化。
- 统计每个 chunk 的 candidate triangle 分布。
- 统计每个 chunk 的 voxel visit / distance calculation 分布。

### Step 2：评估 SetCellDistanceIfSmaller 细粒度统计

原则：先统计，不改变行为。

可能字段：

- `distanceNotImprovedCount`
- `stateUnchangedWriteCount`
- `occupiedOverwriteCount`
- `clearanceOverwriteCount`

### Step 3：改进有效范围内候选过滤

候选方向：

- 按 chunk-local influence intersection 进一步过滤 triangle。
- 评估 triangle-to-voxel 分派，减少远离当前 voxel 的 triangle 距离计算。
- 评估 voxel-to-nearby-triangle 查询，但需要控制空间索引和内存成本。

### Step 4：benchmark 稳定化

- 支持多轮运行并输出均值/最小值/最大值。
- 可选 JSON 输出，方便后续自动对比。
- 增加短路径、小局部范围的非球体 benchmark。

## 不做事项

- 不把 lazy 默认打开。
- 不为 `sphere_pole_to_pole` 写专用逻辑。
- 不因为某次 benchmark 波动调整算法参数。
- 不移除 full build correctness baseline。
