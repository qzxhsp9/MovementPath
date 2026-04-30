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

状态：完成。

- 统计距离未改善次数。
- 统计状态写入前后状态是否变化。
- 统计每个 chunk 的 candidate triangle 分布。
- 统计每个 chunk 的 voxel visit / distance calculation 分布。

### Step 2：评估 SetCellDistanceIfSmaller 细粒度统计

状态：已完成第一轮统计，不改变行为。

原则：先统计，不改变行为。

可能字段：

- `distanceNotImprovedCount`
- `stateUnchangedWriteCount`
- `occupiedOverwriteCount`
- `clearanceOverwriteCount`

## 2026-04-29 统计增强结果

本轮只增强统计，不改变体素距离、状态写入或路径搜索行为。

新增字段：

- `lazyDistanceNotImprovedCount`
- `lazyStateUnchangedWriteCount`
- `lazyMinCandidateTriangleCount`
- `lazyMaxCandidateTriangleCount`
- `lazyMinRawCandidateTriangleCount`
- `lazyMaxRawCandidateTriangleCount`

本轮 benchmark 摘要：

```text
sphere_pole_to_pole / lazy:
  distanceCalculationCount=5141175
  distanceImprovedCount=1449289
  distanceNotImprovedCount=3691886
  stateWriteCount=1533215
  stateUnchangedWriteCount=1259536
  candidateTriangleCount min/max=5/157
  rawCandidateTriangleCount min/max=49/294

box_long_face_to_face / lazy:
  distanceCalculationCount=135204
  distanceImprovedCount=63886
  distanceNotImprovedCount=71318
  stateWriteCount=72808
  stateUnchangedWriteCount=31381
  candidateTriangleCount min/max=4/10
  rawCandidateTriangleCount min/max=4/10
```

初步判断：

- `sphere_pole_to_pole` 中距离未改善约占 71.8%，状态未变化写入约占 82.2%，无效计算和重复写入比例很高。
- `box_long_face_to_face` 中距离未改善约占 52.7%，状态未变化写入约占 43.1%，仍有优化空间但压力低于球体场景。
- 当前数据支持继续研究 chunk-local 二次过滤，而不是优先转向并行化或 benchmark 格式调整。
- 由于状态未变化写入比例高，后续可以继续细分 `Occupied` 覆盖、`ClearanceBand` 覆盖和状态变化类型，但仍应先统计再改变行为。

### Step 3：改进有效范围内候选过滤

状态：已完成第一轮评估，尚未改变算法行为。

候选方向：

- 按 chunk-local influence intersection 进一步过滤 triangle。
- 评估 triangle-to-voxel 分派，减少远离当前 voxel 的 triangle 距离计算。
- 评估 voxel-to-nearby-triangle 查询，但需要控制空间索引和内存成本。

### Step 4：benchmark 稳定化

- 支持多轮运行，CSV 增加 `runIndex` / `runCount`。
- 可选 JSON 输出，方便后续自动对比。
- 增加短路径、小局部范围的非球体 benchmark。

## 不做事项

- 不把 lazy 默认打开。
- 不为 `sphere_pole_to_pole` 写专用逻辑。
- 不因为某次 benchmark 波动调整算法参数。
- 不移除 full build correctness baseline。

## 2026-04-30 下一步评估执行结果

本轮继续保持“不改变体素判定行为”，只增强统计、benchmark 和方案评估。

新增统计：

- `lazyOccupiedUnchangedWriteCount`
- `lazyClearanceUnchangedWriteCount`
- benchmark CSV 增加 `runIndex` / `runCount`，支持 `MovementPathBenchmark.exe --runs N` 多轮输出。

三轮 benchmark lazy 均值摘录：

```text
sphere_pole_to_pole / lazy:
  totalMeasuredMs avg≈2854.8
  lazyVoxelMarkMs avg≈2393.2
  distanceCalculationCount=5141175
  distanceNotImprovedCount=3691886
  stateWriteCount=1533215
  stateUnchangedWriteCount=1259536
  occupiedUnchangedWriteCount=64618
  clearanceUnchangedWriteCount=1194918
  candidateTriangleCount min/max=5/157

box_long_face_to_face / lazy:
  totalMeasuredMs avg≈126.8
  lazyVoxelMarkMs avg≈77.9
  distanceCalculationCount=135204
  distanceNotImprovedCount=71318
  stateWriteCount=72808
  stateUnchangedWriteCount=31381
  occupiedUnchangedWriteCount=3074
  clearanceUnchangedWriteCount=28307
  candidateTriangleCount min/max=4/10
```

评估结论：

- 状态未变化写入主要来自 `ClearanceBand -> ClearanceBand`，不是 `Occupied -> Occupied`。
- `sphere_pole_to_pole` 的候选三角形分布很宽，单个 chunk 的 filtered candidate 可到 157，raw candidate 可更高；chunk-local 二次过滤有继续评估价值。
- `box_long_face_to_face` 的 candidate 分布很窄，继续做复杂二次过滤收益可能有限。
- 当前数据更支持“按场景/按分布启用更细过滤”，而不是全局替换成更复杂的查询结构。

对三种方向的反思：

- chunk-local 二次过滤：最贴近现有架构，风险最低。可以先在 chunk 内进一步按 influence range 与 chunk 子块交集统计潜在减少量，再决定是否真正跳过计算。
- triangle-to-voxel：当前实现就是 triangle 主导扫描；继续优化应围绕减少 triangle 覆盖的 voxel 和候选 triangle，而不是简单缓存更多结果。
- voxel-to-triangle：理论上能减少无效 triangle 距离计算，但需要 chunk 内 BVH/空间索引，构建成本和内存成本不确定。应作为后续实验，不宜直接替换当前路径。

下一步建议：

- 先新增“chunk-local 二次过滤可节省量”的 dry-run 统计，例如估算每个 chunk 内按子块过滤后 candidate 数变化，不改变 `MarkTriangleToVoxelSpace()` 行为。
- 继续拆分 ClearanceBand 重复写入来源：相同 triangle 重复覆盖、相邻 triangle 覆盖、相邻 chunk 影响。
- benchmark 后续再补 summary row 或 JSON；当前 `runIndex/runCount` 已能支持外部聚合。

## 2026-04-30 chunk-local 二次过滤 dry-run

本轮继续不改变 marking 行为，只增加 dry-run 统计，用于评估“在 chunk 内进一步过滤 candidate triangle”是否值得推进。

dry-run 口径：

- `lazyDryRunActiveCandidateTriangleCount`：filtered candidate 中，triangle influence index range 与当前 chunk write bounds 有交集的数量。
- `lazyDryRunInactiveCandidateTriangleCount`：filtered candidate 中，与当前 chunk write bounds 无 voxel index 交集的数量。
- `lazyDryRunCandidateVoxelPairUpperBound`：如果每个 filtered candidate 都扫描整个 chunk，会产生的 triangle-voxel pair 上界。
- `lazyDryRunClippedVoxelPairCount`：按 triangle influence range 与 chunk bounds 交集裁剪后的 pair 数；该值应等于当前 `distanceCalculationCount`。

三轮 benchmark lazy 均值：

```text
sphere_pole_to_pole / lazy:
  candidateTriangleCount=11378
  activeCandidateTriangleCount=11378
  inactiveCandidateTriangleCount=0
  candidateVoxelPairUpperBound=53888266
  clippedVoxelPairCount=5141175
  clipped/upper≈9.5%

box_long_face_to_face / lazy:
  candidateTriangleCount=238
  activeCandidateTriangleCount=238
  inactiveCandidateTriangleCount=0
  candidateVoxelPairUpperBound=1109618
  clippedVoxelPairCount=135204
  clipped/upper≈12.2%
```

结论：

- 当前 filtered candidate 基本都能触达当前 chunk 的至少一个 voxel，单纯“再过滤掉整 triangle”的空间很小。
- 现有 influence range 与 chunk bounds 裁剪已经把 pair 数降到 chunk-wide 上界的约 10% 左右，这是当前已实现优化的主要收益。
- 剩余瓶颈不在“整 triangle 是否参与 chunk”，而在 active triangle 的 clipped range 内仍有大量距离计算未改善和 ClearanceBand 重复覆盖。

后续更优方向：

- 不建议立即实现 chunk-local 整 triangle 二次过滤，因为 dry-run 显示 inactive candidate 为 0。
- 若继续优化，应评估更细粒度的 block/voxel 级候选组织，但要先统计构建成本和潜在 pair 数变化，避免引入比距离计算更贵的局部索引。
- 更务实的下一步是拆分 `ClearanceBand -> ClearanceBand` 重复写来源，并评估是否存在安全的“状态不变但距离也未改善”的跳过策略；该策略仍需先 dry-run 统计，不应直接改变行为。
