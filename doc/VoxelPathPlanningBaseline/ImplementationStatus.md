# MovementPath 当前实现状态

本文档描述体素 baseline 当前已经落地的能力、源码结构和运行产物。进度计划见 `Roadmap.md`，性能优化细节见 `OptimizationPlan.md`，全项目逐次变更记录见 `../ChangeLog.md`。

## 项目目标

MovementPath 当前核心目标是：基于 OCCT Shape 三角化结果构建体素空间，在障碍物表面外的安全距离层中搜索从起点到终点的路径，并导出可视化 VTK 结果用于检查路径质量和体素覆盖范围。

当前实现同时保留三种构建模式：

- `FullMeshBounds`：完整 mesh bounds 体素化，作为正确性和路径质量基线。
- `StartGoalBox`：基于起终点 AABB 加 padding 的局部预体素化，适合短路径实验，但可能裁剪有效绕行区域。
- `LazyChunks`：A* 搜索过程中按需构建 chunk，当前仍默认关闭，用于性能优化实验。

## 源码结构

- `src/main.cpp`：示例入口，负责组织场景、调用 planner、输出 profile 和 VTK。
- `src/VoxelPathPlanner/`：体素 baseline 库目录，主流程应尽量沉淀到这里，测试和示例只调用接口。
- `src/VoxelPathPlanner/VoxelPathPlanner.*`：规划总入口，负责三角化、空间索引、体素构建、A*、优化、fallback、profile 和 VTK 导出协调。
- `src/VoxelPathPlanner/VoxelMeshBuilder.*`：Shape/triangle 到 `VoxelSpace` 的体素化逻辑，包含 full/local/append 构建、候选三角形过滤和标记统计。
- `src/VoxelPathPlanner/VoxelChunkCache.*`：lazy chunk 按需构建缓存，负责 chunk index、构建去重、triangle influence range cache 和 chunk 统计。
- `src/VoxelPathPlanner/VoxelAStar.*`：体素 A* 搜索，支持起终点吸附、lazy ensure hook、不同邻接方式和路径标记。
- `src/VoxelPathPlanner/VoxelPathOptimizer.*`：路径后处理，包括共线点删除和 line-of-sight shortcut。
- `src/VoxelPathPlanner/TriangleSpatialHash.*`：三角形空间哈希，用于减少 local/lazy 候选三角形查询成本。
- `src/VoxelPathPlanner/VoxelVtkExporter.*` / `MeshVtkExporter.*`：体素、路径、chunk bounds 和 mesh 的 VTK 导出。
- `tests/`：单元/集成测试，覆盖空间哈希、chunk cache、planner 行为、lazy fallback 和导出。
- `benchmarks/VoxelPlannerBenchmark.cpp`：稳定 benchmark 场景，输出 full/local/lazy 对比 CSV。
- `doc/`：项目文档和 benchmark CSV 输出目录。

## 当前关键行为

- 默认选项仍保持 `lazyBuildOptions.enabled=false`，避免在数据不足时把实验路径作为默认路径。
- `maxCostRegressionRatio` 默认不设置推荐值，只有调用方显式配置时才启用 lazy 质量回退。
- A* 的 `voxelPath` 表示内部 snapped voxel 序列，用于搜索、可通行性和 voxel 可视化。
- `pointPath` 是对外显示路径；planner 会把首尾替换为接口入参 start/goal 点，避免 VTK polyline 显示为 snapped voxel center。
- `optimized_path_voxels.vtk` 显示体素搜索结果，Start/Goal 仍是 snapped voxel。
- `optimized_path_polyline.vtk` 显示接口入参点到入参点的折线路径。
- `lazy_chunk_bounds.vtk` 表示 A* 实际触发构建的 chunk 集合，不代表完整 mesh 包壳，也不要求封闭。

## Benchmark 输出

Benchmark 默认输出：

```text
doc/VoxelPathPlanningBaseline/voxel_planner_benchmark.csv
```

CSV 记录 full/local/lazy 在同一场景下的耗时、候选三角形、chunk 数、路径 cost、lazy 标记统计等字段。当前稳定场景：

- `sphere_pole_to_pole`：球体南北极路径，用于观察 long detour 和 StartGoalBox 裁剪风险。
- `box_long_face_to_face`：非球体稳定场景，避免优化只围绕球体调参。

当前 lazy 诊断字段包括距离计算、距离改善/未改善、状态写入/未变化写入、Occupied/ClearanceBand 未变化写入拆分、chunk candidate triangle min/max 分布、chunk-local dry-run pair 估算和 influence cache hit/miss。benchmark 支持 `--runs N` 多轮输出，CSV 使用 `runIndex` / `runCount` 标识轮次。

## 当前验证基线

常用验证命令：

```powershell
cmd.exe /c "call ""D:\software\ide\vs\Microsoft Visual Studio2022\Community\VC\Auxiliary\Build\vcvars64.bat"" && cmake --build out\build\x64-Debug --config Debug --target VoxelPathPlannerTests MovementPathBenchmark"
ctest --test-dir out\build\x64-Debug --output-on-failure
out\build\x64-Debug\MovementPathBenchmark.exe
```

当前测试覆盖：

- `TriangleSpatialHashTests`
- `VoxelChunkCacheTests`
- `VoxelPathPlannerTests`

## 已知限制

- `StartGoalBox` 可能裁剪起终点 AABB 之外的更优路径，不能作为无 fallback 的质量基线。
- lazy chunk 当前减少了全局预体素化范围，但仍可能在有效 chunk 内重复执行大量点到三角形距离计算。
- benchmark 仍是单次运行，耗时受本机状态影响；后续需要多轮统计或 JSON 输出以支持趋势分析。
- 部分中文源码注释在当前 Windows 代码页下会触发 C4819 编译警告；警告不影响构建，但后续应统一文件编码。

## 2026-04-30 新模块定位

当前体素方案正式定位为 baseline：继续用于 correctness、路径质量对照、VTK 可视化和 benchmark 对比，但不再作为长期性能与路径质量的唯一主线。

新增 `src/GeometryQueryPathPlanner/` 作为长期方向的新模块：

- `GeometryQueryPathPlanner` 是独立 CMake library，当前不链接到 `main.cpp`、benchmark 或既有测试。
- 新模块不包含 `VoxelPathPlanner` 头文件，避免 baseline 体素实现向新方案泄漏。
- 初始 API 只定义独立几何 primitive、planner request/options/result/profile 和 `GeometryQueryPathPlanner` 边界。
- 后续应在新模块内推进 BVH/AABB tree、按需 closest-point/clearance 查询、局部缓存和连续路径优化。
