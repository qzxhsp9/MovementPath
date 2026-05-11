# MovementPath 修改日志

本文档记录 MovementPath 项目的所有重要代码修改。模块文档分别放在 `doc/VoxelPathPlanningBaseline/` 和 `doc/GeometryQueryPathPlanner/`。

## 2026-05-11

### Voxel baseline cleanup and source layout

- Removed the start-goal local box build mode from the public voxel planner API.
- Kept `FullMeshBounds` as the only eager voxel build path and the fallback target for lazy planning.
- Reduced benchmark modes to `full` and `lazy`.
- Moved algorithm code under `src/Algorithms/`.
- Moved VTK export code under `src/IO/VtkExport/` and built it as `MovementPathVtkExport`.
- Moved executable/workbench code under `src/Apps/`.
- Updated tests and current docs to match the smaller mode set and new layout.

## 2026-04-30

### PathPlanningWorkbench 交互平台骨架

- 新增可选 Qt + OCCT target `PathPlanningWorkbench`，Qt Widgets 不存在时自动跳过，不影响现有命令行 target 和测试。
- `PathPlanningWorkbench` CMake 支持自动探测 `C:/Qt/6.11.0/msvc2022_64`，也可通过 `PATH_PLANNING_WORKBENCH_QT_ROOT` 指定 Qt 安装根目录。
- 新增 `src/PathPlanningWorkbench/`，包含模型导入、OCCT 视图、交互面板和 workbench 入口。
- 支持导入 STEP/STP、BREP 和 legacy ASCII VTK polygon mesh；STEP/BREP 可通过 linear/angular deflection 调整离散参数。
- 支持 shaded/wireframe 显示、手动输入起终点与方向、双击拾取起终点与方向、主动计算路径和输入变更后实时计算路径。
- 改进视图交互：滚轮以鼠标位置为缩放中心；双击拾取改为对当前模型三角网格做射线命中，拾取点落在模型表面。
- 显示模式新增 mesh，基于 OCCT triangulation 叠加显示离散网格边；VTK 导入后也会补三角化用于拾取和网格显示。
- 离散参数改为显式 Apply Discretization 后生效，更新后同步影响显示网格和后续路径计算。
- 起终点方向取消拾取设置，默认使用 +X；快捷键 `Q` 循环起点方向，快捷键 `E` 循环终点方向，顺序为 +X、+Y、+Z、-X、-Y、-Z。
- 起终点方向刷新时合并 spinbox 信号并只重绘 endpoint overlay，减少方向图标闪烁。
- OCCT 视图新增世界坐标轴显示。
- 路径计算前增加 planner、voxel size、离散参数和开始/结束日志，并主动刷新 UI 日志区域。
- 路径计算改为后台任务执行，新增 Stop Computation 按钮；`VoxelPathPlanner`/`VoxelAStar` 增加协作式取消回调，停止后可调整参数并重新计算。
- mesh 显示模式只显示离散网格边，不再显示实体；网格边按模型缓存，避免 VTK 网格导入后切换显示模式反复重建导致卡顿。
- VTK 导入不再转换为 `TopoDS_Shape`；workbench 直接保留 `TriangleMeshData` 并用 `AIS_Triangulation` 显示和拾取。当前 voxel planner 对 VTK mesh 会提示需要 triangle-input adapter。
- 交互面板提供 voxel full-bounds、voxel lazy 和 geometry-query 规划方法入口；当前 geometry-query 在 UI 中保留入口但尚未接入 OCCT 导入。
- voxel 规划结果可在 OCCT 视图中叠加显示路径和关键路径体素。

### Voxel baseline BREP 场景入口

- `VoxelPathPlanner` 新增 `VoxelBrepScenarioRequest`、`ReadBrepShape()`、`MakeScenarioFromBrepFile()` 和 `MakeBrepBaselineOptions()`，沉淀通过 BREP 文件构造 baseline 测例的公共方式。
- `main.cpp` 的 `user_brep` 示例改为只填写 BREP 文件路径、起点、终点和端点 snap 方向，再调用 `VoxelPathPlanner` 模块 API。
- BREP baseline options 默认使用 `FullMeshBounds`，关闭 lazy build，避免调试外部 BREP 文件时混入 lazy fallback 行为。
- 补充缺失 BREP 文件的 helper 单测，验证失败原因和 baseline options。

### GeometryQueryPathPlanner Phase 1 边界测试

- 扩展 `GeometryQueryPathPlannerTests`，覆盖 segment-triangle 共面穿越、端点接触、边重叠、近平行、退化线段和退化三角形。
- 修正并行/近并行 segment-triangle 相交判定中的点在三角形上容差，避免近平行但未接触的线段被误判为碰撞。
- 保持 segment clearance AABB tree 当前保守遍历行为不变，继续与暴力 oracle 对齐。

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
- 新增 `GeometrySearchGraph.h`，定义按需搜索图第一版节点、边、边 clearance 结果和 graph profile。
- `GeometryQueryPathPlanner::Plan()` 增加 start/goal 有限性检查、clearance 合法性检查和失败原因。
- 补充搜索图接口与 planner invalid input 单测。
- 在 `GeometryQueryPathPlanner` Roadmap 中补充第一版 geometry benchmark 输出字段设计，用于后续与 voxel baseline 对齐。
- `TriangleAabbTree` 新增 `EstimateSegmentClearancePruning()`，独立模拟 segment clearance 剪枝收益，不改变正式查询行为。
- 剪枝 dry-run 统计新增估算访问节点数、估算测试三角形数、估算跳过三角形数、best-distance 剪枝数和 clearance-safe 剪枝数。
- `GeometryPathProfile` 同步新增上述剪枝估算字段，便于后续 benchmark 输出。
