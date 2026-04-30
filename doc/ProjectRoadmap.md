# MovementPath 进度计划

本文档维护项目从当前状态到最终目标的阶段计划。当前实现状态见 `ProjectImplementationStatus.md`，性能专项计划见 `VoxelPathPlanningOptimizationPlan.md`，逐次修改记录见 `ChangeLog.md`。

## 最终目标

- 提供稳定的 `MovementPathCore` 接口，调用方只需传入 Shape、起终点和规划选项即可得到路径、profile 和可选 VTK。
- 保持 `FullMeshBounds` 作为正确性基线，保证测试覆盖路径质量、fallback 和导出。
- 在不牺牲路径质量的前提下，让 local/lazy 构建成为可按场景启用的性能优化路径。
- benchmark 输出可复现、字段稳定、能支撑后续自动对比。

## 当前阶段

当前处于 `Phase 3: LazyChunks 性能诊断与保守优化`。

已完成：

- `MovementPathCore` 已从示例入口中拆出。
- full/local/lazy 三种构建模式可通过 planner 统一入口调用。
- lazy chunk cache 已接入 planner，并具备失败 fallback 和统计字段。
- benchmark 已覆盖球体和非球体场景，CSV 输出到 `doc/voxel_planner_benchmark.csv`。
- VTK 导出已覆盖 mesh、A* path、optimized path、lazy chunk bounds。
- planner 的 `pointPath` 显示端点已对齐接口入参点。

## 阶段计划

### Phase 0：基线整理

状态：完成。

- 梳理体素路径规划主流程。
- 建立 `VoxelPathPlanning` 设计文档。
- 明确 `StartGoalBox` 局限性。
- 增加 smoke tests。

### Phase 1：局部范围预体素化

状态：完成。

- 支持 `FullMeshBounds` 与 `StartGoalBox`。
- 增加 retry/fallback 思路。
- 发现并记录 `StartGoalBox` 对 `sphere_pole_to_pole` 的路径裁剪风险。

### Phase 2：MeshTriangle 空间索引

状态：完成。

- 增加 `TriangleSpatialHash`。
- local/lazy 构建通过 query box 获取候选三角形。
- 增加空间哈希单测和候选统计。

### Phase 3：LazyChunks

状态：进行中。

已完成：

- lazy 选项、统计字段、fallback 策略。
- `VoxelChunkCache` 接入 planner。
- chunk cache 单测和 planner 级 lazy 成功测试。
- lazy benchmark 输出 full/local/lazy 对比。
- 去除 append 后全量状态统计重复扫描。
- 拆分 chunk 查询范围和写入范围。
- 缓存 triangle influence range。
- mark loop 按写入 bounds 裁剪，消除 lazy out-of-bounds voxel visit。
- lazy VTK 导出补齐路径结果。
- lazy 统计增强已记录距离未改善、状态未变化写入和 chunk candidate 分布。

下一步：

- 基于新增统计评估 chunk-local 二次过滤。
- 继续细分状态未变化写入来源。
- 评估更适合 lazy 的 triangle-to-voxel 或 voxel-to-triangle 查询方式。
- benchmark 多轮运行与输出格式增强。

### Phase 4：质量保护与产品化接口

状态：未开始。

- 明确 lazy/local 的默认启用条件。
- 提供可配置质量 guardrail，例如 cost regression、最大 chunk 数、最大访问数。
- 统一导出路径、profile 输出和错误原因。
- 收敛 public API，减少测试对内部细节的依赖。

### Phase 5：性能定型

状态：未开始。

- 基于稳定 benchmark 决定是否默认启用某些优化。
- 完成多场景数据对比。
- 清理临时诊断字段或将其转为正式 profile 字段。
- 评估并行化、缓存粒度和内存占用。

## 当前优先级

1. 保持 correctness baseline：full build 和现有 planner tests 必须稳定通过。
2. lazy 继续默认关闭，直到 benchmark 数据证明默认开启是合理的。
3. 新优化必须先增加统计或测试，避免为单一场景硬编码。
4. `main.cpp` 只保留示例逻辑，核心行为继续下沉到 `MovementPathCore`。
5. 文档与代码同步更新：状态写入本文档，性能实验写入 `VoxelPathPlanningOptimizationPlan.md`，逐次修改写入 `ChangeLog.md`。
