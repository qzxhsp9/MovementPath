# VoxelPathPlanner

This module is the current voxel path-planning baseline.

It keeps the existing full/local/lazy voxel implementations, A* search,
path optimization, VTK export, benchmark support, and planner-facing API.

Rules for this module:

- Preserve `FullMeshBounds` as the correctness and path-quality baseline.
- Keep lazy/local optimizations conservative and benchmark-driven.
- Do not depend on `GeometryQueryPathPlanner`.
- Keep example and test code calling public planner interfaces.

Related documentation:

- `doc/VoxelPathPlanningBaseline/ImplementationStatus.md`
- `doc/VoxelPathPlanningBaseline/Roadmap.md`
- `doc/VoxelPathPlanningBaseline/OptimizationPlan.md`
- `doc/VoxelPathPlanningBaseline/voxel_planner_benchmark.csv`
- `doc/ChangeLog.md` for project-wide changes.
