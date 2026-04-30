# GeometryQueryPathPlanner Roadmap

This document tracks the long-term geometry-query-driven planner. The voxel
baseline roadmap is maintained separately in
`doc/VoxelPathPlanningBaseline/Roadmap.md`.

## Current Position

`GeometryQueryPathPlanner` is the long-term performance and path-quality module.
It is intentionally independent from `VoxelPathPlanner`.

## Phase 0: Module Isolation

Status: started.

- Add independent library target `GeometryQueryPathPlanner`.
- Define independent geometry primitives and planner API boundaries.
- Keep the module disconnected from `main.cpp`, benchmark executables, and
  voxel tests until planner-level tests exist.

## Phase 1: Geometry Query Foundation

Status: not started.

- Design a mesh BVH, AABB tree, or equivalent closest-triangle query structure.
- Add point/segment to mesh clearance query interfaces.
- Add unit tests for distance queries, collision queries, and boundary cases.

## Phase 2: On-Demand Search And Local Cache

Status: not started.

- Design search nodes and edge feasibility checks that do not require dense
  pre-voxelization.
- Cache geometry query results only around the active search frontier.
- Compare cost and path quality against the voxel baseline.

## Phase 3: Continuous Path Quality

Status: not started.

- Apply continuous-space shortcut and smoothing after graph search.
- Use mesh clearance queries to protect safety distance and path quality.
- Emit profile and visualization outputs comparable with the baseline.
