# GeometryQueryPathPlanner

This module is the long-term path-planning direction and must remain independent
from `VoxelPathPlanner`.

`VoxelPathPlanner` is the current voxel baseline. This module is reserved for a
geometry-query-driven planner that can combine:

- mesh spatial indexing, such as BVH or AABB trees;
- on-demand closest-point, clearance, and collision queries;
- local caching around the active search frontier;
- continuous path smoothing and constraint checks after graph search.

Rules for this module:

- Do not include headers from `VoxelPathPlanner`.
- Do not change baseline voxel behavior from this module.
- Add adapters explicitly if data needs to move between modules.
- Add planner-level tests before connecting this module to `main.cpp` or
  benchmark executables.

Current state:

- Source module scaffold exists.
- Public API boundary exists.
- No planner algorithm is implemented yet.
- It is not connected to `main.cpp`, benchmark executables, or voxel tests.

Related documentation:

- `doc/GeometryQueryPathPlanner/Roadmap.md`
- `doc/ChangeLog.md` for project-wide changes.
