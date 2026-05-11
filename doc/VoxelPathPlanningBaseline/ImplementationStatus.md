# MovementPath Current Implementation Status

MovementPath currently uses the voxel planner as its baseline implementation.
The planner triangulates an OCCT shape or consumes caller-provided triangles,
builds a voxel search space, searches the clearance band with A*, optimizes the
path, and can export VTK artifacts for inspection.

## Build Modes

- `FullMeshBounds`: eager full mesh-bounds voxelization. This is the default,
  correctness baseline, benchmark baseline, and lazy fallback target.
- `LazyChunks`: experimental on-demand chunk voxelization during A* search.
  It remains disabled by default and should be enabled only by callers that
  accept its guardrails and fallback behavior.

The previous start-goal local box build mode has been removed from the public
planner API and benchmark matrix.

## Source Layout

- `src/Algorithms/VoxelPathPlanner/`: algorithm library target
  `VoxelPathPlanner`.
- `src/Algorithms/GeometryQueryPathPlanner/`: independent long-term
  geometry-query planner target.
- `src/IO/VtkExport/`: VTK mesh, voxel, path, and lazy chunk exporters.
- `src/Apps/MovementPathCli/`: sample command-line executable.
- `src/Apps/PathPlanningWorkbench/`: optional interactive Qt + OCCT workbench.
- `tests/`: unit and planner integration tests.
- `benchmarks/VoxelPlannerBenchmark.cpp`: benchmark executable.

## Key Behaviors

- `lazyBuildOptions.enabled` defaults to `false`.
- `maxCostRegressionRatio` defaults to `0.0`, meaning disabled unless the
  caller explicitly enables the quality guardrail.
- `voxelPath` stores snapped voxel indices for search and validation.
- `pointPath` preserves the API start and goal points for external display.
- `lazy_chunk_bounds.vtk` shows chunks actually built by lazy search; it is not
  a closed mesh bounds representation.

## Known Limits

- Lazy chunking can still perform many point-to-triangle distance calculations
  inside active chunks.
- Current data shows repeated `ClearanceBand -> ClearanceBand` writes and
  non-improving distance calculations are the main lazy bottlenecks.
- Benchmark output is CSV-first; summary rows or JSON are still future work.
