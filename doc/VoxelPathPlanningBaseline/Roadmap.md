# Voxel Path Planning Roadmap

This roadmap tracks the voxel baseline. Implementation details live in
`ImplementationStatus.md`; performance work lives in `OptimizationPlan.md`.

## Current Position

The voxel planner is the correctness baseline. It now keeps one eager build
mode, `FullMeshBounds`, plus the experimental `LazyChunks` path. The former is
the default and the fallback target; the latter remains opt-in and
benchmark-driven.

The source tree is organized by role:

- `src/Algorithms/VoxelPathPlanner/`: voxel algorithm library.
- `src/IO/VtkExport/`: VTK exporters.
- `src/Apps/MovementPathCli/`: command-line example.
- `src/Apps/PathPlanningWorkbench/`: optional Qt + OCCT workbench.

## Completed

- Extracted `VoxelPathPlanner` from example code into a library target.
- Kept `FullMeshBounds` as the default correctness and path-quality baseline.
- Added lazy chunk options, fallback policy, guardrails, and profile fields.
- Added `VoxelChunkCache`, lazy planner tests, and lazy VTK exports.
- Added benchmark CSV output for full and lazy planner runs.
- Added detailed lazy statistics for distance calculations, state writes,
  candidate distributions, and dry-run voxel-pair estimates.
- Removed the local start-goal box build mode to keep the baseline API smaller
  and avoid a known path-clipping mode.

## Next

- Continue analyzing repeated `ClearanceBand -> ClearanceBand` writes.
- Add dry-run statistics before changing any voxel marking behavior.
- Evaluate whether unchanged-state and non-improving-distance writes can be
  skipped safely.
- Keep `LazyChunks` disabled by default until benchmark data supports changing
  that default.
- Consider JSON or summary-row benchmark output once the CSV schema stabilizes.
