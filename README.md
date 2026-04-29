# MovementPath

MovementPath is a C++17 / Open CASCADE based voxel path planning prototype.

The project triangulates an OCCT `TopoDS_Shape`, builds a voxel search space
around the mesh, searches a path in the clearance band with A*, optionally
optimizes the path, and exports VTK files for inspection.

## Current Focus

- Keep `FullMeshBounds` as the correctness baseline.
- Support experimental `StartGoalBox` and `LazyChunks` build modes.
- Measure performance with stable benchmark scenarios before enabling lazy by
  default.
- Keep planner logic inside `src/MovementPathCore`; examples and tests should
  call public interfaces instead of duplicating workflow code.

## Documentation

The `doc/` directory is the source of truth for project status and planning:

- `doc/ProjectImplementationStatus.md` describes the current implementation.
- `doc/ProjectRoadmap.md` tracks current and final progress plans.
- `doc/VoxelPathPlanningOptimizationPlan.md` tracks performance optimization.
- `doc/ChangeLog.md` records important changes.
- `doc/voxel_planner_benchmark.csv` is the latest benchmark CSV output.

## Source Layout

- `src/main.cpp`: sample executable.
- `src/MovementPathCore/`: core planner library.
- `tests/`: unit and planner integration tests.
- `benchmarks/VoxelPlannerBenchmark.cpp`: benchmark executable.
- `doc/`: documentation and benchmark CSV output.

## Build And Test

On the current Windows/MSVC setup:

```powershell
cmd.exe /c "call ""D:\software\ide\vs\Microsoft Visual Studio2022\Community\VC\Auxiliary\Build\vcvars64.bat"" && cmake --build out\build\x64-Debug --config Debug"
ctest --test-dir out\build\x64-Debug --output-on-failure
out\build\x64-Debug\MovementPathBenchmark.exe
```

The benchmark writes:

```text
doc/voxel_planner_benchmark.csv
```

## Notes

- `LazyChunks` remains disabled by default.
- `StartGoalBox` can clip valid detours and should not be used as a quality
  baseline without fallback.
- Some source files currently trigger MSVC C4819 code-page warnings because of
  Chinese comments. These warnings do not block the current build.
