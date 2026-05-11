# MovementPath

MovementPath is a C++17 / Open CASCADE based path planning prototype.

The project triangulates an OCCT `TopoDS_Shape`, builds a voxel search space
around the mesh, searches a path in the clearance band with A*, optionally
optimizes the path, and exports VTK files for inspection.

## Current Focus

- Keep `FullMeshBounds` as the correctness baseline.
- Support experimental `LazyChunks` build mode.
- Measure performance with stable benchmark scenarios before enabling lazy by
  default.
- Keep the voxel implementation as the baseline module.
- Develop the long-term geometry-query planner in an isolated module.

## Documentation

The `doc/` directory separates module documentation:

- `doc/VoxelPathPlanningBaseline/` documents the current voxel baseline.
- `doc/GeometryQueryPathPlanner/` documents the long-term geometry-query module.
- `doc/ChangeLog.md` records important code changes across the whole project.

## Source Layout

- `src/Algorithms/VoxelPathPlanner/`: voxel baseline planner library.
- `src/Algorithms/GeometryQueryPathPlanner/`: independent geometry-query planner module.
- `src/IO/VtkExport/`: VTK mesh, voxel, path, and lazy chunk exporters.
- `src/Apps/MovementPathCli/`: sample command-line executable.
- `src/Apps/PathPlanningWorkbench/`: optional Qt + OCCT interactive workbench.
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
doc/VoxelPathPlanningBaseline/voxel_planner_benchmark.csv
```

## Notes

- `LazyChunks` remains disabled by default.
- Some source files currently trigger MSVC C4819 code-page warnings because of
  Chinese comments. These warnings do not block the current build.
