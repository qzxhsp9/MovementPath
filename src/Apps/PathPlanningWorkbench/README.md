# PathPlanningWorkbench

Qt + OCCT interactive workbench for building path-planning test cases.

Initial scope:

- import STEP/STP, BREP, and legacy ASCII VTK polygon meshes;
- remesh OCCT shape inputs with adjustable linear and angular deflection;
- switch between shaded and wireframe display;
- enter start/goal points and snap directions manually;
- run voxel full-bounds, voxel lazy, or geometry-query planner entries;
- display computed paths and key path voxels in the OCCT view.

The target is optional. CMake builds `PathPlanningWorkbench` only when Qt
Widgets is available; command-line planner builds and tests do not depend on Qt.

Qt discovery:

- Default auto-detect uses `C:/Qt/6.11.0/msvc2022_64` when that kit exists.
- For another kit path without CMake flags: copy `QtWorkbenchDefaults.cmake.example`
  to `QtWorkbenchDefaults.cmake` (same folder; the copy is gitignored) and set
  `MOVEMENTPATH_QT_WORKBENCH_DEFAULT_KIT` there — that is the only path to edit locally.
- Alternatively configure with `-DPATH_PLANNING_WORKBENCH_QT_ROOT=<kit root>` or
  set `CMAKE_PREFIX_PATH`.
