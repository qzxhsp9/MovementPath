# Path Planning Methods

This document summarizes the planning methods currently exposed by
MovementPath. Code names and UI labels stay in English so they match source,
logs, and benchmark output.

## Overview

| Method | Workbench | Status | Core idea | Main use | Main limitation |
|---|---:|---|---|---|---|
| `Voxel full bounds` | Yes | Correctness baseline | Voxelize the full mesh bounds, then run voxel A*. | Path-quality checks, regression tests, and benchmark baseline. | Highest voxel-build and memory cost. |
| `Voxel lazy` | Yes | Experimental acceleration path | Start from a sparse voxel space and classify voxel states on demand with cached triangle/voxel pair results. | Large models where the searched area is small. | Depends on cache locality and the timeout budget; timeout is reported instead of hidden fallback. |
| `Geometry query planner` | Visible in UI, not connected to calculation | Query primitives and scaffolding exist | Use mesh spatial indexes, closest-point queries, and segment clearance instead of full pre-voxelization. | Long-term replacement direction. | `Plan()` currently builds query context and returns `NotImplemented`. |

## Voxel Flow

| Step | Full bounds | Voxel lazy |
|---|---|---|
| Import and triangulate | Same shape or triangle input path. | Same shape or triangle input path. |
| Build triangle spatial index | Used for profiling and lazy support. | Used for per-voxel candidate triangle lookup. |
| Build voxel space | Build the full mesh bounds before search. | Initialize sparse bounds; classify voxel states only when A* or optimization asks for a voxel. |
| Snap endpoints | Supported. | Supported. |
| A* search | Searches the full built voxel space. | Search triggers per-voxel state queries as needed. |
| Connectivity fallback | Supported for clearance-band failures. | Supported before lazy fallback. |
| Restore original endpoints | Output path preserves API start and goal points. | Same. |
| Polyline optimization | Collinear removal and line-of-sight shortcut. | Same. |
| Optional smoothing | Enabled only when `smoothOptimizedPath` is set. | Same. |
| Visualization/export | Voxel/path VTK output. | Voxel/path VTK output. |

## Recommended Use

| Scenario | Recommended configuration |
|---|---|
| Correctness or path-quality validation | Use `Voxel full bounds` with `ClearanceBand`. |
| Large-model performance experiments | Use `Voxel lazy` with full-bounds fallback. |
| Endpoint snapping debugging | Show key, occupied, and clearance voxels. |
| Paths too close to obstacles | Inspect occupied/clearance voxels and reduce voxel size if needed. |
| Smoother display path | Enable `Smooth path` and check whether smoothing was accepted or rejected in logs. |

## Related Documents

- [路径优化与平滑流程](PathOptimizationAndSmoothingFlow.md): current post-A* optimization, smoothing, and Workbench display flow.
