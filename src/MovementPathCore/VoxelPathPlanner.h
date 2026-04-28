#pragma once

#include "VoxelAStar.h"
#include "VoxelMeshBuilder.h"

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cstddef>
#include <string>

struct VoxelPlanningProfile
{
    double shapeMeshExportMs = 0.0;
    double triangulationMs = 0.0;
    double spatialIndexBuildMs = 0.0;
    double voxelBuildMs = 0.0;
    double astarMs = 0.0;
    double optimizeMs = 0.0;
    double vtkExportMs = 0.0;

    std::size_t triangleCount = 0;
    std::size_t candidateTriangleCount = 0;
    std::size_t rawCandidateTriangleCount = 0;
    std::size_t hashCellCount = 0;
    std::size_t hashEntryCount = 0;
    std::size_t hashQueryCellCount = 0;
    std::size_t hashRawTriangleCount = 0;
    std::size_t storedCellCount = 0;
    std::size_t occupiedCount = 0;
    std::size_t clearanceBandCount = 0;

    int astarVisitedCount = 0;
    std::size_t rawPathCount = 0;
    std::size_t optimizedPathCount = 0;
    int lineCheckCount = 0;

    double totalCost = 0.0;

    std::string buildRegionMode;
    std::string scenarioName;
    int buildAttemptCount = 0;
    double finalSearchPadding = 0.0;
    bool buildSucceeded = false;
    bool astarSucceeded = false;
};

struct VoxelPlanningScenario
{
    std::string name;
    TopoDS_Shape shape;
    gp_Pnt startPoint;
    gp_Vec startDir;
    gp_Pnt goalPoint;
    gp_Vec goalDir;
};

struct VoxelPlanningRunOptions
{
    bool exportVtk = true;
    bool debugNeighborhood = true;
    bool verbose = true;

    std::string shapeMeshVtkPath = "D:/shape_mesh.vtk";
    std::string astarFailedVtkPath = "D:/astar_failed.vtk";
    std::string astarPathVtkPath = "D:/astar_path.vtk";
    std::string optimizedPathVoxelsVtkPath = "D:/optimized_path_voxels.vtk";
    std::string optimizedPathPolylineVtkPath = "D:/optimized_path_polyline.vtk";
};

enum class VoxelBuildRegionMode
{
    // Correctness baseline. Builds voxels for the full mesh bounds and should
    // remain the default unless a caller has its own quality fallback.
    FullMeshBounds,

    // Performance-oriented local build. The build box is derived from the
    // start-goal AABB plus padding, so it can clip valid detours outside that
    // box. Use only for short local paths or experiments with a fallback to
    // FullMeshBounds when path quality matters.
    StartGoalBox
};

struct VoxelLocalBuildOptions
{
    // VoxelPathPlanner::MakeDefaultOptions() uses FullMeshBounds. This field
    // defaults to StartGoalBox only for direct aggregate construction; callers
    // should prefer MakeDefaultOptions() for production-safe defaults.
    VoxelBuildRegionMode regionMode = VoxelBuildRegionMode::StartGoalBox;

    // Padding around the start-goal box in world units. Larger values reduce
    // clipping risk but move the mode closer to full-bounds cost.
    double searchPadding = 20.0;

    // Retries expand the same start-goal box. They help when the local region
    // is slightly too small, but do not guarantee preserving the global best
    // path for long detours.
    int maxRetryCount = 3;
    double retryExpandFactor = 2.0;
};

struct VoxelPathPlannerOptions
{
    VoxelMeshBuildOptions meshBuildOptions;
    VoxelLocalBuildOptions localBuildOptions;
    VoxelAStarOptions astarOptions;
    VoxelPlanningRunOptions runOptions;

    int searchBoundsExtraRadius = 10;
    int debugNeighborhoodRadius = 10;
    int optimizerMaxShortcutLookAhead = 200;
};

struct VoxelPathPlannerResult
{
    bool success = false;
    VoxelPlanningProfile profile;
    VoxelAStarResult astarResult;
};

class VoxelPathPlanner
{
public:
    static VoxelPathPlannerOptions MakeDefaultOptions();

    static VoxelPathPlannerResult Plan(
        const VoxelPlanningScenario& scenario,
        const VoxelPathPlannerOptions& options);

    static void PrintProfile(
        const VoxelPlanningProfile& profile);

    static double SafeRatio(
        std::size_t numerator,
        std::size_t denominator);
};
