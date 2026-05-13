#pragma once

#include "VoxelAStar.h"
#include "VoxelMeshBuilder.h"
#include "VoxelPathOptimizer.h"

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

struct VoxelPlanningProfile
{
    double shapeMeshExportMs = 0.0;
    double triangulationMs = 0.0;
    double spatialIndexBuildMs = 0.0;
    double voxelBuildMs = 0.0;

    // Lazy query time is included in astarMs because voxel states are resolved
    // from A* ensureCellBuilt callbacks. The fields below split that cost.
    double lazyVoxelQueryMs = 0.0;
    double lazyCandidateQueryMs = 0.0;
    double lazyCandidateFilterMs = 0.0;
    double lazyVoxelMarkMs = 0.0;
    double lazyStateCountMs = 0.0;
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

    // Number of A* lazy hook invocations, including cache hits.
    std::size_t lazyEnsureCallCount = 0;
    std::size_t lazyCacheHitCount = 0;
    std::size_t lazyFailedBuildCount = 0;
    std::size_t lazyVoxelQueryCount = 0;
    std::size_t lazyTriangleVoxelIntersectTestCount = 0;
    std::size_t lazyTriangleVoxelNoIntersectCacheHitCount = 0;
    std::size_t lazyCandidateTriangleCount = 0;
    std::size_t lazyRawCandidateTriangleCount = 0;
    std::size_t lazyVoxelVisitCount = 0;
    std::size_t lazyOutOfBoundsVoxelCount = 0;
    std::size_t lazyDistanceCalculationCount = 0;
    std::size_t lazyDistanceImprovedCount = 0;
    std::size_t lazyDistanceNotImprovedCount = 0;
    std::size_t lazyStateWriteCount = 0;
    std::size_t lazyStateUnchangedWriteCount = 0;
    std::size_t lazyOccupiedUnchangedWriteCount = 0;
    std::size_t lazyClearanceUnchangedWriteCount = 0;
    std::size_t lazyOccupiedWriteCount = 0;
    std::size_t lazyClearanceWriteCount = 0;
    std::size_t lazyInfluenceCacheHitCount = 0;
    std::size_t lazyInfluenceCacheMissCount = 0;
    std::size_t lazyMinCandidateTriangleCount = 0;
    std::size_t lazyMaxCandidateTriangleCount = 0;
    std::size_t lazyMinRawCandidateTriangleCount = 0;
    std::size_t lazyMaxRawCandidateTriangleCount = 0;
    std::size_t lazyDryRunActiveCandidateTriangleCount = 0;
    std::size_t lazyDryRunInactiveCandidateTriangleCount = 0;
    std::size_t lazyDryRunCandidateVoxelPairUpperBound = 0;
    std::size_t lazyDryRunClippedVoxelPairCount = 0;
    std::size_t storedCellCount = 0;
    std::size_t occupiedCount = 0;
    std::size_t clearanceBandCount = 0;

    int astarVisitedCount = 0;
    std::size_t rawPathCount = 0;
    std::size_t optimizedPathCount = 0;
    std::size_t smoothedPathPointCount = 0;
    int lineCheckCount = 0;
    int smoothingLineCheckCount = 0;
    bool smoothingRequested = false;
    bool smoothingSucceeded = false;

    double totalCost = 0.0;

    double rawPathLength = 0.0;
    double rawPathDetourRatio = 0.0;
    std::size_t rawPathTurnCount = 0;
    double rawPathTotalTurnSeverity = 0.0;
    double rawPathMaxTurnSeverity = 0.0;
    double rawStartDirectionAlignment = 0.0;
    double rawGoalDirectionAlignment = 0.0;

    double finalPathLength = 0.0;
    double finalPathDetourRatio = 0.0;
    std::size_t finalPathTurnCount = 0;
    double finalPathTotalTurnSeverity = 0.0;
    double finalPathMaxTurnSeverity = 0.0;
    double finalStartDirectionAlignment = 0.0;
    double finalGoalDirectionAlignment = 0.0;

    std::string buildRegionMode;
    std::string scenarioName;
    int buildAttemptCount = 0;
    double finalSearchPadding = 0.0;
    bool buildSucceeded = false;
    bool astarSucceeded = false;
    bool lazyBuildEnabled = false;
    bool lazyFallbackTriggered = false;
    bool lazyTimeoutTriggered = false;
    double lazyTimeBudgetMs = 0.0;
    std::string lazyFallbackReason;
};

struct VoxelPlanningScenario
{
    std::string name;
    TopoDS_Shape shape;
    std::vector<MeshTriangle> triangles;
    gp_Pnt startPoint;
    gp_Vec startDir;
    gp_Pnt goalPoint;
    gp_Vec goalDir;
};

struct VoxelBrepScenarioRequest
{
    std::string name = "user_brep";
    std::string brepPath;
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
    bool collectVoxelOverlayCenters = false;
    // 0 means unlimited. Workbench normally filters overlays to the final
    // path bounds, so unlimited remains usable for local debugging.
    std::size_t maxVoxelOverlayCentersPerState = 0;

    std::string shapeMeshVtkPath = "D:/shape_mesh.vtk";
    std::string astarFailedVtkPath = "D:/astar_failed.vtk";
    std::string astarPathVtkPath = "D:/astar_path.vtk";
    std::string optimizedPathVoxelsVtkPath = "D:/optimized_path_voxels.vtk";
    std::string optimizedPathPolylineVtkPath = "D:/optimized_path_polyline.vtk";

    std::function<bool()> shouldCancel;
};

enum class VoxelLazyFallbackPolicy
{
    // Do not fallback automatically. Lazy failures are reported to the caller.
    None,

    // If lazy planning fails or violates guardrails, rerun with FullMeshBounds.
    FullMeshBoundsOnFailure
};

enum class VoxelPlannerExecutionMode
{
    Unknown,
    FullMeshBounds,
    VoxelLazy
};

struct VoxelLazyBuildOptions
{
    // Experimental. Keep disabled for correctness baseline.
    bool enabled = false;

    // 0 means disabled. When enabled, callers can reject lazy paths that are
    // much more expensive than the fallback/baseline path.
    double maxCostRegressionRatio = 0.0;

    // 0 means unlimited. Lazy reports timeout instead of falling back when the
    // budget is exceeded.
    double timeBudgetMs = 3000.0;

    VoxelLazyFallbackPolicy fallbackPolicy =
        VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure;
};

struct VoxelPathPlannerOptions
{
    VoxelMeshBuildOptions meshBuildOptions;
    VoxelLazyBuildOptions lazyBuildOptions;
    VoxelAStarOptions astarOptions;
    VoxelPlanningRunOptions runOptions;

    // Optional immutable FullMeshBounds voxel snapshot. It is used only by
    // FullMeshBounds runs; lazy runs intentionally ignore it.
    bool useCachedFullBoundsVoxelSpace = false;
    VoxelSpace cachedFullBoundsVoxelSpace;
    VoxelMeshBuildResult cachedFullBoundsBuildResult;

    // Extra voxel radius added to the model/search bounds after endpoints are
    // included. Larger values allow wider detours but increase A* work.
    int searchBoundsExtraRadius = 10;
    int debugNeighborhoodRadius = 10;
    // Maximum number of future control points tested by line-of-sight
    // shortcutting. 0 means unlimited.
    int optimizerMaxShortcutLookAhead = 200;
    // Enables post-A* curve smoothing. The smoothed path is accepted only if
    // sampled points and sampled segments stay out of occupied/restricted
    // voxels.
    bool smoothOptimizedPath = false;
    int smoothPathSamplesPerSegment = 8;
    int displayPathSamplesPerSegment = 48;
    double smoothPathSampleSpacing = 0.0;
    // Maximum allowed distance from a smoothed sample to its control polyline.
    // 0 disables the deviation limit.
    double smoothPathMaxDeviation = 0.0;
    // Weights used to rank valid smoothing candidates. Length is always part
    // of the score; these terms bias the result toward fewer turns, lower
    // curvature spikes, and less detour.
    double smoothingSignificantTurnWeight = 3.0;
    double smoothingTotalTurnWeight = 2.0;
    double smoothingMaxTurnWeight = 6.0;
    double smoothingDetourWeight = 1.5;
    std::vector<VoxelRestrictedHalfSpace> restrictedHalfSpaces;
};

struct VoxelPathPlannerResult
{
    bool success = false;
    VoxelPlannerExecutionMode executionMode =
        VoxelPlannerExecutionMode::Unknown;
    VoxelPlannerExecutionMode fallbackExecutionMode =
        VoxelPlannerExecutionMode::Unknown;
    double lazyAttemptCost = 0.0;
    double fallbackCost = 0.0;
    VoxelPlanningProfile profile;
    VoxelAStarResult astarResult;
    VoxelPathOptimizeResult optimizeResult;
    // Path used by the Workbench display. Unlike optimizeResult.pointPath,
    // this is validated as a full start-point to goal-point polyline for UI
    // rendering and voxel overlay. Occupied voxels are allowed only for the
    // endpoint voxels.
    std::vector<Vec> displayPathPoints;
    bool hasReusableFullBoundsVoxelSpace = false;
    VoxelSpace reusableFullBoundsVoxelSpace;
    VoxelMeshBuildResult reusableFullBoundsBuildResult;
    std::vector<Vec> pathFreeVoxelCenters;
    std::vector<Vec> pathClearanceVoxelCenters;
    std::vector<Vec> pathOccupiedVoxelCenters;
    VoxelBounds finalSearchBounds;
    bool hasFinalSearchBounds = false;
};

class VoxelPathPlanner
{
public:
    static VoxelPathPlannerOptions MakeDefaultOptions();

    static VoxelPathPlannerOptions MakeBrepBaselineOptions(
        const std::string& vtkPathPrefix);

    static bool ReadBrepShape(
        const std::string& path,
        TopoDS_Shape& shape);

    static bool MakeScenarioFromBrepFile(
        const VoxelBrepScenarioRequest& request,
        VoxelPlanningScenario& scenario,
        std::string* errorMessage = nullptr);

    static VoxelPathPlannerResult Plan(
        const VoxelPlanningScenario& scenario,
        const VoxelPathPlannerOptions& options);

    static void PrintProfile(
        const VoxelPlanningProfile& profile);

    static double SafeRatio(
        std::size_t numerator,
        std::size_t denominator);
};
