#pragma once

#include "VoxelAStar.h"
#include "VoxelChunkCache.h"
#include "VoxelMeshBuilder.h"
#include "VoxelPathOptimizer.h"

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cstddef>
#include <string>
#include <vector>

struct VoxelPlanningProfile
{
    double shapeMeshExportMs = 0.0;
    double triangulationMs = 0.0;
    double spatialIndexBuildMs = 0.0;
    double voxelBuildMs = 0.0;

    // Lazy chunk build time is included in astarMs because chunks are built
    // from A* ensureCellBuilt callbacks. The fields below split that cost.
    double lazyChunkBuildMs = 0.0;
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
    std::size_t lazyChunkBuildCount = 0;
    std::size_t lazyCacheHitCount = 0;
    std::size_t lazyFailedBuildCount = 0;
    std::size_t lazyCandidateTriangleCount = 0;
    std::size_t lazyRawCandidateTriangleCount = 0;
    std::size_t lazyVoxelVisitCount = 0;
    std::size_t lazyOutOfBoundsVoxelCount = 0;
    std::size_t lazyDistanceCalculationCount = 0;
    std::size_t lazyDistanceImprovedCount = 0;
    std::size_t lazyDistanceNotImprovedCount = 0;
    std::size_t lazyStateWriteCount = 0;
    std::size_t lazyStateUnchangedWriteCount = 0;
    std::size_t lazyOccupiedWriteCount = 0;
    std::size_t lazyClearanceWriteCount = 0;
    std::size_t lazyInfluenceCacheHitCount = 0;
    std::size_t lazyInfluenceCacheMissCount = 0;
    std::size_t lazyMinCandidateTriangleCount = 0;
    std::size_t lazyMaxCandidateTriangleCount = 0;
    std::size_t lazyMinRawCandidateTriangleCount = 0;
    std::size_t lazyMaxRawCandidateTriangleCount = 0;
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
    bool lazyBuildEnabled = false;
    bool lazyFallbackTriggered = false;
    std::string lazyFallbackReason;
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
    std::string lazyChunkBoundsVtkPath = "D:/lazy_chunk_bounds.vtk";
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
    StartGoalBox,
    LazyChunks
};

struct VoxelLazyBuildOptions
{
    // Experimental. Keep disabled for correctness baseline.
    bool enabled = false;

    VoxelChunkCacheOptions chunkCacheOptions;

    // 0 means unlimited. Intended as a guardrail against accidental full-model
    // expansion through lazy chunk generation.
    std::size_t maxChunkBuildCount = 0;

    // 0 means disabled. When enabled, callers can reject lazy paths that are
    // much more expensive than the fallback/baseline path.
    double maxCostRegressionRatio = 0.0;

    VoxelLazyFallbackPolicy fallbackPolicy =
        VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure;
};

struct VoxelPathPlannerOptions
{
    VoxelMeshBuildOptions meshBuildOptions;
    VoxelLocalBuildOptions localBuildOptions;
    VoxelLazyBuildOptions lazyBuildOptions;
    VoxelAStarOptions astarOptions;
    VoxelPlanningRunOptions runOptions;

    int searchBoundsExtraRadius = 10;
    int debugNeighborhoodRadius = 10;
    int optimizerMaxShortcutLookAhead = 200;
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
    VoxelBounds finalSearchBounds;
    bool hasFinalSearchBounds = false;
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
