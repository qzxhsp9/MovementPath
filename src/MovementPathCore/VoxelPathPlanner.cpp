#include "VoxelPathPlanner.h"

#include "MeshVtkExporter.h"
#include "VoxelPathOptimizer.h"
#include "VoxelVtkExporter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace
{
class ScopedTimer
{
public:
    explicit ScopedTimer(double& elapsedMs)
        : m_elapsedMs(elapsedMs),
        m_start(std::chrono::steady_clock::now())
    {
    }

    ~ScopedTimer()
    {
        const auto end = std::chrono::steady_clock::now();
        m_elapsedMs += std::chrono::duration<double, std::milli>(
            end - m_start
        ).count();
    }

private:
    double& m_elapsedMs;
    std::chrono::steady_clock::time_point m_start;
};

const char* ToString(VoxelBuildRegionMode mode)
{
    if (mode == VoxelBuildRegionMode::StartGoalBox)
    {
        return "StartGoalBox";
    }

    return "FullMeshBounds";
}

VoxelPlannerExecutionMode ToExecutionMode(
    VoxelBuildRegionMode mode)
{
    if (mode == VoxelBuildRegionMode::StartGoalBox)
    {
        return VoxelPlannerExecutionMode::StartGoalBox;
    }

    return VoxelPlannerExecutionMode::FullMeshBounds;
}

void PreserveLazyAttemptOnFallback(
    const VoxelPlanningProfile& lazyProfile,
    VoxelPathPlannerResult& fallbackResult)
{
    fallbackResult.profile.lazyBuildEnabled = true;
    fallbackResult.profile.lazyFallbackTriggered = true;
    fallbackResult.profile.lazyFallbackReason =
        lazyProfile.lazyFallbackReason;
    fallbackResult.profile.lazyChunkBuildCount =
        lazyProfile.lazyChunkBuildCount;
    fallbackResult.profile.lazyEnsureCallCount =
        lazyProfile.lazyEnsureCallCount;
    fallbackResult.profile.lazyChunkBuildMs =
        lazyProfile.lazyChunkBuildMs;
    fallbackResult.profile.lazyCandidateQueryMs =
        lazyProfile.lazyCandidateQueryMs;
    fallbackResult.profile.lazyCandidateFilterMs =
        lazyProfile.lazyCandidateFilterMs;
    fallbackResult.profile.lazyVoxelMarkMs =
        lazyProfile.lazyVoxelMarkMs;
    fallbackResult.profile.lazyStateCountMs =
        lazyProfile.lazyStateCountMs;
    fallbackResult.profile.lazyCacheHitCount =
        lazyProfile.lazyCacheHitCount;
    fallbackResult.profile.lazyFailedBuildCount =
        lazyProfile.lazyFailedBuildCount;
    fallbackResult.profile.lazyCandidateTriangleCount =
        lazyProfile.lazyCandidateTriangleCount;
    fallbackResult.profile.lazyRawCandidateTriangleCount =
        lazyProfile.lazyRawCandidateTriangleCount;
    fallbackResult.profile.lazyVoxelVisitCount =
        lazyProfile.lazyVoxelVisitCount;
    fallbackResult.profile.lazyOutOfBoundsVoxelCount =
        lazyProfile.lazyOutOfBoundsVoxelCount;
    fallbackResult.profile.lazyDistanceCalculationCount =
        lazyProfile.lazyDistanceCalculationCount;
    fallbackResult.profile.lazyDistanceImprovedCount =
        lazyProfile.lazyDistanceImprovedCount;
    fallbackResult.profile.lazyStateWriteCount =
        lazyProfile.lazyStateWriteCount;
    fallbackResult.profile.lazyOccupiedWriteCount =
        lazyProfile.lazyOccupiedWriteCount;
    fallbackResult.profile.lazyClearanceWriteCount =
        lazyProfile.lazyClearanceWriteCount;
    fallbackResult.profile.lazyInfluenceCacheHitCount =
        lazyProfile.lazyInfluenceCacheHitCount;
    fallbackResult.profile.lazyInfluenceCacheMissCount =
        lazyProfile.lazyInfluenceCacheMissCount;
}

MeshAABB MakeStartGoalBuildBox(
    const Vec& startPoint,
    const Vec& goalPoint,
    double expand)
{
    MeshAABB box;

    box.minP = Vec(
        std::min(startPoint.x, goalPoint.x),
        std::min(startPoint.y, goalPoint.y),
        std::min(startPoint.z, goalPoint.z)
    );

    box.maxP = Vec(
        std::max(startPoint.x, goalPoint.x),
        std::max(startPoint.y, goalPoint.y),
        std::max(startPoint.z, goalPoint.z)
    );

    box.minP.x -= expand;
    box.minP.y -= expand;
    box.minP.z -= expand;

    box.maxP.x += expand;
    box.maxP.y += expand;
    box.maxP.z += expand;

    return box;
}

double ComputeLocalSearchPadding(
    const VoxelLocalBuildOptions& options,
    int attemptIndex)
{
    if (attemptIndex <= 0)
    {
        return options.searchPadding;
    }

    return options.searchPadding *
        std::pow(options.retryExpandFactor, attemptIndex);
}

VoxelMeshBuildResult BuildVoxelSpaceForAttempt(
    const std::vector<MeshTriangle>& triangles,
    const TriangleSpatialHash& spatialHash,
    const VoxelMeshBuildOptions& buildOptions,
    const VoxelLocalBuildOptions& localOptions,
    const Vec& startPoint,
    const Vec& goalPoint,
    double searchPadding,
    VoxelSpace& voxelSpace)
{
    if (localOptions.regionMode == VoxelBuildRegionMode::FullMeshBounds)
    {
        return VoxelMeshBuilder::BuildVoxelSpaceFromTriangles(
            triangles,
            buildOptions,
            voxelSpace
        );
    }

    const double halfDiag =
        0.5 * std::sqrt(3.0) * buildOptions.voxelSize;
    const double expand =
        buildOptions.clearance + halfDiag + searchPadding;

    const MeshAABB buildBox =
        MakeStartGoalBuildBox(startPoint, goalPoint, expand);

    return VoxelMeshBuilder::BuildVoxelSpaceFromTrianglesInBox(
        triangles,
        &spatialHash,
        buildBox,
        buildOptions,
        voxelSpace
    );
}

void DebugVoxelStateAround(
    const VoxelSpace& space,
    const VoxelIndex& seed,
    int radius)
{
    int freeCount = 0;
    int occupiedCount = 0;
    int clearanceBandCount = 0;
    int startCount = 0;
    int goalCount = 0;
    int pathCount = 0;
    int outOfBoundsCount = 0;

    for (int dx = -radius; dx <= radius; ++dx)
    {
        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dz = -radius; dz <= radius; ++dz)
            {
                VoxelIndex index(
                    seed.x + dx,
                    seed.y + dy,
                    seed.z + dz
                );

                if (!space.IsInsideSearchBounds(index))
                {
                    ++outOfBoundsCount;
                    continue;
                }

                VoxelState state = space.GetCellState(index);

                switch (state)
                {
                case VoxelState::Free:
                    ++freeCount;
                    break;
                case VoxelState::Occupied:
                    ++occupiedCount;
                    break;
                case VoxelState::ClearanceBand:
                    ++clearanceBandCount;
                    break;
                case VoxelState::Start:
                    ++startCount;
                    break;
                case VoxelState::Goal:
                    ++goalCount;
                    break;
                case VoxelState::Path:
                    ++pathCount;
                    break;
                default:
                    break;
                }
            }
        }
    }

    std::cout << "Debug around index: "
        << seed.x << ", "
        << seed.y << ", "
        << seed.z << std::endl;

    std::cout << "radius: " << radius << std::endl;
    std::cout << "Free: " << freeCount << std::endl;
    std::cout << "Occupied: " << occupiedCount << std::endl;
    std::cout << "ClearanceBand: " << clearanceBandCount << std::endl;
    std::cout << "Start: " << startCount << std::endl;
    std::cout << "Goal: " << goalCount << std::endl;
    std::cout << "Path: " << pathCount << std::endl;
    std::cout << "OutOfBounds: " << outOfBoundsCount << std::endl;
}

void CopyBuildStatsToProfile(
    const VoxelMeshBuildResult& buildResult,
    const VoxelSpace& voxelSpace,
    VoxelPlanningProfile& profile)
{
    profile.triangleCount = buildResult.triangleCount;
    profile.candidateTriangleCount =
        buildResult.candidateTriangleCount;
    profile.rawCandidateTriangleCount =
        buildResult.rawCandidateTriangleCount;
    profile.hashQueryCellCount =
        buildResult.hashQueryCellCount;
    profile.hashRawTriangleCount =
        buildResult.hashRawTriangleCount;
    profile.storedCellCount = voxelSpace.CellCount();
    profile.occupiedCount = buildResult.occupiedVoxelCount;
    profile.clearanceBandCount =
        buildResult.clearanceBandVoxelCount;
}

void CopyLazyStatsToProfile(
    const VoxelChunkCacheStats& stats,
    VoxelPlanningProfile& profile)
{
    profile.lazyEnsureCallCount = stats.ensureCallCount;
    profile.lazyChunkBuildCount = stats.chunkBuildCount;
    profile.lazyCacheHitCount = stats.cacheHitCount;
    profile.lazyFailedBuildCount = stats.failedBuildCount;
    profile.lazyChunkBuildMs = stats.totalBuildMs;
    profile.lazyCandidateQueryMs = stats.totalCandidateQueryMs;
    profile.lazyCandidateFilterMs = stats.totalCandidateFilterMs;
    profile.lazyVoxelMarkMs = stats.totalVoxelMarkMs;
    profile.lazyStateCountMs = stats.totalStateCountMs;
    profile.lazyCandidateTriangleCount =
        stats.totalCandidateTriangleCount;
    profile.lazyRawCandidateTriangleCount =
        stats.totalRawCandidateTriangleCount;
    profile.lazyVoxelVisitCount = stats.totalVoxelVisitCount;
    profile.lazyOutOfBoundsVoxelCount =
        stats.totalOutOfBoundsVoxelCount;
    profile.lazyDistanceCalculationCount =
        stats.totalDistanceCalculationCount;
    profile.lazyDistanceImprovedCount =
        stats.totalDistanceImprovedCount;
    profile.lazyStateWriteCount = stats.totalStateWriteCount;
    profile.lazyOccupiedWriteCount = stats.totalOccupiedWriteCount;
    profile.lazyClearanceWriteCount = stats.totalClearanceWriteCount;
    profile.lazyInfluenceCacheHitCount =
        stats.totalInfluenceCacheHitCount;
    profile.lazyInfluenceCacheMissCount =
        stats.totalInfluenceCacheMissCount;
}

bool ComputeTrianglesAABBForPlanner(
    const std::vector<MeshTriangle>& triangles,
    MeshAABB& outBox)
{
    if (triangles.empty())
    {
        return false;
    }

    double xmin = triangles[0].p0.x;
    double ymin = triangles[0].p0.y;
    double zmin = triangles[0].p0.z;
    double xmax = triangles[0].p0.x;
    double ymax = triangles[0].p0.y;
    double zmax = triangles[0].p0.z;

    for (const MeshTriangle& tri : triangles)
    {
        const Vec points[] = { tri.p0, tri.p1, tri.p2 };

        for (const Vec& point : points)
        {
            xmin = std::min(xmin, point.x);
            ymin = std::min(ymin, point.y);
            zmin = std::min(zmin, point.z);
            xmax = std::max(xmax, point.x);
            ymax = std::max(ymax, point.y);
            zmax = std::max(zmax, point.z);
        }
    }

    outBox.minP = Vec(xmin, ymin, zmin);
    outBox.maxP = Vec(xmax, ymax, zmax);

    return true;
}

void ExpandAABBForPlanner(
    MeshAABB& box,
    double offset)
{
    if (offset <= 0.0)
    {
        return;
    }

    box.minP.x -= offset;
    box.minP.y -= offset;
    box.minP.z -= offset;
    box.maxP.x += offset;
    box.maxP.y += offset;
    box.maxP.z += offset;
}

void NormalizeIndexRangeForPlanner(
    VoxelIndex& minIndex,
    VoxelIndex& maxIndex)
{
    if (minIndex.x > maxIndex.x)
    {
        std::swap(minIndex.x, maxIndex.x);
    }

    if (minIndex.y > maxIndex.y)
    {
        std::swap(minIndex.y, maxIndex.y);
    }

    if (minIndex.z > maxIndex.z)
    {
        std::swap(minIndex.z, maxIndex.z);
    }
}

void CountVoxelStates(
    const VoxelSpace& space,
    std::size_t& occupiedCount,
    std::size_t& clearanceBandCount)
{
    occupiedCount = 0;
    clearanceBandCount = 0;

    for (const auto& kv : space.Cells())
    {
        if (kv.second.state == VoxelState::Occupied)
        {
            ++occupiedCount;
        }
        else if (kv.second.state == VoxelState::ClearanceBand)
        {
            ++clearanceBandCount;
        }
    }
}

bool InitializeLazyVoxelSpace(
    const std::vector<MeshTriangle>& triangles,
    const VoxelMeshBuildOptions& options,
    VoxelSpace& voxelSpace,
    VoxelBounds& outBounds)
{
    MeshAABB globalBox;

    if (!ComputeTrianglesAABBForPlanner(triangles, globalBox))
    {
        return false;
    }

    const double globalExpand =
        std::max(0.0, options.bboxPadding) +
        std::max(0.0, options.clearance) +
        options.voxelSize;

    ExpandAABBForPlanner(globalBox, globalExpand);

    voxelSpace = VoxelSpace(globalBox.minP, options.voxelSize);

    outBounds.minIndex = voxelSpace.WorldToIndex(globalBox.minP);
    outBounds.maxIndex = voxelSpace.WorldToIndex(globalBox.maxP);
    NormalizeIndexRangeForPlanner(outBounds.minIndex, outBounds.maxIndex);

    voxelSpace.SetSearchBounds(outBounds);

    return true;
}
}

double VoxelPathPlanner::SafeRatio(
    std::size_t numerator,
    std::size_t denominator)
{
    if (denominator == 0)
    {
        return 0.0;
    }

    return static_cast<double>(numerator) /
        static_cast<double>(denominator);
}

VoxelPathPlannerOptions VoxelPathPlanner::MakeDefaultOptions()
{
    VoxelPathPlannerOptions options;

    options.meshBuildOptions.voxelSize = 1.0;
    options.meshBuildOptions.meshDeflection = 0.25;
    options.meshBuildOptions.angularDeflection = 0.3;
    options.meshBuildOptions.clearance = 3.0;
    options.meshBuildOptions.bboxPadding = 10.0;
    options.meshBuildOptions.conservativeClearance = true;
    options.meshBuildOptions.storeFreeCells = false;

    options.localBuildOptions.regionMode = VoxelBuildRegionMode::FullMeshBounds;
    options.localBuildOptions.searchPadding = 20.0;
    options.localBuildOptions.maxRetryCount = 3;
    options.localBuildOptions.retryExpandFactor = 2.0;

    options.lazyBuildOptions.enabled = false;
    options.lazyBuildOptions.chunkCacheOptions.chunkVoxelSize = 16;
    options.lazyBuildOptions.chunkCacheOptions.buildPadding =
        options.meshBuildOptions.clearance +
        0.5 * std::sqrt(3.0) * options.meshBuildOptions.voxelSize;
    options.lazyBuildOptions.maxChunkBuildCount = 0;
    options.lazyBuildOptions.maxCostRegressionRatio = 0.0;
    options.lazyBuildOptions.fallbackPolicy =
        VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure;

    options.astarOptions.searchMode = VoxelAStarSearchMode::ClearanceBand;
    options.astarOptions.neighborType = VoxelNeighborType::Face6;
    options.astarOptions.heuristicWeight = 1.0;
    options.astarOptions.turnPenalty =
        options.meshBuildOptions.voxelSize * 0.1;
    options.astarOptions.snapStartGoalToWalkable = true;
    options.astarOptions.snapMaxRadius = 20;
    options.astarOptions.useStartSnapDirection = true;
    options.astarOptions.useGoalSnapDirection = true;
    options.astarOptions.maxVisitedCount = 0;
    options.astarOptions.markPathToVoxelSpace = true;

    return options;
}

VoxelPathPlannerResult VoxelPathPlanner::Plan(
    const VoxelPlanningScenario& scenario,
    const VoxelPathPlannerOptions& options)
{
    VoxelPathPlannerResult result;
    VoxelPlanningProfile& profile = result.profile;
    profile.scenarioName = scenario.name;
    profile.buildRegionMode = ToString(options.localBuildOptions.regionMode);
    profile.lazyBuildEnabled = options.lazyBuildOptions.enabled;
    result.executionMode = ToExecutionMode(options.localBuildOptions.regionMode);

    std::vector<MeshTriangle> triangles;

    {
        ScopedTimer timer(profile.triangulationMs);

        if (!VoxelMeshBuilder::BuildShapeTriangulation(
            scenario.shape,
            options.meshBuildOptions.meshDeflection,
            options.meshBuildOptions.angularDeflection,
            triangles))
        {
            if (options.runOptions.verbose)
            {
                std::cout << "Build shape triangulation failed." << std::endl;
            }
            return result;
        }
    }

    profile.triangleCount = triangles.size();

    if (options.runOptions.exportVtk)
    {
        bool ok = false;

        {
            ScopedTimer timer(profile.shapeMeshExportMs);
            ok = MeshVtkExporter::ExportTrianglesToVtk(
                triangles,
                options.runOptions.shapeMeshVtkPath
            );
        }

        if (options.runOptions.verbose)
        {
            std::cout
                << (ok ? "Export shape mesh success." :
                    "Export shape mesh failed.")
                << std::endl;
        }
    }

    TriangleSpatialHash spatialHash;
    TriangleSpatialHashOptions spatialHashOptions;
    spatialHashOptions.cellSize =
        std::max(10.0, options.meshBuildOptions.voxelSize * 10.0);

    {
        ScopedTimer timer(profile.spatialIndexBuildMs);

        if (!spatialHash.Build(triangles, spatialHashOptions))
        {
            if (options.runOptions.verbose)
            {
                std::cout << "Build triangle spatial hash failed."
                    << std::endl;
            }
            return result;
        }
    }

    TriangleSpatialHashStats hashStats = spatialHash.GetStats();
    profile.hashCellCount = hashStats.cellCount;
    profile.hashEntryCount = hashStats.entryCount;

    Vec startPoint3D = {
        scenario.startPoint.X(),
        scenario.startPoint.Y(),
        scenario.startPoint.Z()
    };
    Vec goalPoint3D = {
        scenario.goalPoint.X(),
        scenario.goalPoint.Y(),
        scenario.goalPoint.Z()
    };

    VoxelAStarOptions astarOptions = options.astarOptions;
    astarOptions.startSnapDirection = {
        scenario.startDir.X(),
        scenario.startDir.Y(),
        scenario.startDir.Z()
    };
    astarOptions.goalSnapDirection = {
        scenario.goalDir.X(),
        scenario.goalDir.Y(),
        scenario.goalDir.Z()
    };

    if (options.lazyBuildOptions.enabled)
    {
        profile.buildRegionMode = "LazyChunks";
        result.executionMode = VoxelPlannerExecutionMode::LazyChunks;

        VoxelSpace lazyVoxelSpace;
        VoxelBounds lazyBounds;

        if (!InitializeLazyVoxelSpace(
            triangles,
            options.meshBuildOptions,
            lazyVoxelSpace,
            lazyBounds))
        {
            profile.lazyFallbackTriggered = true;
            profile.lazyFallbackReason = "failed to initialize lazy bounds";
        }
        else
        {
            result.finalSearchBounds = lazyBounds;
            result.hasFinalSearchBounds = lazyBounds.IsValid();
            profile.buildAttemptCount = 1;
            profile.finalSearchPadding = 0.0;
            profile.buildSucceeded = true;

            VoxelChunkCache chunkCache;
            bool lazyGuardrailTriggered = false;

            if (!chunkCache.Configure(
                &triangles,
                &spatialHash,
                options.meshBuildOptions,
                options.lazyBuildOptions.chunkCacheOptions))
            {
                profile.lazyFallbackTriggered = true;
                profile.lazyFallbackReason = "failed to configure chunk cache";
            }
            else
            {
                astarOptions.ensureCellBuilt =
                    [&chunkCache,
                     &options,
                     &lazyGuardrailTriggered,
                     &profile](
                        VoxelSpace& space,
                        const VoxelIndex& index)
                    {
                        const std::size_t maxChunkBuildCount =
                            options.lazyBuildOptions.maxChunkBuildCount;

                        if (maxChunkBuildCount > 0 &&
                            !chunkCache.IsChunkBuiltForIndex(index) &&
                            chunkCache.GetStats().chunkBuildCount >=
                                maxChunkBuildCount)
                        {
                            lazyGuardrailTriggered = true;
                            profile.lazyFallbackReason =
                                "maxChunkBuildCount exceeded";
                            return;
                        }

                        chunkCache.EnsureChunkForIndex(space, index);
                    };

                VoxelAStarResult lazyAStarResult;

                {
                    ScopedTimer timer(profile.astarMs);
                    lazyAStarResult =
                        VoxelAStar::Search(
                            lazyVoxelSpace,
                            startPoint3D,
                            goalPoint3D,
                            astarOptions
                        );
                }

                CopyLazyStatsToProfile(chunkCache.GetStats(), profile);

                profile.astarVisitedCount = lazyAStarResult.visitedCount;
                profile.rawPathCount = lazyAStarResult.voxelPath.size();
                profile.totalCost = lazyAStarResult.totalCost;
                profile.astarSucceeded = lazyAStarResult.success;
                result.astarResult = lazyAStarResult;

                CountVoxelStates(
                    lazyVoxelSpace,
                    profile.occupiedCount,
                    profile.clearanceBandCount);
                profile.storedCellCount = lazyVoxelSpace.CellCount();

                if (options.runOptions.exportVtk)
                {
                    const std::vector<VoxelChunkIndex> builtChunks =
                        chunkCache.GetBuiltChunks();

                    ScopedTimer timer(profile.vtkExportMs);
                    VoxelVtkExporter::ExportChunkBoundsToVtk(
                        lazyVoxelSpace,
                        builtChunks.data(),
                        builtChunks.size(),
                        options.lazyBuildOptions
                            .chunkCacheOptions.chunkVoxelSize,
                        options.runOptions.lazyChunkBoundsVtkPath);
                }

                if (lazyGuardrailTriggered)
                {
                    profile.lazyFallbackTriggered = true;
                }

                if (lazyAStarResult.success && !lazyGuardrailTriggered)
                {
                    VoxelPathOptimizeOptions optOptions;
                    optOptions.searchMode = astarOptions.searchMode;
                    optOptions.removeCollinear = true;
                    optOptions.enableLineOfSightShortcut = true;
                    optOptions.maxShortcutLookAhead =
                        options.optimizerMaxShortcutLookAhead;

                    VoxelPathOptimizeResult optResult;

                    {
                        ScopedTimer timer(profile.optimizeMs);
                        optResult =
                            VoxelPathOptimizer::Optimize(
                                lazyVoxelSpace,
                                lazyAStarResult.voxelPath,
                                optOptions
                            );
                    }

                    profile.optimizedPathCount = optResult.outputCount;
                    profile.lineCheckCount = optResult.lineCheckCount;
                    result.optimizeResult = optResult;
                    result.success = true;
                    result.lazyAttemptCost = lazyAStarResult.totalCost;

                    if (options.lazyBuildOptions.maxCostRegressionRatio > 0.0)
                    {
                        VoxelPathPlannerOptions baselineOptions = options;
                        baselineOptions.lazyBuildOptions.enabled = false;
                        baselineOptions.localBuildOptions.regionMode =
                            VoxelBuildRegionMode::FullMeshBounds;

                        VoxelPathPlannerResult baselineResult =
                            VoxelPathPlanner::Plan(
                                scenario,
                                baselineOptions);

                        result.fallbackCost =
                            baselineResult.profile.totalCost;

                        if (baselineResult.success &&
                            lazyAStarResult.totalCost >
                                baselineResult.profile.totalCost *
                                options.lazyBuildOptions
                                    .maxCostRegressionRatio)
                        {
                            profile.lazyFallbackTriggered = true;
                            profile.lazyFallbackReason =
                                "maxCostRegressionRatio exceeded";

                            PreserveLazyAttemptOnFallback(
                                profile,
                                baselineResult);

                            baselineResult.lazyAttemptCost =
                                lazyAStarResult.totalCost;
                            baselineResult.fallbackCost =
                                baselineResult.profile.totalCost;
                            baselineResult.fallbackExecutionMode =
                                VoxelPlannerExecutionMode::FullMeshBounds;

                            return baselineResult;
                        }
                    }

                    return result;
                }

                if (!profile.lazyFallbackTriggered)
                {
                    profile.lazyFallbackTriggered = true;
                    profile.lazyFallbackReason = "lazy A* failed";
                }
            }
        }

        if (options.lazyBuildOptions.fallbackPolicy ==
            VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure)
        {
            VoxelPathPlannerOptions fallbackOptions = options;
            fallbackOptions.lazyBuildOptions.enabled = false;
            fallbackOptions.localBuildOptions.regionMode =
                VoxelBuildRegionMode::FullMeshBounds;

            VoxelPathPlannerResult fallbackResult =
                VoxelPathPlanner::Plan(scenario, fallbackOptions);

            PreserveLazyAttemptOnFallback(profile, fallbackResult);
            fallbackResult.lazyAttemptCost = profile.totalCost;
            fallbackResult.fallbackCost = fallbackResult.profile.totalCost;
            fallbackResult.fallbackExecutionMode =
                VoxelPlannerExecutionMode::FullMeshBounds;

            return fallbackResult;
        }

        return result;
    }

    VoxelAStarResult astarResult;
    VoxelMeshBuildResult buildResult;
    VoxelSpace voxelSpace;

    const int maxAttemptCount =
        options.localBuildOptions.regionMode ==
            VoxelBuildRegionMode::FullMeshBounds ?
        1 :
        options.localBuildOptions.maxRetryCount + 1;

    for (int attempt = 0; attempt < maxAttemptCount; ++attempt)
    {
        const double searchPadding =
            options.localBuildOptions.regionMode ==
                VoxelBuildRegionMode::FullMeshBounds ?
            0.0 :
            ComputeLocalSearchPadding(options.localBuildOptions, attempt);

        if (options.runOptions.verbose)
        {
            std::cout << "Build/search attempt: "
                << attempt + 1 << " / " << maxAttemptCount
                << ", mode: " << ToString(options.localBuildOptions.regionMode)
                << ", search padding: " << searchPadding
                << std::endl;
        }

        {
            ScopedTimer timer(profile.voxelBuildMs);
            buildResult =
                BuildVoxelSpaceForAttempt(
                    triangles,
                    spatialHash,
                    options.meshBuildOptions,
                    options.localBuildOptions,
                    startPoint3D,
                    goalPoint3D,
                    searchPadding,
                    voxelSpace
                );
        }

        profile.buildAttemptCount = attempt + 1;
        profile.finalSearchPadding = searchPadding;
        profile.buildSucceeded = buildResult.success;

        if (!buildResult.success)
        {
            if (options.runOptions.verbose)
            {
                std::cout << "Build voxel space failed." << std::endl;
            }
            continue;
        }

        CopyBuildStatsToProfile(buildResult, voxelSpace, profile);

        if (options.runOptions.debugNeighborhood)
        {
            DebugVoxelStateAround(
                voxelSpace,
                voxelSpace.WorldToIndex(startPoint3D),
                options.debugNeighborhoodRadius
            );
            DebugVoxelStateAround(
                voxelSpace,
                voxelSpace.WorldToIndex(goalPoint3D),
                options.debugNeighborhoodRadius
            );
        }

        VoxelBounds bounds = voxelSpace.GetSearchBounds();

        ExpandBoundsToInclude(bounds, voxelSpace.WorldToIndex(startPoint3D));
        ExpandBoundsToInclude(bounds, voxelSpace.WorldToIndex(goalPoint3D));
        ExpandBoundsByVoxelRadius(bounds, options.searchBoundsExtraRadius);

        voxelSpace.SetSearchBounds(bounds);
        result.finalSearchBounds = bounds;
        result.hasFinalSearchBounds = bounds.IsValid();

        {
            ScopedTimer timer(profile.astarMs);
            astarResult =
                VoxelAStar::Search(
                    voxelSpace,
                    startPoint3D,
                    goalPoint3D,
                    astarOptions
                );
        }

        profile.astarVisitedCount = astarResult.visitedCount;
        profile.rawPathCount = astarResult.voxelPath.size();
        profile.totalCost = astarResult.totalCost;
        profile.astarSucceeded = astarResult.success;

        if (astarResult.success)
        {
            break;
        }

        if (options.runOptions.verbose)
        {
            std::cout << "A* failed in attempt "
                << attempt + 1 << "." << std::endl;
        }
    }

    result.astarResult = astarResult;

    if (!profile.buildSucceeded)
    {
        return result;
    }

    if (!astarResult.success)
    {
        if (options.runOptions.verbose)
        {
            std::cout << "A* failed." << std::endl;
            std::cout << "Visited count: "
                << astarResult.visitedCount << std::endl;
        }

        if (options.runOptions.exportVtk)
        {
            ScopedTimer timer(profile.vtkExportMs);
            VoxelVtkExporter::ExportVoxelSpaceToVtk(
                voxelSpace,
                options.runOptions.astarFailedVtkPath,
                {
                    VoxelState::Occupied,
                    VoxelState::ClearanceBand,
                    VoxelState::Start,
                    VoxelState::Goal
                }
            );
        }

        profile.storedCellCount = voxelSpace.CellCount();
        return result;
    }

    if (options.runOptions.verbose)
    {
        std::cout << "A* success." << std::endl;
        std::cout << "Visited count: "
            << astarResult.visitedCount << std::endl;
        std::cout << "Path voxel count: "
            << astarResult.voxelPath.size() << std::endl;
        std::cout << "Total cost: "
            << astarResult.totalCost << std::endl;
    }

    if (options.runOptions.exportVtk)
    {
        ScopedTimer timer(profile.vtkExportMs);
        VoxelVtkExporter::ExportVoxelSpaceToVtk(
            voxelSpace,
            options.runOptions.astarPathVtkPath,
            {
                VoxelState::Start,
                VoxelState::Goal,
                VoxelState::Path
            }
        );
    }

    VoxelPathOptimizeOptions optOptions;
    optOptions.searchMode = astarOptions.searchMode;
    optOptions.removeCollinear = true;
    optOptions.enableLineOfSightShortcut = true;
    optOptions.maxShortcutLookAhead = options.optimizerMaxShortcutLookAhead;

    VoxelPathOptimizeResult optResult;

    {
        ScopedTimer timer(profile.optimizeMs);
        optResult =
            VoxelPathOptimizer::Optimize(
                voxelSpace,
                astarResult.voxelPath,
                optOptions
            );
    }

    profile.optimizedPathCount = optResult.outputCount;
    profile.lineCheckCount = optResult.lineCheckCount;
    result.optimizeResult = optResult;

    if (options.runOptions.verbose)
    {
        std::cout << "Path optimize result:" << std::endl;
        std::cout << "Input count: "
            << optResult.inputCount << std::endl;
        std::cout << "After collinear: "
            << optResult.afterCollinearCount << std::endl;
        std::cout << "Output count: "
            << optResult.outputCount << std::endl;
        std::cout << "Line check count: "
            << optResult.lineCheckCount << std::endl;
    }

    VoxelVtkExporter::MarkPathToVoxelSpace(
        voxelSpace,
        optResult.voxelPath
    );

    if (options.runOptions.exportVtk)
    {
        ScopedTimer timer(profile.vtkExportMs);
        VoxelVtkExporter::ExportVoxelSpaceToVtk(
            voxelSpace,
            options.runOptions.optimizedPathVoxelsVtkPath,
            {
                VoxelState::Occupied,
                VoxelState::ClearanceBand,
                VoxelState::Start,
                VoxelState::Goal,
                VoxelState::Path
            }
        );

        VoxelVtkExporter::ExportPathPolylineToVtk(
            voxelSpace,
            optResult.voxelPath,
            options.runOptions.optimizedPathPolylineVtkPath
        );
    }

    profile.storedCellCount = voxelSpace.CellCount();
    result.success = true;
    result.astarResult = astarResult;

    return result;
}

void VoxelPathPlanner::PrintProfile(
    const VoxelPlanningProfile& profile)
{
    std::cout << std::fixed << std::setprecision(3);

    std::cout << "Voxel planning profile:" << std::endl;
    std::cout << "Scenario: "
        << profile.scenarioName << std::endl;
    std::cout << "Shape mesh export ms: "
        << profile.shapeMeshExportMs << std::endl;
    std::cout << "Triangulation ms: "
        << profile.triangulationMs << std::endl;
    std::cout << "Spatial index build ms: "
        << profile.spatialIndexBuildMs << std::endl;
    std::cout << "Voxel build ms: "
        << profile.voxelBuildMs << std::endl;
    std::cout << "Lazy chunk build ms: "
        << profile.lazyChunkBuildMs << std::endl;
    std::cout << "Lazy candidate query ms: "
        << profile.lazyCandidateQueryMs << std::endl;
    std::cout << "Lazy candidate filter ms: "
        << profile.lazyCandidateFilterMs << std::endl;
    std::cout << "Lazy voxel mark ms: "
        << profile.lazyVoxelMarkMs << std::endl;
    std::cout << "Lazy state count ms: "
        << profile.lazyStateCountMs << std::endl;
    std::cout << "A* ms: "
        << profile.astarMs << std::endl;
    std::cout << "Optimize ms: "
        << profile.optimizeMs << std::endl;
    std::cout << "VTK export ms: "
        << profile.vtkExportMs << std::endl;

    std::cout << "Build region mode: "
        << profile.buildRegionMode << std::endl;
    std::cout << "Build attempt count: "
        << profile.buildAttemptCount << std::endl;
    std::cout << "Final search padding: "
        << profile.finalSearchPadding << std::endl;
    std::cout << "Build succeeded: "
        << (profile.buildSucceeded ? "true" : "false") << std::endl;
    std::cout << "A* succeeded: "
        << (profile.astarSucceeded ? "true" : "false") << std::endl;
    std::cout << "Lazy build enabled: "
        << (profile.lazyBuildEnabled ? "true" : "false") << std::endl;
    std::cout << "Lazy fallback triggered: "
        << (profile.lazyFallbackTriggered ? "true" : "false") << std::endl;
    std::cout << "Lazy fallback reason: "
        << profile.lazyFallbackReason << std::endl;

    std::cout << "Triangle count: "
        << profile.triangleCount << std::endl;
    std::cout << "Candidate triangle count: "
        << profile.candidateTriangleCount << std::endl;
    std::cout << "Raw candidate triangle count: "
        << profile.rawCandidateTriangleCount << std::endl;
    std::cout << "Hash cell count: "
        << profile.hashCellCount << std::endl;
    std::cout << "Hash entry count: "
        << profile.hashEntryCount << std::endl;
    std::cout << "Hash query cell count: "
        << profile.hashQueryCellCount << std::endl;
    std::cout << "Hash raw triangle count: "
        << profile.hashRawTriangleCount << std::endl;
    std::cout << "Lazy ensure call count: "
        << profile.lazyEnsureCallCount << std::endl;
    std::cout << "Lazy chunk build count: "
        << profile.lazyChunkBuildCount << std::endl;
    std::cout << "Lazy cache hit count: "
        << profile.lazyCacheHitCount << std::endl;
    std::cout << "Lazy failed build count: "
        << profile.lazyFailedBuildCount << std::endl;
    std::cout << "Lazy candidate triangle count: "
        << profile.lazyCandidateTriangleCount << std::endl;
    std::cout << "Lazy raw candidate triangle count: "
        << profile.lazyRawCandidateTriangleCount << std::endl;
    std::cout << "Lazy voxel visit count: "
        << profile.lazyVoxelVisitCount << std::endl;
    std::cout << "Lazy out-of-bounds voxel count: "
        << profile.lazyOutOfBoundsVoxelCount << std::endl;
    std::cout << "Lazy distance calculation count: "
        << profile.lazyDistanceCalculationCount << std::endl;
    std::cout << "Lazy distance improved count: "
        << profile.lazyDistanceImprovedCount << std::endl;
    std::cout << "Lazy state write count: "
        << profile.lazyStateWriteCount << std::endl;
    std::cout << "Lazy occupied write count: "
        << profile.lazyOccupiedWriteCount << std::endl;
    std::cout << "Lazy clearance write count: "
        << profile.lazyClearanceWriteCount << std::endl;
    std::cout << "Lazy influence cache hit count: "
        << profile.lazyInfluenceCacheHitCount << std::endl;
    std::cout << "Lazy influence cache miss count: "
        << profile.lazyInfluenceCacheMissCount << std::endl;
    std::cout << "Stored cell count: "
        << profile.storedCellCount << std::endl;
    std::cout << "Occupied count: "
        << profile.occupiedCount << std::endl;
    std::cout << "ClearanceBand count: "
        << profile.clearanceBandCount << std::endl;

    std::cout << "A* visited count: "
        << profile.astarVisitedCount << std::endl;
    std::cout << "Raw path count: "
        << profile.rawPathCount << std::endl;
    std::cout << "Optimized path count: "
        << profile.optimizedPathCount << std::endl;
    std::cout << "Line check count: "
        << profile.lineCheckCount << std::endl;
    std::cout << "Total cost: "
        << profile.totalCost << std::endl;

    std::cout << std::setprecision(6);
    std::cout << "Path usage ratio: "
        << SafeRatio(profile.rawPathCount, profile.storedCellCount)
        << std::endl;
    std::cout << "Visited ratio: "
        << SafeRatio(
            static_cast<std::size_t>(profile.astarVisitedCount),
            profile.storedCellCount)
        << std::endl;
    std::cout << "Candidate triangle ratio: "
        << SafeRatio(profile.candidateTriangleCount, profile.triangleCount)
        << std::endl;
}
