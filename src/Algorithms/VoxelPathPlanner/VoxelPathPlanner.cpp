#include "VoxelPathPlanner.h"

#include "LazyVoxelStateQuery.h"
#include "MeshVtkExporter.h"
#include "VoxelPathOptimizer.h"
#include "VoxelVtkExporter.h"

#include <BRep_Builder.hxx>
#include <BRepTools.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <utility>
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

bool IsCancelled(const VoxelPathPlannerOptions& options)
{
    return options.runOptions.shouldCancel &&
        options.runOptions.shouldCancel();
}

bool IsFiniteVec(const Vec& value)
{
    return std::isfinite(value.x) &&
        std::isfinite(value.y) &&
        std::isfinite(value.z);
}

bool AreValidRestrictedHalfSpaces(
    const std::vector<VoxelRestrictedHalfSpace>& halfSpaces)
{
    for (const VoxelRestrictedHalfSpace& halfSpace : halfSpaces)
    {
        if (!IsFiniteVec(halfSpace.point) ||
            !IsFiniteVec(halfSpace.normal) ||
            halfSpace.normal.SquareMagnitude() <= 1.0e-20)
        {
            return false;
        }
    }

    return true;
}

bool PrepareRestrictedHalfSpaces(
    const std::vector<VoxelRestrictedHalfSpace>& input,
    std::vector<VoxelRestrictedHalfSpace>& output)
{
    output = input;

    if (!AreValidRestrictedHalfSpaces(output))
    {
        return false;
    }

    for (VoxelRestrictedHalfSpace& halfSpace : output)
    {
        VoxelWalkability::CachePointNormalDot(halfSpace);
    }

    return true;
}

void CollectVoxelOverlayCenters(
    const VoxelSpace& voxelSpace,
    std::size_t maxPerState,
    const VoxelBounds* filterBounds,
    VoxelPathPlannerResult& result)
{
    result.occupiedVoxelCenters.clear();
    result.clearanceVoxelCenters.clear();

    for (const auto& kv : voxelSpace.Cells())
    {
        if (filterBounds != nullptr &&
            filterBounds->IsValid() &&
            !filterBounds->Contains(kv.first))
        {
            continue;
        }

        if (kv.second.state == VoxelState::Occupied &&
            (maxPerState == 0 ||
                result.occupiedVoxelCenters.size() < maxPerState))
        {
            result.occupiedVoxelCenters.push_back(
                voxelSpace.IndexToCenter(kv.first));
        }
        else if (kv.second.state == VoxelState::ClearanceBand &&
            (maxPerState == 0 ||
                result.clearanceVoxelCenters.size() < maxPerState))
        {
            result.clearanceVoxelCenters.push_back(
                voxelSpace.IndexToCenter(kv.first));
        }

        if (maxPerState > 0 &&
            result.occupiedVoxelCenters.size() >= maxPerState &&
            result.clearanceVoxelCenters.size() >= maxPerState)
        {
            break;
        }
    }
}

bool ComputePathBounds(
    const std::vector<VoxelIndex>& path,
    int padding,
    VoxelBounds& outBounds)
{
    if (path.empty())
    {
        return false;
    }

    outBounds.minIndex = path.front();
    outBounds.maxIndex = path.front();

    for (const VoxelIndex& index : path)
    {
        ExpandBoundsToInclude(outBounds, index);
    }

    ExpandBoundsByVoxelRadius(outBounds, padding);
    return outBounds.IsValid();
}

void CollectVoxelOverlayCentersIfRequested(
    const VoxelSpace& voxelSpace,
    const VoxelPathPlannerOptions& options,
    const std::vector<VoxelIndex>* path,
    VoxelPathPlannerResult& result)
{
    if (!options.runOptions.collectVoxelOverlayCenters)
    {
        return;
    }

    VoxelBounds pathBounds;
    const VoxelBounds* filterBounds = nullptr;

    if (path != nullptr &&
        ComputePathBounds(*path, 1, pathBounds))
    {
        filterBounds = &pathBounds;
    }

    CollectVoxelOverlayCenters(
        voxelSpace,
        options.runOptions.maxVoxelOverlayCentersPerState,
        filterBounds,
        result);
}

bool IsBroaderNeighborType(
    VoxelNeighborType candidate,
    VoxelNeighborType current)
{
    return static_cast<int>(candidate) > static_cast<int>(current);
}

bool ShouldRetryWithBroaderConnectivity(
    const VoxelAStarResult& result,
    const VoxelAStarOptions& options)
{
    return !result.success &&
        result.failReason == VoxelAStarFailReason::OpenSetEmpty &&
        options.searchMode == VoxelAStarSearchMode::ClearanceBand &&
        options.neighborType != VoxelNeighborType::FaceEdgeVertex26;
}

std::vector<std::pair<VoxelIndex, VoxelState>> CaptureVoxelStates(
    const VoxelSpace& voxelSpace)
{
    std::vector<std::pair<VoxelIndex, VoxelState>> states;
    states.reserve(voxelSpace.CellCount());

    for (const auto& kv : voxelSpace.Cells())
    {
        states.push_back({ kv.first, kv.second.state });
    }

    return states;
}

void RestoreVoxelStates(
    VoxelSpace& voxelSpace,
    const std::vector<std::pair<VoxelIndex, VoxelState>>& states)
{
    for (const auto& state : states)
    {
        voxelSpace.SetCellState(state.first, state.second);
    }
}

VoxelAStarResult SearchRestoringStateOnFailure(
    VoxelSpace& voxelSpace,
    const Vec& startPoint,
    const Vec& goalPoint,
    const VoxelAStarOptions& astarOptions)
{
    const std::vector<std::pair<VoxelIndex, VoxelState>> states =
        CaptureVoxelStates(voxelSpace);

    VoxelAStarResult result =
        VoxelAStar::Search(
            voxelSpace,
            startPoint,
            goalPoint,
            astarOptions
        );

    if (!result.success)
    {
        RestoreVoxelStates(voxelSpace, states);
    }

    return result;
}

VoxelAStarResult SearchWithConnectivityFallback(
    VoxelSpace& voxelSpace,
    const Vec& startPoint,
    const Vec& goalPoint,
    const VoxelAStarOptions& astarOptions,
    bool verbose,
    bool allowEndpointRaySnapRetry)
{
    VoxelAStarResult result =
        SearchRestoringStateOnFailure(
            voxelSpace,
            startPoint,
            goalPoint,
            astarOptions
        );

    if (result.success ||
        astarOptions.searchMode != VoxelAStarSearchMode::ClearanceBand)
    {
        return result;
    }

    if (ShouldRetryWithBroaderConnectivity(result, astarOptions))
    {
        const VoxelNeighborType fallbackTypes[] = {
            VoxelNeighborType::FaceEdge18,
            VoxelNeighborType::FaceEdgeVertex26
        };

        for (VoxelNeighborType fallbackType : fallbackTypes)
        {
            if (!IsBroaderNeighborType(fallbackType, astarOptions.neighborType))
            {
                continue;
            }

            VoxelAStarOptions retryOptions = astarOptions;
            retryOptions.neighborType = fallbackType;

            if (verbose)
            {
                std::cout
                    << "A* open set exhausted; retrying clearance-band search "
                    << "with broader neighbor connectivity."
                    << std::endl;
            }

            result =
                SearchRestoringStateOnFailure(
                    voxelSpace,
                    startPoint,
                    goalPoint,
                    retryOptions
                );

            if (result.success)
            {
                return result;
            }

            if (!ShouldRetryWithBroaderConnectivity(result, retryOptions))
            {
                break;
            }
        }
    }

    if (allowEndpointRaySnapRetry &&
        result.failReason == VoxelAStarFailReason::OpenSetEmpty)
    {
        VoxelAStarOptions raySnapOptions = astarOptions;
        raySnapOptions.neighborType = VoxelNeighborType::FaceEdgeVertex26;

        const bool canRaySnapStart =
            astarOptions.useStartSnapDirection &&
            voxelSpace.GetCellState(result.inputStartIndex) ==
                VoxelState::Occupied;
        const bool canRaySnapGoal =
            astarOptions.useGoalSnapDirection &&
            voxelSpace.GetCellState(result.inputGoalIndex) ==
                VoxelState::Occupied;

        raySnapOptions.forceStartSnapAlongDirection = canRaySnapStart;
        raySnapOptions.forceGoalSnapAlongDirection = canRaySnapGoal;

        if (!canRaySnapStart && !canRaySnapGoal)
        {
            return result;
        }

        if (verbose)
        {
            std::cout
                << "Clearance-band A* failed from an occupied endpoint; "
                << "retrying with ray-based directional endpoint snapping."
                << std::endl;
        }

        VoxelAStarResult raySnapResult =
            SearchRestoringStateOnFailure(
                voxelSpace,
                startPoint,
                goalPoint,
                raySnapOptions
            );

        if (raySnapResult.success)
        {
            return raySnapResult;
        }
    }

    return result;
}

void AddOriginalEndpointsToPath(
    std::vector<Vec>& points,
    const Vec& startPoint,
    const Vec& goalPoint)
{
    if (points.empty())
    {
        return;
    }

    constexpr double epsilon = 1.0e-9;

    if (points.front().Distance(startPoint) > epsilon)
    {
        points.insert(points.begin(), startPoint);
    }

    if (points.back().Distance(goalPoint) > epsilon)
    {
        points.push_back(goalPoint);
    }
}

void PreserveLazyAttemptOnFallback(
    const VoxelPlanningProfile& lazyProfile,
    VoxelPathPlannerResult& fallbackResult)
{
    fallbackResult.profile.lazyBuildEnabled = true;
    fallbackResult.profile.lazyFallbackTriggered = true;
    fallbackResult.profile.lazyFallbackReason =
        lazyProfile.lazyFallbackReason;
    fallbackResult.profile.lazyTimeoutTriggered =
        lazyProfile.lazyTimeoutTriggered;
    fallbackResult.profile.lazyTimeBudgetMs =
        lazyProfile.lazyTimeBudgetMs;
    fallbackResult.profile.lazyEnsureCallCount =
        lazyProfile.lazyEnsureCallCount;
    fallbackResult.profile.lazyVoxelQueryMs =
        lazyProfile.lazyVoxelQueryMs;
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
    fallbackResult.profile.lazyVoxelQueryCount =
        lazyProfile.lazyVoxelQueryCount;
    fallbackResult.profile.lazyTriangleVoxelIntersectTestCount =
        lazyProfile.lazyTriangleVoxelIntersectTestCount;
    fallbackResult.profile.lazyTriangleVoxelNoIntersectCacheHitCount =
        lazyProfile.lazyTriangleVoxelNoIntersectCacheHitCount;
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
    fallbackResult.profile.lazyDistanceNotImprovedCount =
        lazyProfile.lazyDistanceNotImprovedCount;
    fallbackResult.profile.lazyStateWriteCount =
        lazyProfile.lazyStateWriteCount;
    fallbackResult.profile.lazyStateUnchangedWriteCount =
        lazyProfile.lazyStateUnchangedWriteCount;
    fallbackResult.profile.lazyOccupiedUnchangedWriteCount =
        lazyProfile.lazyOccupiedUnchangedWriteCount;
    fallbackResult.profile.lazyClearanceUnchangedWriteCount =
        lazyProfile.lazyClearanceUnchangedWriteCount;
    fallbackResult.profile.lazyOccupiedWriteCount =
        lazyProfile.lazyOccupiedWriteCount;
    fallbackResult.profile.lazyClearanceWriteCount =
        lazyProfile.lazyClearanceWriteCount;
    fallbackResult.profile.lazyInfluenceCacheHitCount =
        lazyProfile.lazyInfluenceCacheHitCount;
    fallbackResult.profile.lazyInfluenceCacheMissCount =
        lazyProfile.lazyInfluenceCacheMissCount;
    fallbackResult.profile.lazyMinCandidateTriangleCount =
        lazyProfile.lazyMinCandidateTriangleCount;
    fallbackResult.profile.lazyMaxCandidateTriangleCount =
        lazyProfile.lazyMaxCandidateTriangleCount;
    fallbackResult.profile.lazyMinRawCandidateTriangleCount =
        lazyProfile.lazyMinRawCandidateTriangleCount;
    fallbackResult.profile.lazyMaxRawCandidateTriangleCount =
        lazyProfile.lazyMaxRawCandidateTriangleCount;
    fallbackResult.profile.lazyDryRunActiveCandidateTriangleCount =
        lazyProfile.lazyDryRunActiveCandidateTriangleCount;
    fallbackResult.profile.lazyDryRunInactiveCandidateTriangleCount =
        lazyProfile.lazyDryRunInactiveCandidateTriangleCount;
    fallbackResult.profile.lazyDryRunCandidateVoxelPairUpperBound =
        lazyProfile.lazyDryRunCandidateVoxelPairUpperBound;
    fallbackResult.profile.lazyDryRunClippedVoxelPairCount =
        lazyProfile.lazyDryRunClippedVoxelPairCount;
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

void CopyLazyQueryStatsToProfile(
    const LazyVoxelStateQueryStats& stats,
    VoxelPlanningProfile& profile);

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

void CopyLazyQueryStatsToProfile(
    const LazyVoxelStateQueryStats& stats,
    VoxelPlanningProfile& profile)
{
    profile.lazyEnsureCallCount = stats.ensureCallCount;
    profile.lazyCacheHitCount = stats.voxelCacheHitCount;
    profile.lazyVoxelQueryCount = stats.voxelQueryCount;
    profile.lazyCandidateTriangleCount = stats.candidateTriangleCount;
    profile.lazyRawCandidateTriangleCount = stats.rawCandidateTriangleCount;
    profile.lazyVoxelVisitCount = stats.voxelQueryCount;
    profile.lazyDistanceCalculationCount = stats.distanceCalculationCount;
    profile.lazyStateWriteCount =
        stats.occupiedWriteCount + stats.clearanceWriteCount +
        stats.freeWriteCount;
    profile.lazyOccupiedWriteCount = stats.occupiedWriteCount;
    profile.lazyClearanceWriteCount = stats.clearanceWriteCount;
    profile.lazyTriangleVoxelIntersectTestCount =
        stats.triangleVoxelIntersectTestCount;
    profile.lazyTriangleVoxelNoIntersectCacheHitCount =
        stats.triangleVoxelNoIntersectCacheHitCount;
    profile.lazyCandidateQueryMs = stats.candidateQueryMs;
    profile.lazyVoxelMarkMs = stats.voxelEvaluateMs;
    profile.lazyVoxelQueryMs = stats.voxelEvaluateMs;
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

    options.lazyBuildOptions.enabled = false;
    options.lazyBuildOptions.maxCostRegressionRatio = 0.0;
    options.lazyBuildOptions.timeBudgetMs = 3000.0;
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

VoxelPathPlannerOptions VoxelPathPlanner::MakeBrepBaselineOptions(
    const std::string& vtkPathPrefix)
{
    VoxelPathPlannerOptions options = MakeDefaultOptions();
    options.runOptions.exportVtk = true;
    options.runOptions.debugNeighborhood = true;
    options.runOptions.verbose = true;

    options.runOptions.shapeMeshVtkPath =
        vtkPathPrefix + "_shape_mesh.vtk";
    options.runOptions.astarFailedVtkPath =
        vtkPathPrefix + "_astar_failed.vtk";
    options.runOptions.astarPathVtkPath =
        vtkPathPrefix + "_astar_path.vtk";
    options.runOptions.optimizedPathVoxelsVtkPath =
        vtkPathPrefix + "_optimized_path_voxels.vtk";
    options.runOptions.optimizedPathPolylineVtkPath =
        vtkPathPrefix + "_optimized_path_polyline.vtk";

    options.lazyBuildOptions.enabled = false;
    return options;
}

bool VoxelPathPlanner::ReadBrepShape(
    const std::string& path,
    TopoDS_Shape& shape)
{
    BRep_Builder builder;

    if (!BRepTools::Read(shape, path.c_str(), builder))
    {
        return false;
    }

    return !shape.IsNull();
}

bool VoxelPathPlanner::MakeScenarioFromBrepFile(
    const VoxelBrepScenarioRequest& request,
    VoxelPlanningScenario& scenario,
    std::string* errorMessage)
{
    TopoDS_Shape shape;

    if (!ReadBrepShape(request.brepPath, shape))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = "Failed to read BREP file: " + request.brepPath;
        }
        return false;
    }

    scenario.name = request.name.empty() ? "user_brep" : request.name;
    scenario.shape = shape;
    scenario.startPoint = request.startPoint;
    scenario.startDir = request.startDir;
    scenario.goalPoint = request.goalPoint;
    scenario.goalDir = request.goalDir;

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }
    return true;
}

VoxelPathPlannerResult VoxelPathPlanner::Plan(
    const VoxelPlanningScenario& scenario,
    const VoxelPathPlannerOptions& options)
{
    VoxelPathPlannerResult result;
    VoxelPlanningProfile& profile = result.profile;
    profile.scenarioName = scenario.name;
    profile.buildRegionMode = "FullMeshBounds";
    profile.lazyBuildEnabled = options.lazyBuildOptions.enabled;
    result.executionMode = VoxelPlannerExecutionMode::FullMeshBounds;

    VoxelMeshBuildOptions meshBuildOptions = options.meshBuildOptions;
    meshBuildOptions.shouldCancel = options.runOptions.shouldCancel;

    std::vector<VoxelRestrictedHalfSpace> restrictedHalfSpaces;
    if (!PrepareRestrictedHalfSpaces(
            options.restrictedHalfSpaces,
            restrictedHalfSpaces))
    {
        if (options.runOptions.verbose)
        {
            std::cout << "Invalid restricted half-space options." << std::endl;
        }
        return result;
    }

    std::vector<MeshTriangle> triangles;

    if (IsCancelled(options))
    {
        return result;
    }

    if (!scenario.triangles.empty())
    {
        triangles = scenario.triangles;
    }
    else
    {
        ScopedTimer timer(profile.triangulationMs);

        if (!VoxelMeshBuilder::BuildShapeTriangulation(
                scenario.shape,
                meshBuildOptions.meshDeflection,
                meshBuildOptions.angularDeflection,
                triangles,
                options.runOptions.shouldCancel))
        {
            if (options.runOptions.verbose)
            {
                std::cout << "Build shape triangulation failed." << std::endl;
            }
            return result;
        }
    }

    profile.triangleCount = triangles.size();

    if (IsCancelled(options))
    {
        return result;
    }

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
        std::max(10.0, meshBuildOptions.voxelSize * 10.0);

    {
        ScopedTimer timer(profile.spatialIndexBuildMs);

        if (!spatialHash.Build(
                triangles,
                spatialHashOptions,
                options.runOptions.shouldCancel))
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
    astarOptions.shouldCancel = options.runOptions.shouldCancel;
    astarOptions.restrictedHalfSpaces = restrictedHalfSpaces;
    astarOptions.minTravelDistanceToSurface =
        astarOptions.searchMode == VoxelAStarSearchMode::ClearanceBand ?
            options.meshBuildOptions.clearance :
            0.0;
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
        if (IsCancelled(options))
        {
            return result;
        }

        profile.buildRegionMode = "VoxelLazy";
        result.executionMode = VoxelPlannerExecutionMode::VoxelLazy;

        VoxelSpace lazyVoxelSpace;
        VoxelBounds lazyBounds;

        if (!InitializeLazyVoxelSpace(
            triangles,
            meshBuildOptions,
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

            LazyVoxelStateQuery lazyQuery;

            if (!lazyQuery.Configure(
                    &triangles,
                    &spatialHash,
                    meshBuildOptions,
                    options.lazyBuildOptions.timeBudgetMs,
                    options.runOptions.shouldCancel))
            {
                profile.lazyFallbackTriggered = true;
                profile.lazyFallbackReason = "failed to configure lazy query";
            }
            else
            {
                lazyQuery.StartBudget();
                profile.lazyTimeBudgetMs =
                    options.lazyBuildOptions.timeBudgetMs;

                astarOptions.ensureCellBuilt =
                    [&lazyQuery](
                        VoxelSpace& space,
                        const VoxelIndex& index)
                    {
                        lazyQuery.EnsureVoxel(space, index);
                    };

                astarOptions.shouldCancel =
                    [&options, &lazyQuery]()
                    {
                        return IsCancelled(options) ||
                            lazyQuery.HasTimedOut();
                    };

                VoxelAStarResult lazyAStarResult;

                {
                    ScopedTimer timer(profile.astarMs);
                    lazyAStarResult =
                        SearchWithConnectivityFallback(
                            lazyVoxelSpace,
                            startPoint3D,
                            goalPoint3D,
                            astarOptions,
                            options.runOptions.verbose,
                            false
                        );
                }

                if (IsCancelled(options))
                {
                    return result;
                }

                CopyLazyQueryStatsToProfile(lazyQuery.GetStats(), profile);

                if (lazyQuery.HasTimedOut())
                {
                    profile.lazyTimeoutTriggered = true;
                    profile.lazyFallbackTriggered = false;
                    profile.lazyFallbackReason = "timeout";
                    lazyAStarResult.failReason =
                        VoxelAStarFailReason::Timeout;
                }

                profile.astarVisitedCount = lazyAStarResult.visitedCount;
                profile.rawPathCount = lazyAStarResult.voxelPath.size();
                profile.totalCost = lazyAStarResult.totalCost;
                profile.astarSucceeded = lazyAStarResult.success;
                AddOriginalEndpointsToPath(
                    lazyAStarResult.pointPath,
                    startPoint3D,
                    goalPoint3D);
                result.astarResult = lazyAStarResult;

                CountVoxelStates(
                    lazyVoxelSpace,
                    profile.occupiedCount,
                    profile.clearanceBandCount);
                profile.storedCellCount = lazyVoxelSpace.CellCount();

                if (lazyAStarResult.success && !lazyQuery.HasTimedOut())
                {
                    if (options.runOptions.exportVtk)
                    {
                        ScopedTimer timer(profile.vtkExportMs);
                        VoxelVtkExporter::ExportVoxelSpaceToVtk(
                            lazyVoxelSpace,
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
                    optOptions.minTravelDistanceToSurface =
                        astarOptions.searchMode ==
                            VoxelAStarSearchMode::ClearanceBand ?
                            options.meshBuildOptions.clearance :
                            0.0;
                    optOptions.restrictedHalfSpaces = restrictedHalfSpaces;
                    optOptions.removeCollinear = true;
                    optOptions.enableLineOfSightShortcut = true;
                    optOptions.maxShortcutLookAhead =
                        options.optimizerMaxShortcutLookAhead;
                    optOptions.enableCurveSmoothing =
                        options.smoothOptimizedPath;
                    optOptions.curveSamplesPerSegment =
                        options.smoothPathSamplesPerSegment;
                    optOptions.curveSampleSpacing =
                        options.smoothPathSampleSpacing;
                    optOptions.maxCurveDeviation =
                        options.smoothPathMaxDeviation;
                    optOptions.ensureCellBuilt =
                        [&lazyQuery](
                            VoxelSpace& space,
                            const VoxelIndex& index)
                        {
                            lazyQuery.EnsureVoxel(space, index);
                        };
                    optOptions.shouldCancel =
                        [&options, &lazyQuery]()
                        {
                            return IsCancelled(options) ||
                                lazyQuery.HasTimedOut();
                        };

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

                    if (IsCancelled(options))
                    {
                        return result;
                    }

                    if (lazyQuery.HasTimedOut())
                    {
                        CopyLazyQueryStatsToProfile(
                            lazyQuery.GetStats(),
                            profile);
                        profile.lazyTimeoutTriggered = true;
                        profile.lazyFallbackReason = "timeout";
                        result.success = false;
                        result.astarResult.failReason =
                            VoxelAStarFailReason::Timeout;
                        return result;
                    }

                    profile.optimizedPathCount = optResult.outputCount;
                    CopyLazyQueryStatsToProfile(
                        lazyQuery.GetStats(),
                        profile);
                    profile.smoothedPathPointCount =
                        optResult.smoothedPointCount;
                    profile.lineCheckCount = optResult.lineCheckCount;
                    profile.smoothingLineCheckCount =
                        optResult.smoothingLineCheckCount;
                    profile.smoothingRequested =
                        options.smoothOptimizedPath;
                    profile.smoothingSucceeded =
                        optResult.smoothingSucceeded;
                    AddOriginalEndpointsToPath(
                        optResult.pointPath,
                        startPoint3D,
                        goalPoint3D);
                    result.optimizeResult = optResult;
                    result.success = true;
                    result.lazyAttemptCost = lazyAStarResult.totalCost;

                    VoxelVtkExporter::MarkPathToVoxelSpace(
                        lazyVoxelSpace,
                        optResult.voxelPath
                    );
                    CollectVoxelOverlayCentersIfRequested(
                        lazyVoxelSpace,
                        options,
                        &optResult.voxelPath,
                        result);

                    if (options.runOptions.exportVtk)
                    {
                        ScopedTimer timer(profile.vtkExportMs);
                        VoxelVtkExporter::ExportVoxelSpaceToVtk(
                            lazyVoxelSpace,
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
                            optResult.pointPath,
                            options.runOptions.optimizedPathPolylineVtkPath
                        );
                    }

                    if (options.lazyBuildOptions.maxCostRegressionRatio > 0.0)
                    {
                        VoxelPathPlannerOptions baselineOptions = options;
                        baselineOptions.lazyBuildOptions.enabled = false;

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

                if (!profile.lazyFallbackTriggered &&
                    !profile.lazyTimeoutTriggered)
                {
                    profile.lazyFallbackTriggered = true;
                    profile.lazyFallbackReason = "lazy A* failed";
                }
            }
        }

        if (profile.lazyTimeoutTriggered)
        {
            return result;
        }

        if (options.lazyBuildOptions.fallbackPolicy ==
            VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure)
        {
            VoxelPathPlannerOptions fallbackOptions = options;
            fallbackOptions.lazyBuildOptions.enabled = false;

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

    constexpr int maxAttemptCount = 1;

    for (int attempt = 0; attempt < maxAttemptCount; ++attempt)
    {
        if (IsCancelled(options))
        {
            return result;
        }

        constexpr double searchPadding = 0.0;

        if (options.runOptions.verbose)
        {
            std::cout << "Build/search attempt: "
                << attempt + 1 << " / " << maxAttemptCount
                << ", mode: FullMeshBounds"
                << ", search padding: " << searchPadding
                << std::endl;
        }

        {
            ScopedTimer timer(profile.voxelBuildMs);
            buildResult =
                VoxelMeshBuilder::BuildVoxelSpaceFromTriangles(
                    triangles,
                    meshBuildOptions,
                    voxelSpace
                );
        }

        if (IsCancelled(options))
        {
            return result;
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
                SearchWithConnectivityFallback(
                    voxelSpace,
                    startPoint3D,
                    goalPoint3D,
                    astarOptions,
                    options.runOptions.verbose,
                    true
                );
        }

        if (IsCancelled(options))
        {
            return result;
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
    AddOriginalEndpointsToPath(
        result.astarResult.pointPath,
        startPoint3D,
        goalPoint3D);

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
        CollectVoxelOverlayCentersIfRequested(
            voxelSpace,
            options,
            astarResult.voxelPath.empty() ? nullptr : &astarResult.voxelPath,
            result);
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
    optOptions.minTravelDistanceToSurface =
        astarOptions.searchMode == VoxelAStarSearchMode::ClearanceBand ?
            options.meshBuildOptions.clearance :
            0.0;
    optOptions.restrictedHalfSpaces = restrictedHalfSpaces;
    optOptions.removeCollinear = true;
    optOptions.enableLineOfSightShortcut = true;
    optOptions.maxShortcutLookAhead = options.optimizerMaxShortcutLookAhead;
    optOptions.enableCurveSmoothing = options.smoothOptimizedPath;
    optOptions.curveSamplesPerSegment = options.smoothPathSamplesPerSegment;
    optOptions.curveSampleSpacing = options.smoothPathSampleSpacing;
    optOptions.maxCurveDeviation = options.smoothPathMaxDeviation;

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
    profile.smoothedPathPointCount = optResult.smoothedPointCount;
    profile.lineCheckCount = optResult.lineCheckCount;
    profile.smoothingLineCheckCount = optResult.smoothingLineCheckCount;
    profile.smoothingRequested = options.smoothOptimizedPath;
    profile.smoothingSucceeded = optResult.smoothingSucceeded;
    AddOriginalEndpointsToPath(
        optResult.pointPath,
        startPoint3D,
        goalPoint3D);
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
    CollectVoxelOverlayCentersIfRequested(
        voxelSpace,
        options,
        &optResult.voxelPath,
        result);

    if (options.runOptions.exportVtk)
    {
        ScopedTimer timer(profile.vtkExportMs);
        VoxelVtkExporter::ExportVoxelSpaceToVtk(
            voxelSpace,
            options.runOptions.optimizedPathVoxelsVtkPath,
            {
                VoxelState::Occupied,
                VoxelState::Path,
                VoxelState::Start,
                VoxelState::Goal,
                VoxelState::Path
            }
        );

        VoxelVtkExporter::ExportPathPolylineToVtk(
            optResult.pointPath,
            options.runOptions.optimizedPathPolylineVtkPath
        );
    }

    profile.storedCellCount = voxelSpace.CellCount();
    result.success = true;
    result.astarResult = astarResult;
    AddOriginalEndpointsToPath(
        result.astarResult.pointPath,
        startPoint3D,
        goalPoint3D);
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
    std::cout << "Lazy voxel query ms: "
        << profile.lazyVoxelQueryMs << std::endl;
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
    std::cout << "Lazy timeout triggered: "
        << (profile.lazyTimeoutTriggered ? "true" : "false") << std::endl;
    std::cout << "Lazy time budget ms: "
        << profile.lazyTimeBudgetMs << std::endl;
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
    std::cout << "Lazy voxel query count: "
        << profile.lazyVoxelQueryCount << std::endl;
    std::cout << "Lazy cache hit count: "
        << profile.lazyCacheHitCount << std::endl;
    std::cout << "Lazy failed build count: "
        << profile.lazyFailedBuildCount << std::endl;
    std::cout << "Lazy triangle-voxel intersect test count: "
        << profile.lazyTriangleVoxelIntersectTestCount << std::endl;
    std::cout << "Lazy triangle-voxel no-intersect cache hit count: "
        << profile.lazyTriangleVoxelNoIntersectCacheHitCount << std::endl;
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
    std::cout << "Lazy distance not improved count: "
        << profile.lazyDistanceNotImprovedCount << std::endl;
    std::cout << "Lazy state write count: "
        << profile.lazyStateWriteCount << std::endl;
    std::cout << "Lazy state unchanged write count: "
        << profile.lazyStateUnchangedWriteCount << std::endl;
    std::cout << "Lazy occupied unchanged write count: "
        << profile.lazyOccupiedUnchangedWriteCount << std::endl;
    std::cout << "Lazy clearance unchanged write count: "
        << profile.lazyClearanceUnchangedWriteCount << std::endl;
    std::cout << "Lazy occupied write count: "
        << profile.lazyOccupiedWriteCount << std::endl;
    std::cout << "Lazy clearance write count: "
        << profile.lazyClearanceWriteCount << std::endl;
    std::cout << "Lazy influence cache hit count: "
        << profile.lazyInfluenceCacheHitCount << std::endl;
    std::cout << "Lazy influence cache miss count: "
        << profile.lazyInfluenceCacheMissCount << std::endl;
    std::cout << "Lazy min candidate triangle count: "
        << profile.lazyMinCandidateTriangleCount << std::endl;
    std::cout << "Lazy max candidate triangle count: "
        << profile.lazyMaxCandidateTriangleCount << std::endl;
    std::cout << "Lazy min raw candidate triangle count: "
        << profile.lazyMinRawCandidateTriangleCount << std::endl;
    std::cout << "Lazy max raw candidate triangle count: "
        << profile.lazyMaxRawCandidateTriangleCount << std::endl;
    std::cout << "Lazy dry-run active candidate triangle count: "
        << profile.lazyDryRunActiveCandidateTriangleCount << std::endl;
    std::cout << "Lazy dry-run inactive candidate triangle count: "
        << profile.lazyDryRunInactiveCandidateTriangleCount << std::endl;
    std::cout << "Lazy dry-run candidate voxel pair upper bound: "
        << profile.lazyDryRunCandidateVoxelPairUpperBound << std::endl;
    std::cout << "Lazy dry-run clipped voxel pair count: "
        << profile.lazyDryRunClippedVoxelPairCount << std::endl;
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
