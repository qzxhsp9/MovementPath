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
