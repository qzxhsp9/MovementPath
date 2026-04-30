#include "VoxelPathPlanner.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
struct BenchmarkCase
{
    // One scene/mode/options tuple. The benchmark runs these independently
    // instead of mutating one shared options object.
    std::string sceneName;
    std::string modeName;
    VoxelPlanningScenario scenario;
    VoxelPathPlannerOptions options;
};

struct BenchmarkRow
{
    // Flat CSV-oriented result. Keep this struct close to WriteCsvHeader()
    // so benchmark schema changes remain obvious in code review.
    std::string sceneName;
    std::string modeName;
    int runIndex = 0;
    int runCount = 0;
    bool success = false;
    std::string buildRegionMode;
    bool lazyFallbackTriggered = false;
    std::string lazyFallbackReason;

    double triangulationMs = 0.0;
    double spatialIndexBuildMs = 0.0;
    double voxelBuildMs = 0.0;
    double lazyChunkBuildMs = 0.0;
    double lazyCandidateQueryMs = 0.0;
    double lazyCandidateFilterMs = 0.0;
    double lazyVoxelMarkMs = 0.0;
    double lazyStateCountMs = 0.0;
    double astarMs = 0.0;
    double astarNonChunkMs = 0.0;
    double optimizeMs = 0.0;
    double totalMeasuredMs = 0.0;

    std::size_t triangleCount = 0;
    std::size_t candidateTriangleCount = 0;
    std::size_t rawCandidateTriangleCount = 0;
    std::size_t storedCellCount = 0;
    std::size_t occupiedCount = 0;
    std::size_t clearanceBandCount = 0;
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

    int astarVisitedCount = 0;
    std::size_t rawPathCount = 0;
    std::size_t optimizedPathCount = 0;
    double totalCost = 0.0;
    double lazyAttemptCost = 0.0;
    double fallbackCost = 0.0;
};

VoxelPathPlannerOptions MakeBenchmarkBaseOptions()
{
    VoxelPathPlannerOptions options =
        VoxelPathPlanner::MakeDefaultOptions();

    options.runOptions.exportVtk = false;
    options.runOptions.debugNeighborhood = false;
    options.runOptions.verbose = false;
    options.lazyBuildOptions.enabled = false;
    options.lazyBuildOptions.maxCostRegressionRatio = 0.0;

    return options;
}

VoxelPathPlannerOptions MakeFullOptions()
{
    VoxelPathPlannerOptions options = MakeBenchmarkBaseOptions();
    options.localBuildOptions.regionMode = VoxelBuildRegionMode::FullMeshBounds;
    return options;
}

VoxelPathPlannerOptions MakeLocalOptions()
{
    VoxelPathPlannerOptions options = MakeBenchmarkBaseOptions();
    options.localBuildOptions.regionMode = VoxelBuildRegionMode::StartGoalBox;
    return options;
}

VoxelPathPlannerOptions MakeLazyOptions()
{
    VoxelPathPlannerOptions options = MakeBenchmarkBaseOptions();
    options.lazyBuildOptions.enabled = true;
    options.lazyBuildOptions.chunkCacheOptions.chunkVoxelSize = 16;
    options.lazyBuildOptions.chunkCacheOptions.buildPadding =
        options.meshBuildOptions.clearance +
        0.5 * std::sqrt(3.0) * options.meshBuildOptions.voxelSize;
    options.lazyBuildOptions.maxChunkBuildCount = 0;
    options.lazyBuildOptions.maxCostRegressionRatio = 0.0;
    options.lazyBuildOptions.fallbackPolicy =
        VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure;
    // options.runOptions.exportVtk = true;
    return options;
}

VoxelPlanningScenario MakeSpherePoleScenario()
{
    const double radius = 50.0;

    VoxelPlanningScenario scenario;
    scenario.name = "sphere_pole_to_pole";
    scenario.shape =
        BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), radius).Shape();
    scenario.startPoint = gp_Pnt(0, 0, -radius);
    scenario.startDir = gp_Vec(0, 0, -1);
    scenario.goalPoint = gp_Pnt(0, 0, radius);
    scenario.goalDir = gp_Vec(0, 0, 1);

    return scenario;
}

VoxelPlanningScenario MakeBoxFaceScenario()
{
    VoxelPlanningScenario scenario;
    scenario.name = "box_long_face_to_face";
    scenario.shape =
        BRepPrimAPI_MakeBox(
            gp_Pnt(-30, -10, -10),
            gp_Pnt(30, 10, 10)).Shape();
    scenario.startPoint = gp_Pnt(-30, 0, 0);
    scenario.startDir = gp_Vec(-1, 0, 0);
    scenario.goalPoint = gp_Pnt(30, 0, 0);
    scenario.goalDir = gp_Vec(1, 0, 0);

    return scenario;
}

std::vector<BenchmarkCase> MakeBenchmarkCases()
{
    std::vector<BenchmarkCase> cases;
    const std::vector<VoxelPlanningScenario> scenarios = {
        MakeSpherePoleScenario(),
        MakeBoxFaceScenario()
    };

    for (const VoxelPlanningScenario& scenario : scenarios)
    {
        cases.push_back(
            { scenario.name, "full", scenario, MakeFullOptions() });
        cases.push_back(
            { scenario.name, "local", scenario, MakeLocalOptions() });
        cases.push_back(
            { scenario.name, "lazy", scenario, MakeLazyOptions() });
    }

    return cases;
}

BenchmarkRow MakeRow(
    const BenchmarkCase& benchmarkCase,
    const VoxelPathPlannerResult& result,
    int runIndex,
    int runCount)
{
    const VoxelPlanningProfile& profile = result.profile;

    BenchmarkRow row;
    row.sceneName = benchmarkCase.sceneName;
    row.modeName = benchmarkCase.modeName;
    row.runIndex = runIndex;
    row.runCount = runCount;
    row.success = result.success;
    row.buildRegionMode = profile.buildRegionMode;
    row.lazyFallbackTriggered = profile.lazyFallbackTriggered;
    row.lazyFallbackReason = profile.lazyFallbackReason;
    row.triangulationMs = profile.triangulationMs;
    row.spatialIndexBuildMs = profile.spatialIndexBuildMs;
    row.voxelBuildMs = profile.voxelBuildMs;
    row.lazyChunkBuildMs = profile.lazyChunkBuildMs;
    row.lazyCandidateQueryMs = profile.lazyCandidateQueryMs;
    row.lazyCandidateFilterMs = profile.lazyCandidateFilterMs;
    row.lazyVoxelMarkMs = profile.lazyVoxelMarkMs;
    row.lazyStateCountMs = profile.lazyStateCountMs;
    row.astarMs = profile.astarMs;
    row.astarNonChunkMs = profile.astarMs - profile.lazyChunkBuildMs;
    if (row.astarNonChunkMs < 0.0)
    {
        row.astarNonChunkMs = 0.0;
    }
    row.optimizeMs = profile.optimizeMs;
    row.totalMeasuredMs =
        profile.triangulationMs +
        profile.spatialIndexBuildMs +
        profile.voxelBuildMs +
        profile.astarMs +
        profile.optimizeMs;
    row.triangleCount = profile.triangleCount;
    row.candidateTriangleCount = profile.candidateTriangleCount;
    row.rawCandidateTriangleCount = profile.rawCandidateTriangleCount;
    row.storedCellCount = profile.storedCellCount;
    row.occupiedCount = profile.occupiedCount;
    row.clearanceBandCount = profile.clearanceBandCount;
    row.lazyEnsureCallCount = profile.lazyEnsureCallCount;
    row.lazyChunkBuildCount = profile.lazyChunkBuildCount;
    row.lazyCacheHitCount = profile.lazyCacheHitCount;
    row.lazyFailedBuildCount = profile.lazyFailedBuildCount;
    row.lazyCandidateTriangleCount =
        profile.lazyCandidateTriangleCount;
    row.lazyRawCandidateTriangleCount =
        profile.lazyRawCandidateTriangleCount;
    row.lazyVoxelVisitCount = profile.lazyVoxelVisitCount;
    row.lazyOutOfBoundsVoxelCount =
        profile.lazyOutOfBoundsVoxelCount;
    row.lazyDistanceCalculationCount =
        profile.lazyDistanceCalculationCount;
    row.lazyDistanceImprovedCount =
        profile.lazyDistanceImprovedCount;
    row.lazyDistanceNotImprovedCount =
        profile.lazyDistanceNotImprovedCount;
    row.lazyStateWriteCount = profile.lazyStateWriteCount;
    row.lazyStateUnchangedWriteCount =
        profile.lazyStateUnchangedWriteCount;
    row.lazyOccupiedUnchangedWriteCount =
        profile.lazyOccupiedUnchangedWriteCount;
    row.lazyClearanceUnchangedWriteCount =
        profile.lazyClearanceUnchangedWriteCount;
    row.lazyOccupiedWriteCount = profile.lazyOccupiedWriteCount;
    row.lazyClearanceWriteCount = profile.lazyClearanceWriteCount;
    row.lazyInfluenceCacheHitCount =
        profile.lazyInfluenceCacheHitCount;
    row.lazyInfluenceCacheMissCount =
        profile.lazyInfluenceCacheMissCount;
    row.lazyMinCandidateTriangleCount =
        profile.lazyMinCandidateTriangleCount;
    row.lazyMaxCandidateTriangleCount =
        profile.lazyMaxCandidateTriangleCount;
    row.lazyMinRawCandidateTriangleCount =
        profile.lazyMinRawCandidateTriangleCount;
    row.lazyMaxRawCandidateTriangleCount =
        profile.lazyMaxRawCandidateTriangleCount;
    row.astarVisitedCount = profile.astarVisitedCount;
    row.rawPathCount = profile.rawPathCount;
    row.optimizedPathCount = profile.optimizedPathCount;
    row.totalCost = profile.totalCost;
    row.lazyAttemptCost = result.lazyAttemptCost;
    row.fallbackCost = result.fallbackCost;

    return row;
}

void WriteCsvHeader(std::ostream& os)
{
    os
        << "scene,mode,runIndex,runCount,success,buildRegionMode,"
        << "lazyFallbackTriggered,"
        << "lazyFallbackReason,triangulationMs,spatialIndexBuildMs,"
        << "voxelBuildMs,astarMs,lazyChunkBuildMs,"
        << "lazyCandidateQueryMs,lazyCandidateFilterMs,"
        << "lazyVoxelMarkMs,lazyStateCountMs,astarNonChunkMs,"
        << "optimizeMs,totalMeasuredMs,"
        << "triangleCount,candidateTriangleCount,rawCandidateTriangleCount,"
        << "storedCellCount,occupiedCount,clearanceBandCount,"
        << "lazyEnsureCallCount,lazyChunkBuildCount,"
        << "lazyCacheHitCount,lazyFailedBuildCount,"
        << "lazyCandidateTriangleCount,lazyRawCandidateTriangleCount,"
        << "lazyVoxelVisitCount,lazyOutOfBoundsVoxelCount,"
        << "lazyDistanceCalculationCount,lazyDistanceImprovedCount,"
        << "lazyDistanceNotImprovedCount,"
        << "lazyStateWriteCount,lazyStateUnchangedWriteCount,"
        << "lazyOccupiedUnchangedWriteCount,"
        << "lazyClearanceUnchangedWriteCount,"
        << "lazyOccupiedWriteCount,lazyClearanceWriteCount,"
        << "lazyInfluenceCacheHitCount,lazyInfluenceCacheMissCount,"
        << "lazyMinCandidateTriangleCount,"
        << "lazyMaxCandidateTriangleCount,"
        << "lazyMinRawCandidateTriangleCount,"
        << "lazyMaxRawCandidateTriangleCount,"
        << "astarVisitedCount,rawPathCount,optimizedPathCount,totalCost,"
        << "lazyAttemptCost,fallbackCost\n";
}

void WriteCsvRow(
    std::ostream& os,
    const BenchmarkRow& row)
{
    os
        << row.sceneName << ","
        << row.modeName << ","
        << row.runIndex << ","
        << row.runCount << ","
        << (row.success ? "true" : "false") << ","
        << row.buildRegionMode << ","
        << (row.lazyFallbackTriggered ? "true" : "false") << ","
        << row.lazyFallbackReason << ","
        << row.triangulationMs << ","
        << row.spatialIndexBuildMs << ","
        << row.voxelBuildMs << ","
        << row.astarMs << ","
        << row.lazyChunkBuildMs << ","
        << row.lazyCandidateQueryMs << ","
        << row.lazyCandidateFilterMs << ","
        << row.lazyVoxelMarkMs << ","
        << row.lazyStateCountMs << ","
        << row.astarNonChunkMs << ","
        << row.optimizeMs << ","
        << row.totalMeasuredMs << ","
        << row.triangleCount << ","
        << row.candidateTriangleCount << ","
        << row.rawCandidateTriangleCount << ","
        << row.storedCellCount << ","
        << row.occupiedCount << ","
        << row.clearanceBandCount << ","
        << row.lazyEnsureCallCount << ","
        << row.lazyChunkBuildCount << ","
        << row.lazyCacheHitCount << ","
        << row.lazyFailedBuildCount << ","
        << row.lazyCandidateTriangleCount << ","
        << row.lazyRawCandidateTriangleCount << ","
        << row.lazyVoxelVisitCount << ","
        << row.lazyOutOfBoundsVoxelCount << ","
        << row.lazyDistanceCalculationCount << ","
        << row.lazyDistanceImprovedCount << ","
        << row.lazyDistanceNotImprovedCount << ","
        << row.lazyStateWriteCount << ","
        << row.lazyStateUnchangedWriteCount << ","
        << row.lazyOccupiedUnchangedWriteCount << ","
        << row.lazyClearanceUnchangedWriteCount << ","
        << row.lazyOccupiedWriteCount << ","
        << row.lazyClearanceWriteCount << ","
        << row.lazyInfluenceCacheHitCount << ","
        << row.lazyInfluenceCacheMissCount << ","
        << row.lazyMinCandidateTriangleCount << ","
        << row.lazyMaxCandidateTriangleCount << ","
        << row.lazyMinRawCandidateTriangleCount << ","
        << row.lazyMaxRawCandidateTriangleCount << ","
        << row.astarVisitedCount << ","
        << row.rawPathCount << ","
        << row.optimizedPathCount << ","
        << row.totalCost << ","
        << row.lazyAttemptCost << ","
        << row.fallbackCost << "\n";
}

int ParseRunCount(
    int argc,
    char** argv)
{
    int runCount = 3;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];

        if (arg == "--runs" && i + 1 < argc)
        {
            std::istringstream iss(argv[i + 1]);
            iss >> runCount;
            ++i;
        }
    }

    if (runCount < 1)
    {
        runCount = 1;
    }

    return runCount;
}
}

int main(
    int argc,
    char** argv)
{
    const int runCount = ParseRunCount(argc, argv);

    // Keep generated benchmark data with the planning documents.
    const std::string outputPath = "doc/voxel_planner_benchmark.csv";
    std::ofstream ofs(outputPath.c_str(), std::ios::out);
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open benchmark output: "
            << outputPath << std::endl;
        return EXIT_FAILURE;
    }

    ofs << std::fixed << std::setprecision(6);
    WriteCsvHeader(ofs);

    std::cout << "Writing benchmark CSV: " << outputPath << std::endl;
    std::cout << "Benchmark runs: " << runCount << std::endl;

    const std::vector<BenchmarkCase> benchmarkCases = MakeBenchmarkCases();

    for (int runIndex = 1; runIndex <= runCount; ++runIndex)
    {
        for (const BenchmarkCase& benchmarkCase : benchmarkCases)
        {
            const VoxelPathPlannerResult result =
                VoxelPathPlanner::Plan(
                    benchmarkCase.scenario,
                    benchmarkCase.options);

            const BenchmarkRow row =
                MakeRow(benchmarkCase, result, runIndex, runCount);
            WriteCsvRow(ofs, row);

            std::cout
                << "run=" << runIndex << "/" << runCount << " "
                << row.sceneName << " / " << row.modeName
                << " success=" << (row.success ? "true" : "false")
                << " cost=" << row.totalCost
                << " chunks=" << row.lazyChunkBuildCount
                << " measuredMs=" << row.totalMeasuredMs
                << std::endl;
        }
    }

    return EXIT_SUCCESS;
}
