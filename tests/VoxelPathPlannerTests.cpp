#include "TestCommon.h"

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace
{
bool IsNear(
    const Vec& p,
    const gp_Pnt& q,
    double tolerance = 1.0e-9)
{
    return std::abs(p.x - q.X()) <= tolerance &&
        std::abs(p.y - q.Y()) <= tolerance &&
        std::abs(p.z - q.Z()) <= tolerance;
}

bool TestLocalPlannerDefaultBehavior(
    const VoxelPlanningScenario& scenario,
    VoxelPathPlannerResult& localResult)
{
    VoxelPathPlannerOptions localOptions = MakeLocalBuildSmokeOptions();
    bool ok = true;
    ok &= Expect(
        !localOptions.lazyBuildOptions.enabled,
        "lazy build should be disabled by default");
    ok &= Expect(
        localOptions.lazyBuildOptions.fallbackPolicy ==
            VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure,
        "lazy fallback policy should default to full-bounds fallback");

    localResult = VoxelPathPlanner::Plan(scenario, localOptions);

    ok &= Expect(localResult.success, "local plan should succeed");
    ok &= Expect(
        !localResult.profile.lazyBuildEnabled,
        "planner profile should report lazy disabled by default");
    ok &= Expect(
        localResult.profile.lazyChunkBuildCount == 0,
        "default planner should not build lazy chunks");
    ok &= Expect(
        !localResult.profile.lazyFallbackTriggered,
        "default planner should not trigger lazy fallback");
    ok &= Expect(
        localResult.profile.astarSucceeded,
        "local A* should succeed");
    ok &= Expect(
        localResult.profile.rawPathCount > 0,
        "local path should contain voxels");
    ok &= Expect(
        localResult.profile.candidateTriangleCount > 0,
        "local candidate triangle count should be positive");
    ok &= Expect(
        localResult.profile.candidateTriangleCount <
            localResult.profile.triangleCount,
        "local build should reduce candidate triangle count");
    ok &= Expect(
        localResult.optimizeResult.voxelPath.size() ==
            localResult.profile.optimizedPathCount,
        "planner result should expose optimized voxel path");
    ok &= Expect(
        localResult.hasFinalSearchBounds,
        "planner result should expose final search bounds");
    ok &= Expect(
        !localResult.astarResult.pointPath.empty() &&
            IsNear(localResult.astarResult.pointPath.front(),
                scenario.startPoint) &&
            IsNear(localResult.astarResult.pointPath.back(),
                scenario.goalPoint),
        "A* point path should preserve API start and goal points");
    ok &= Expect(
        !localResult.optimizeResult.pointPath.empty() &&
            IsNear(localResult.optimizeResult.pointPath.front(),
                scenario.startPoint) &&
            IsNear(localResult.optimizeResult.pointPath.back(),
                scenario.goalPoint),
        "optimized point path should preserve API start and goal points");

    return ok;
}

bool TestNoopEnsureHookDoesNotChangePath(
    const VoxelPlanningScenario& scenario,
    const VoxelPathPlannerResult& localResult)
{
    VoxelPathPlannerOptions noopHookOptions = MakeLocalBuildSmokeOptions();
    int ensureCallCount = 0;
    noopHookOptions.astarOptions.ensureCellBuilt =
        [&ensureCallCount](VoxelSpace&, const VoxelIndex&)
        {
            ++ensureCallCount;
        };

    const VoxelPathPlannerResult noopHookResult =
        VoxelPathPlanner::Plan(scenario, noopHookOptions);

    bool ok = true;
    ok &= Expect(noopHookResult.success, "no-op hook plan should succeed");
    ok &= Expect(ensureCallCount > 0, "ensure hook should be exercised");
    ok &= Expect(
        noopHookResult.profile.rawPathCount ==
            localResult.profile.rawPathCount,
        "no-op ensure hook should not change raw path count");
    ok &= Expect(
        noopHookResult.profile.optimizedPathCount ==
            localResult.profile.optimizedPathCount,
        "no-op ensure hook should not change optimized path count");
    ok &= Expect(
        std::abs(
            noopHookResult.profile.totalCost -
            localResult.profile.totalCost) < 1.0e-9,
        "no-op ensure hook should not change total cost");

    return ok;
}

bool TestFullBoundsAvoidsStartGoalBoxClipping(
    const VoxelPlanningScenario& scenario)
{
    const VoxelPathPlannerResult fullResult =
        VoxelPathPlanner::Plan(scenario, MakeSmokeOptions());

    const VoxelPathPlannerResult localResult =
        VoxelPathPlanner::Plan(scenario, MakeLocalBuildSmokeOptions());

    bool ok = true;
    ok &= Expect(
        fullResult.success,
        "full-bounds pole-to-pole plan should succeed");
    ok &= Expect(
        localResult.success,
        "local pole-to-pole plan should still succeed for comparison");
    ok &= Expect(
        fullResult.profile.buildRegionMode == "FullMeshBounds",
        "default build mode should be full mesh bounds");
    ok &= Expect(
        fullResult.profile.totalCost < localResult.profile.totalCost,
        "full-bounds pole-to-pole path should avoid local-box clipping");

    return ok;
}

bool TestLazyGuardrailFallback(
    const VoxelPlanningScenario& scenario)
{
    VoxelPathPlannerOptions options = MakeSmokeOptions();
    options.lazyBuildOptions.enabled = true;
    options.lazyBuildOptions.maxChunkBuildCount = 1;
    options.lazyBuildOptions.fallbackPolicy =
        VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure;

    const VoxelPathPlannerResult result =
        VoxelPathPlanner::Plan(scenario, options);

    bool ok = true;
    ok &= Expect(result.success, "lazy guardrail fallback plan should succeed");
    ok &= Expect(
        result.profile.lazyBuildEnabled,
        "fallback result should preserve lazy enabled profile state");
    ok &= Expect(
        result.profile.lazyFallbackTriggered,
        "lazy guardrail should trigger fallback");
    ok &= Expect(
        result.profile.lazyChunkBuildCount <= 1,
        "lazy guardrail should limit chunk builds");
    ok &= Expect(
        result.profile.buildRegionMode == "FullMeshBounds",
        "lazy fallback should rerun full-bounds planning");
    ok &= Expect(
        result.fallbackExecutionMode == VoxelPlannerExecutionMode::FullMeshBounds,
        "lazy fallback should expose fallback execution mode");

    return ok;
}

bool TestPlannerLazySuccessWithoutFallback(
    const VoxelPlanningScenario& scenario)
{
    const VoxelPathPlannerResult baselineResult =
        VoxelPathPlanner::Plan(scenario, MakeSmokeOptions());

    const VoxelPathPlannerResult lazyResult =
        VoxelPathPlanner::Plan(scenario, MakeLazySmokeOptions());

    bool ok = true;
    ok &= Expect(baselineResult.success, "lazy baseline should succeed");
    ok &= Expect(lazyResult.success, "lazy planner should succeed");
    ok &= Expect(
        lazyResult.executionMode == VoxelPlannerExecutionMode::LazyChunks,
        "lazy planner should expose lazy execution mode");
    ok &= Expect(
        lazyResult.profile.lazyBuildEnabled,
        "lazy planner should report lazy enabled");
    ok &= Expect(
        !lazyResult.profile.lazyFallbackTriggered,
        "lazy planner should not fallback in success scenario");
    ok &= Expect(
        lazyResult.profile.buildRegionMode == "LazyChunks",
        "lazy planner should report LazyChunks build mode");
    ok &= Expect(
        lazyResult.profile.lazyChunkBuildCount > 0,
        "lazy planner should build at least one chunk");
    ok &= Expect(
        lazyResult.profile.lazyCandidateTriangleCount > 0,
        "lazy planner should accumulate candidate triangles");
    ok &= Expect(
        lazyResult.profile.lazyFailedBuildCount == 0,
        "lazy planner should not report failed chunk builds");
    ok &= Expect(
        lazyResult.profile.rawPathCount > 0,
        "lazy planner should produce raw path");
    ok &= Expect(
        lazyResult.optimizeResult.voxelPath.size() ==
            lazyResult.profile.optimizedPathCount,
        "lazy planner should expose optimized path");
    ok &= Expect(
        lazyResult.profile.totalCost <= baselineResult.profile.totalCost * 1.25,
        "lazy planner cost should stay close to full baseline");
    ok &= Expect(
        !lazyResult.astarResult.pointPath.empty() &&
            IsNear(lazyResult.astarResult.pointPath.front(),
                scenario.startPoint) &&
            IsNear(lazyResult.astarResult.pointPath.back(),
                scenario.goalPoint),
        "lazy A* point path should preserve API start and goal points");
    ok &= Expect(
        !lazyResult.optimizeResult.pointPath.empty() &&
            IsNear(lazyResult.optimizeResult.pointPath.front(),
                scenario.startPoint) &&
            IsNear(lazyResult.optimizeResult.pointPath.back(),
                scenario.goalPoint),
        "lazy optimized point path should preserve API start and goal points");

    return ok;
}

bool TestPlannerLazyCostRegressionFallback(
    const VoxelPlanningScenario& scenario)
{
    VoxelPathPlannerOptions options = MakeLazySmokeOptions();
    options.lazyBuildOptions.maxCostRegressionRatio = 0.1;
    options.lazyBuildOptions.fallbackPolicy =
        VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure;

    const VoxelPathPlannerResult result =
        VoxelPathPlanner::Plan(scenario, options);

    bool ok = true;
    ok &= Expect(result.success, "lazy cost fallback plan should succeed");
    ok &= Expect(
        result.executionMode == VoxelPlannerExecutionMode::FullMeshBounds,
        "cost fallback should return full-bounds execution mode");
    ok &= Expect(
        result.fallbackExecutionMode == VoxelPlannerExecutionMode::FullMeshBounds,
        "cost fallback should expose fallback execution mode");
    ok &= Expect(
        result.profile.lazyBuildEnabled,
        "cost fallback should preserve lazy enabled profile state");
    ok &= Expect(
        result.profile.lazyFallbackTriggered,
        "cost regression should trigger fallback");
    ok &= Expect(
        result.profile.lazyFallbackReason == "maxCostRegressionRatio exceeded",
        "cost fallback reason should be explicit");
    ok &= Expect(
        result.lazyAttemptCost > 0.0,
        "cost fallback should expose lazy attempt cost");
    ok &= Expect(
        result.fallbackCost > 0.0,
        "cost fallback should expose fallback cost");

    return ok;
}

bool TestPlannerLazyChunkBoundsExport(
    const VoxelPlanningScenario& scenario)
{
    const std::string shapeMeshPath = "test_planner_lazy_shape_mesh.vtk";
    const std::string astarPathPath = "test_planner_lazy_astar_path.vtk";
    const std::string optimizedVoxelPath =
        "test_planner_lazy_optimized_path_voxels.vtk";
    const std::string optimizedPolylinePath =
        "test_planner_lazy_optimized_path_polyline.vtk";
    const std::string chunkBoundsPath =
        "test_planner_lazy_chunk_bounds.vtk";

    std::remove(shapeMeshPath.c_str());
    std::remove(astarPathPath.c_str());
    std::remove(optimizedVoxelPath.c_str());
    std::remove(optimizedPolylinePath.c_str());
    std::remove(chunkBoundsPath.c_str());

    VoxelPathPlannerOptions options = MakeLazySmokeOptions();
    options.runOptions.exportVtk = true;
    options.runOptions.shapeMeshVtkPath = shapeMeshPath;
    options.runOptions.astarPathVtkPath = astarPathPath;
    options.runOptions.optimizedPathVoxelsVtkPath = optimizedVoxelPath;
    options.runOptions.optimizedPathPolylineVtkPath = optimizedPolylinePath;
    options.runOptions.lazyChunkBoundsVtkPath = chunkBoundsPath;

    const VoxelPathPlannerResult result =
        VoxelPathPlanner::Plan(scenario, options);

    bool ok = true;
    ok &= Expect(result.success, "lazy export planner should succeed");
    ok &= Expect(
        result.executionMode == VoxelPlannerExecutionMode::LazyChunks,
        "lazy export planner should stay in lazy mode");

    std::ifstream ifs(chunkBoundsPath.c_str(), std::ios::in);
    ok &= Expect(
        ifs.is_open(),
        "lazy planner should export chunk bounds VTK");

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string content = buffer.str();

    ok &= Expect(
        content.find("Lazy voxel chunk bounds") != std::string::npos,
        "lazy planner chunk bounds VTK should include title");
    ok &= Expect(
        content.find("SCALARS chunk_x int 1") != std::string::npos,
        "lazy planner chunk bounds VTK should include chunk indices");

    std::ifstream astarPathIfs(astarPathPath.c_str(), std::ios::in);
    ok &= Expect(
        astarPathIfs.is_open(),
        "lazy planner should export A* path voxels VTK");

    std::ifstream optimizedVoxelIfs(
        optimizedVoxelPath.c_str(),
        std::ios::in);
    ok &= Expect(
        optimizedVoxelIfs.is_open(),
        "lazy planner should export optimized path voxels VTK");

    std::ifstream optimizedPolylineIfs(
        optimizedPolylinePath.c_str(),
        std::ios::in);
    ok &= Expect(
        optimizedPolylineIfs.is_open(),
        "lazy planner should export optimized path polyline VTK");

    std::remove(shapeMeshPath.c_str());
    std::remove(astarPathPath.c_str());
    std::remove(optimizedVoxelPath.c_str());
    std::remove(optimizedPolylinePath.c_str());
    std::remove(chunkBoundsPath.c_str());

    return ok;
}
}

int main()
{
    const double radius = 50.0;
    const TopoDS_Shape sphere = MakeSmokeSphere(radius);
    const VoxelPlanningScenario localScenario =
        MakeLocalShortPathScenario(sphere, radius);
    const VoxelPlanningScenario poleScenario =
        MakePoleToPoleScenario(sphere, radius);

    bool ok = true;
    VoxelPathPlannerResult localResult;
    ok &= TestLocalPlannerDefaultBehavior(localScenario, localResult);
    ok &= TestNoopEnsureHookDoesNotChangePath(localScenario, localResult);
    ok &= TestFullBoundsAvoidsStartGoalBoxClipping(poleScenario);
    ok &= TestLazyGuardrailFallback(localScenario);
    ok &= TestPlannerLazySuccessWithoutFallback(localScenario);
    ok &= TestPlannerLazyCostRegressionFallback(localScenario);
    ok &= TestPlannerLazyChunkBoundsExport(localScenario);

    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
