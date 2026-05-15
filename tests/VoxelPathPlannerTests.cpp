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

bool IsNear(
    const Vec& p,
    const Vec& q,
    double tolerance = 1.0e-9)
{
    return std::abs(p.x - q.x) <= tolerance &&
        std::abs(p.y - q.y) <= tolerance &&
        std::abs(p.z - q.z) <= tolerance;
}

bool TestBaselinePlannerDefaultBehavior(
    const VoxelPlanningScenario& scenario,
    VoxelPathPlannerResult& baselineResult)
{
    VoxelPathPlannerOptions baselineOptions = MakeBaselineSmokeOptions();
    bool ok = true;
    ok &= Expect(
        !baselineOptions.lazyBuildOptions.enabled,
        "lazy build should be disabled by default");
    ok &= Expect(
        baselineOptions.lazyBuildOptions.fallbackPolicy ==
            VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure,
        "lazy fallback policy should default to full-bounds fallback");

    baselineResult = VoxelPathPlanner::Plan(scenario, baselineOptions);

    ok &= Expect(baselineResult.success, "baseline plan should succeed");
    ok &= Expect(
        !baselineResult.profile.lazyBuildEnabled,
        "planner profile should report lazy disabled by default");
    ok &= Expect(
        baselineResult.profile.lazyVoxelQueryCount == 0,
        "default planner should not query lazy voxels");
    ok &= Expect(
        !baselineResult.profile.lazyFallbackTriggered,
        "default planner should not trigger lazy fallback");
    ok &= Expect(
        baselineResult.profile.astarSucceeded,
        "baseline A* should succeed");
    ok &= Expect(
        baselineResult.profile.rawPathCount > 0,
        "baseline path should contain voxels");
    ok &= Expect(
        baselineResult.profile.candidateTriangleCount > 0,
        "baseline candidate triangle count should be positive");
    ok &= Expect(
        baselineResult.profile.buildRegionMode == "FullMeshBounds",
        "default build mode should be full mesh bounds");
    ok &= Expect(
        baselineResult.optimizeResult.voxelPath.size() ==
            baselineResult.profile.optimizedPathCount,
        "planner result should expose optimized voxel path");
    ok &= Expect(
        baselineResult.hasFinalSearchBounds,
        "planner result should expose final search bounds");
    ok &= Expect(
        !baselineResult.astarResult.pointPath.empty() &&
            IsNear(baselineResult.astarResult.pointPath.front(),
                scenario.startPoint) &&
            IsNear(baselineResult.astarResult.pointPath.back(),
                scenario.goalPoint),
        "A* point path should preserve API start and goal points");
    ok &= Expect(
        !baselineResult.optimizeResult.pointPath.empty() &&
            IsNear(baselineResult.optimizeResult.pointPath.front(),
                scenario.startPoint) &&
            IsNear(baselineResult.optimizeResult.pointPath.back(),
                scenario.goalPoint),
        "optimized point path should preserve API start and goal points");

    return ok;
}

bool TestNoopEnsureHookDoesNotChangePath(
    const VoxelPlanningScenario& scenario,
    const VoxelPathPlannerResult& baselineResult)
{
    VoxelPathPlannerOptions noopHookOptions = MakeBaselineSmokeOptions();
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
            baselineResult.profile.rawPathCount,
        "no-op ensure hook should not change raw path count");
    ok &= Expect(
        noopHookResult.profile.optimizedPathCount ==
            baselineResult.profile.optimizedPathCount,
        "no-op ensure hook should not change optimized path count");
    ok &= Expect(
        std::abs(
            noopHookResult.profile.totalCost -
            baselineResult.profile.totalCost) < 1.0e-9,
        "no-op ensure hook should not change total cost");

    return ok;
}

bool TestFullBoundsBoxPath(
    const VoxelPlanningScenario& scenario)
{
    const VoxelPathPlannerResult fullResult =
        VoxelPathPlanner::Plan(scenario, MakeSmokeOptions());

    bool ok = true;
    ok &= Expect(
        fullResult.success,
        "full-bounds box path plan should succeed");
    ok &= Expect(
        fullResult.profile.buildRegionMode == "FullMeshBounds",
        "default build mode should be full mesh bounds");
    ok &= Expect(
        fullResult.profile.totalCost > 0.0,
        "full-bounds box path should have positive cost");

    return ok;
}

bool TestLazyTimeoutReportsFailure(
    const VoxelPlanningScenario& scenario)
{
    VoxelPathPlannerOptions options = MakeSmokeOptions();
    options.lazyBuildOptions.enabled = true;
    options.lazyBuildOptions.timeBudgetMs = 0.001;
    options.lazyBuildOptions.fallbackPolicy =
        VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure;

    const VoxelPathPlannerResult result =
        VoxelPathPlanner::Plan(scenario, options);

    bool ok = true;
    ok &= Expect(!result.success, "lazy timeout should report failure");
    ok &= Expect(
        result.executionMode == VoxelPlannerExecutionMode::VoxelLazy,
        "lazy timeout should stay in lazy mode");
    ok &= Expect(
        result.profile.lazyTimeoutTriggered,
        "lazy timeout should be explicit");
    ok &= Expect(
        result.profile.lazyFallbackReason == "timeout",
        "lazy timeout reason should be explicit");
    ok &= Expect(
        result.astarResult.failReason == VoxelAStarFailReason::Timeout,
        "lazy timeout should expose A* timeout fail reason");

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
        lazyResult.executionMode == VoxelPlannerExecutionMode::VoxelLazy,
        "lazy planner should expose lazy execution mode");
    ok &= Expect(
        lazyResult.profile.lazyBuildEnabled,
        "lazy planner should report lazy enabled");
    ok &= Expect(
        !lazyResult.profile.lazyFallbackTriggered,
        "lazy planner should not fallback in success scenario");
    ok &= Expect(
        lazyResult.profile.buildRegionMode == "VoxelLazy",
        "lazy planner should report VoxelLazy build mode");
    ok &= Expect(
        lazyResult.profile.lazyVoxelQueryCount > 0,
        "lazy planner should query at least one voxel");
    ok &= Expect(
        lazyResult.profile.lazyCandidateTriangleCount > 0,
        "lazy planner should accumulate candidate triangles");
    ok &= Expect(
        lazyResult.profile.lazyDistanceCalculationCount > 0,
        "lazy planner should calculate non-intersecting triangle distances");
    ok &= Expect(
        lazyResult.profile.lazyTriangleVoxelIntersectTestCount > 0,
        "lazy planner should test triangle-voxel pairs");
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

bool TestPlannerLazyPathExport(
    const VoxelPlanningScenario& scenario)
{
    const std::string shapeMeshPath = "test_planner_lazy_shape_mesh.vtk";
    const std::string astarPathPath = "test_planner_lazy_astar_path.vtk";
    const std::string optimizedVoxelPath =
        "test_planner_lazy_optimized_path_voxels.vtk";
    const std::string optimizedPolylinePath =
        "test_planner_lazy_optimized_path_polyline.vtk";

    std::remove(shapeMeshPath.c_str());
    std::remove(astarPathPath.c_str());
    std::remove(optimizedVoxelPath.c_str());
    std::remove(optimizedPolylinePath.c_str());

    VoxelPathPlannerOptions options = MakeLazySmokeOptions();
    options.runOptions.exportVtk = true;
    options.runOptions.shapeMeshVtkPath = shapeMeshPath;
    options.runOptions.astarPathVtkPath = astarPathPath;
    options.runOptions.optimizedPathVoxelsVtkPath = optimizedVoxelPath;
    options.runOptions.optimizedPathPolylineVtkPath = optimizedPolylinePath;

    const VoxelPathPlannerResult result =
        VoxelPathPlanner::Plan(scenario, options);

    bool ok = true;
    ok &= Expect(result.success, "lazy export planner should succeed");
    ok &= Expect(
        result.executionMode == VoxelPlannerExecutionMode::VoxelLazy,
        "lazy export planner should stay in lazy mode");

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

    return ok;
}

bool TestBrepScenarioHelpersHandleMissingFile()
{
    VoxelBrepScenarioRequest request;
    request.name = "missing_brep";
    request.brepPath = "D:/this_file_should_not_exist_for_voxel_test.brep";
    request.startPoint = gp_Pnt(1.0, 2.0, 3.0);
    request.startDir = gp_Vec(1.0, 0.0, 0.0);
    request.goalPoint = gp_Pnt(4.0, 5.0, 6.0);
    request.goalDir = gp_Vec(0.0, 1.0, 0.0);

    VoxelPlanningScenario scenario;
    std::string errorMessage;

    bool ok = true;
    ok &= Expect(
        !VoxelPathPlanner::MakeScenarioFromBrepFile(
            request,
            scenario,
            &errorMessage),
        "missing BREP file should fail scenario construction");
    ok &= Expect(
        errorMessage.find(request.brepPath) != std::string::npos,
        "missing BREP error should include file path");

    const VoxelPathPlannerOptions options =
        VoxelPathPlanner::MakeBrepBaselineOptions("D:/brep_helper_test");
    ok &= Expect(
        !options.lazyBuildOptions.enabled,
        "BREP helper options should disable lazy build");
    ok &= Expect(
        options.runOptions.shapeMeshVtkPath ==
            "D:/brep_helper_test_shape_mesh.vtk",
        "BREP helper options should derive VTK paths from prefix");

    return ok;
}

bool TestRestrictedHalfSpaceCanBlockSearch(
    const VoxelPlanningScenario& scenario)
{
    VoxelPathPlannerOptions options = MakeSmokeOptions();
    VoxelRestrictedHalfSpace halfSpace;
    halfSpace.point = Vec(0.0, 0.0, 0.0);
    halfSpace.normal = Vec(1.0, 0.0, 0.0);
    options.restrictedHalfSpaces.push_back(halfSpace);

    const VoxelPathPlannerResult result =
        VoxelPathPlanner::Plan(scenario, options);

    bool ok = true;
    ok &= Expect(
        !result.success,
        "restricted half-space should block a path whose goal is inside it");
    ok &= Expect(
        result.astarResult.failReason == VoxelAStarFailReason::SnapGoalFailed ||
            result.astarResult.failReason == VoxelAStarFailReason::OpenSetEmpty,
        "restricted half-space failure should be reported by A*");

    return ok;
}

bool TestFreeSpaceTreatsFreeAndClearanceBandEqually()
{
    bool ok = true;

    ok &= Expect(
        VoxelWalkability::IsStateWalkable(
            VoxelState::Free,
            VoxelAStarSearchMode::FreeSpace),
        "FreeSpace should allow Free voxels");
    ok &= Expect(
        VoxelWalkability::IsStateWalkable(
            VoxelState::ClearanceBand,
            VoxelAStarSearchMode::FreeSpace),
        "FreeSpace should allow ClearanceBand voxels");
    ok &= Expect(
        !VoxelWalkability::IsStateWalkable(
            VoxelState::Occupied,
            VoxelAStarSearchMode::FreeSpace),
        "FreeSpace should block Occupied voxels");
    ok &= Expect(
        !VoxelWalkability::IsStateWalkable(
            VoxelState::Free,
            VoxelAStarSearchMode::ClearanceBand),
        "ClearanceBand mode should still reject Free voxels");

    return ok;
}

bool TestClearanceBandIgnoresSecondClearanceFilter()
{
    VoxelSpace space(Vec(0.0, 0.0, 0.0), 1.0);
    space.SetSearchBounds(VoxelBounds{
        VoxelIndex(0, 0, 0),
        VoxelIndex(0, 0, 0)
    });

    VoxelCell& cell = space.GetOrCreateCell(VoxelIndex(0, 0, 0));
    cell.state = VoxelState::ClearanceBand;
    cell.distanceToSurface = 0.25;

    return Expect(
        VoxelWalkability::IsIndexWalkableWithMinDistance(
            space,
            VoxelIndex(0, 0, 0),
            VoxelAStarSearchMode::ClearanceBand,
            1.0),
        "ClearanceBand mode should not apply clearance a second time");
}

bool TestSmoothingAllowsOccupiedRealEndpoints()
{
    VoxelSpace space(Vec(0.0, 0.0, 0.0), 1.0);
    space.SetSearchBounds(VoxelBounds{
        VoxelIndex(0, 0, 0),
        VoxelIndex(6, 0, 0)
    });
    space.SetCellState(VoxelIndex(0, 0, 0), VoxelState::Occupied);
    space.SetCellState(VoxelIndex(6, 0, 0), VoxelState::Occupied);

    std::vector<VoxelIndex> path{
        VoxelIndex(1, 0, 0),
        VoxelIndex(2, 0, 0),
        VoxelIndex(3, 0, 0),
        VoxelIndex(4, 0, 0),
        VoxelIndex(5, 0, 0)
    };

    VoxelPathOptimizeOptions options;
    options.searchMode = VoxelAStarSearchMode::FreeSpace;
    options.enableCurveSmoothing = true;
    options.curveSamplesPerSegment = 4;
    options.useEndpointDirections = true;
    options.startDirection = Vec(1.0, 0.0, 0.0);
    options.goalDirection = Vec(1.0, 0.0, 0.0);
    options.useRealEndpointsForSmoothing = true;
    options.realStartPoint = Vec(0.2, 0.5, 0.5);
    options.realGoalPoint = Vec(6.2, 0.5, 0.5);
    options.endpointCollisionExemptRadius = 1.0;

    const VoxelPathOptimizeResult result =
        VoxelPathOptimizer::Optimize(space, path, options);

    bool ok = true;
    ok &= Expect(
        result.smoothingSucceeded,
        "smoothing should allow occupied samples near real endpoints");
    ok &= Expect(
        !result.displayPointPath.empty() &&
            IsNear(result.displayPointPath.front(), options.realStartPoint) &&
            IsNear(result.displayPointPath.back(), options.realGoalPoint),
        "smoothed display path should use real endpoints");

    return ok;
}

bool TestSmoothingEndpointGuideIgnoresOffsetSearchEndpoint()
{
    VoxelSpace space(Vec(0.0, 0.0, 0.0), 1.0);
    space.SetSearchBounds(VoxelBounds{
        VoxelIndex(0, 0, 0),
        VoxelIndex(5, 2, 0)
    });

    for (int x = 0; x <= 5; ++x)
    {
        for (int y = 0; y <= 2; ++y)
        {
            space.SetCellState(VoxelIndex(x, y, 0), VoxelState::Free);
        }
    }

    std::vector<VoxelIndex> path{
        VoxelIndex(0, 2, 0),
        VoxelIndex(2, 0, 0),
        VoxelIndex(4, 0, 0),
        VoxelIndex(5, 0, 0)
    };

    VoxelPathOptimizeOptions options;
    options.searchMode = VoxelAStarSearchMode::FreeSpace;
    options.enableCurveSmoothing = true;
    options.curveSamplesPerSegment = 8;
    options.displayCurveSamplesPerSegment = 8;
    options.useEndpointDirections = true;
    options.startDirection = Vec(1.0, 0.0, 0.0);
    options.goalDirection = Vec(1.0, 0.0, 0.0);
    options.useRealEndpointsForSmoothing = true;
    options.realStartPoint = Vec(0.5, 0.5, 0.5);
    options.realGoalPoint = Vec(5.5, 0.5, 0.5);
    options.endpointCollisionExemptRadius = 2.0;

    const VoxelPathOptimizeResult result =
        VoxelPathOptimizer::Optimize(space, path, options);

    bool ok = true;
    ok &= Expect(
        result.smoothingSucceeded,
        "smoothing should succeed with offset search endpoints");
    ok &= Expect(
        result.displayPointPath.size() > 2,
        "smoothed display path should contain curve samples");
    ok &= Expect(
        !result.displayPointPath.empty() &&
            IsNear(result.displayPointPath.front(), options.realStartPoint) &&
            IsNear(result.displayPointPath.back(), options.realGoalPoint),
        "smoothed display path should preserve real endpoints");

    if (result.displayPointPath.size() > 1)
    {
        const Vec firstStep(
            result.displayPointPath.front(),
            result.displayPointPath[1]);
        ok &= Expect(
            firstStep.x > 0.0 && std::abs(firstStep.y) < 1.0e-6,
            "smoothed path should leave the real start along startDirection");
    }

    return ok;
}

bool TestSmoothingDoesNotUseDenseAStarPathAsControlPath()
{
    VoxelSpace space(Vec(0.0, 0.0, 0.0), 1.0);
    space.SetSearchBounds(VoxelBounds{
        VoxelIndex(0, 0, 0),
        VoxelIndex(4, 4, 0)
    });

    std::vector<VoxelIndex> path{
        VoxelIndex(0, 0, 0),
        VoxelIndex(1, 0, 0),
        VoxelIndex(2, 0, 0),
        VoxelIndex(3, 0, 0),
        VoxelIndex(3, 1, 0),
        VoxelIndex(3, 2, 0),
        VoxelIndex(3, 3, 0)
    };

    for (const VoxelIndex& index : path)
    {
        space.SetCellState(index, VoxelState::Free);
    }

    VoxelPathOptimizeOptions options;
    options.searchMode = VoxelAStarSearchMode::FreeSpace;
    options.enableCurveSmoothing = true;
    options.curveSamplesPerSegment = 4;
    options.displayCurveSamplesPerSegment = 4;
    options.useEndpointDirections = true;
    options.startDirection = Vec(1.0, 0.0, 0.0);
    options.goalDirection = Vec(0.0, 1.0, 0.0);
    options.useRealEndpointsForSmoothing = true;
    options.realStartPoint = Vec(0.5, 0.5, 0.5);
    options.realGoalPoint = Vec(3.5, 3.5, 0.5);
    options.endpointCollisionExemptRadius = 1.0;

    const VoxelPathOptimizeResult result =
        VoxelPathOptimizer::Optimize(space, path, options);

    bool ok = true;
    ok &= Expect(
        result.smoothingSucceeded,
        "smoothing should succeed without using the dense raw A* path");
    ok &= Expect(
        !result.controlPointPath.empty() &&
            result.controlPointPath.size() < path.size(),
        "smoothing control path should not keep every raw A* point");
    ok &= Expect(
        result.pathOutputStage == "Path3 smoothed" ||
            result.pathOutputStage == "Path2 smoothed" ||
            result.pathOutputStage == "Path1 smoothed",
        "smoothed optimization should report which path source was smoothed");
    ok &= Expect(
        !result.smoothingMethod.empty() &&
            result.smoothingMethod != "None",
        "smoothed optimization should report which smoothing method was accepted");
    ok &= Expect(
        result.smoothingCandidateCount >=
            result.smoothingAcceptedCandidateCount,
        "smoothing diagnostics should count attempted and accepted candidates");

    return ok;
}

bool TestOptimizeWithoutSmoothingReturnsCollinearReducedPath()
{
    VoxelSpace space(Vec(0.0, 0.0, 0.0), 1.0);
    space.SetSearchBounds(VoxelBounds{
        VoxelIndex(0, 0, 0),
        VoxelIndex(2, 2, 0)
    });

    for (int x = 0; x <= 2; ++x)
    {
        for (int y = 0; y <= 2; ++y)
        {
            space.SetCellState(VoxelIndex(x, y, 0), VoxelState::Free);
        }
    }

    const std::vector<VoxelIndex> path1{
        VoxelIndex(0, 0, 0),
        VoxelIndex(1, 0, 0),
        VoxelIndex(2, 0, 0),
        VoxelIndex(2, 1, 0),
        VoxelIndex(2, 2, 0)
    };

    const std::vector<VoxelIndex> expectedPath2{
        VoxelIndex(0, 0, 0),
        VoxelIndex(2, 0, 0),
        VoxelIndex(2, 2, 0)
    };

    VoxelPathOptimizeOptions options;
    options.searchMode = VoxelAStarSearchMode::FreeSpace;
    options.removeCollinear = true;
    options.enableLineOfSightShortcut = true;
    options.enableCurveSmoothing = false;

    const VoxelPathOptimizeResult result =
        VoxelPathOptimizer::Optimize(space, path1, options);

    bool ok = true;
    ok &= Expect(
        result.voxelPath == expectedPath2,
        "non-smoothed optimization should output path2, not line-of-sight path3");
    ok &= Expect(
        result.lineCheckCount == 0,
        "line-of-sight shortcut should not run when Smooth Path is disabled");
    ok &= Expect(
        !result.smoothingSucceeded,
        "smoothing should be false when Smooth Path is disabled");
    ok &= Expect(
        result.pathOutputStage == "Path2",
        "non-smoothed optimization should report Path2 as the output stage");

    return ok;
}

bool TestSmoothingReportsPath2WhenShortcutDoesNotChangePath()
{
    VoxelSpace space(Vec(0.0, 0.0, 0.0), 1.0);
    space.SetSearchBounds(VoxelBounds{
        VoxelIndex(0, 0, 0),
        VoxelIndex(2, 0, 0)
    });

    for (int x = 0; x <= 2; ++x)
    {
        space.SetCellState(VoxelIndex(x, 0, 0), VoxelState::Free);
    }

    const std::vector<VoxelIndex> path1{
        VoxelIndex(0, 0, 0),
        VoxelIndex(1, 0, 0),
        VoxelIndex(2, 0, 0)
    };

    VoxelPathOptimizeOptions options;
    options.searchMode = VoxelAStarSearchMode::FreeSpace;
    options.removeCollinear = true;
    options.enableLineOfSightShortcut = true;
    options.enableCurveSmoothing = true;
    options.curveSamplesPerSegment = 4;
    options.useRealEndpointsForSmoothing = true;
    options.realStartPoint = Vec(0.5, 0.5, 0.5);
    options.realGoalPoint = Vec(2.5, 0.5, 0.5);
    options.endpointCollisionExemptRadius = 1.0;

    const VoxelPathOptimizeResult result =
        VoxelPathOptimizer::Optimize(space, path1, options);

    bool ok = true;
    ok &= Expect(
        result.smoothingSucceeded,
        "straight path smoothing should succeed");
    ok &= Expect(
        result.pathOutputStage == "Path2 smoothed",
        "smoothing should report Path2 when line-of-sight shortcut does not change path2");

    return ok;
}

bool TestSmoothingIgnoresMaxCurveDeviationAndPrefersCatmullRom()
{
    VoxelSpace space(Vec(0.0, 0.0, 0.0), 1.0);
    space.SetSearchBounds(VoxelBounds{
        VoxelIndex(0, 0, 0),
        VoxelIndex(4, 4, 0)
    });

    for (int x = 0; x <= 4; ++x)
    {
        for (int y = 0; y <= 4; ++y)
        {
            space.SetCellState(VoxelIndex(x, y, 0), VoxelState::Free);
        }
    }

    const std::vector<VoxelIndex> path{
        VoxelIndex(0, 0, 0),
        VoxelIndex(1, 0, 0),
        VoxelIndex(2, 1, 0),
        VoxelIndex(3, 3, 0),
        VoxelIndex(4, 4, 0)
    };

    VoxelPathOptimizeOptions options;
    options.searchMode = VoxelAStarSearchMode::FreeSpace;
    options.removeCollinear = false;
    options.enableLineOfSightShortcut = false;
    options.enableCurveSmoothing = true;
    options.curveSamplesPerSegment = 8;
    options.maxCurveDeviation = 1.0e-9;
    options.useRealEndpointsForSmoothing = true;
    options.realStartPoint = Vec(0.5, 0.5, 0.5);
    options.realGoalPoint = Vec(4.5, 4.5, 0.5);
    options.endpointCollisionExemptRadius = 1.0;

    const VoxelPathOptimizeResult result =
        VoxelPathOptimizer::Optimize(space, path, options);

    bool ok = true;
    ok &= Expect(
        result.smoothingSucceeded,
        "smoothing should succeed without maxCurveDeviation limiting the curve");
    ok &= Expect(
        result.smoothingMethod == "CatmullRom",
        "Catmull-Rom should be preferred when it passes walkability checks");
    ok &= Expect(
        result.catmullRomAccepted,
        "Catmull-Rom should be reported as accepted");

    return ok;
}

bool TestSmoothingRejectsEndpointDirectionMismatch()
{
    VoxelSpace space(Vec(0.0, 0.0, 0.0), 1.0);
    space.SetSearchBounds(VoxelBounds{
        VoxelIndex(0, 0, 0),
        VoxelIndex(1, 0, 0)
    });
    space.SetCellState(VoxelIndex(0, 0, 0), VoxelState::Free);
    space.SetCellState(VoxelIndex(1, 0, 0), VoxelState::Free);

    VoxelPathOptimizeOptions options;
    options.searchMode = VoxelAStarSearchMode::FreeSpace;
    options.useEndpointDirections = true;
    options.startDirection = Vec(0.0, 1.0, 0.0);
    options.goalDirection = Vec(1.0, 0.0, 0.0);
    options.minEndpointDirectionAlignment = 0.95;

    int lineCheckCount = 0;
    const std::vector<Vec> smoothed =
        VoxelPathOptimizer::SmoothPointPath(
            space,
            {
                Vec(0.5, 0.5, 0.5),
                Vec(1.5, 0.5, 0.5)
            },
            options,
            lineCheckCount);

    bool ok = true;
    ok &= Expect(
        smoothed.empty(),
        "two-point smoothing should reject endpoint direction mismatch");

    return ok;
}
}

int main()
{
    const VoxelPlanningScenario boxScenario = MakeBoxFaceScenario();

    bool ok = true;
    VoxelPathPlannerResult baselineResult;
    ok &= TestBaselinePlannerDefaultBehavior(boxScenario, baselineResult);
    ok &= TestNoopEnsureHookDoesNotChangePath(boxScenario, baselineResult);
    ok &= TestFullBoundsBoxPath(boxScenario);
    ok &= TestLazyTimeoutReportsFailure(boxScenario);
    ok &= TestPlannerLazySuccessWithoutFallback(boxScenario);
    ok &= TestPlannerLazyCostRegressionFallback(boxScenario);
    ok &= TestPlannerLazyPathExport(boxScenario);
    ok &= TestBrepScenarioHelpersHandleMissingFile();
    ok &= TestRestrictedHalfSpaceCanBlockSearch(boxScenario);
    ok &= TestFreeSpaceTreatsFreeAndClearanceBandEqually();
    ok &= TestClearanceBandIgnoresSecondClearanceFilter();
    ok &= TestSmoothingAllowsOccupiedRealEndpoints();
    ok &= TestSmoothingEndpointGuideIgnoresOffsetSearchEndpoint();
    ok &= TestSmoothingDoesNotUseDenseAStarPathAsControlPath();
    ok &= TestOptimizeWithoutSmoothingReturnsCollinearReducedPath();
    ok &= TestSmoothingReportsPath2WhenShortcutDoesNotChangePath();
    ok &= TestSmoothingIgnoresMaxCurveDeviationAndPrefersCatmullRom();
    ok &= TestSmoothingRejectsEndpointDirectionMismatch();

    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
