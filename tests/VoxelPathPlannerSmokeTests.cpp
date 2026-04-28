#include "VoxelPathPlanner.h"

#include <BRepPrimAPI_MakeSphere.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{
VoxelPlanningScenario MakeLocalShortPathScenario(
    const TopoDS_Shape& sphere,
    double radius)
{
    const double localGoalZ =
        std::sqrt(radius * radius - 10.0 * 10.0);

    VoxelPlanningScenario scenario;
    scenario.name = "smoke_sphere_local_short_path";
    scenario.shape = sphere;
    scenario.startPoint = gp_Pnt(0, 0, radius);
    scenario.startDir = gp_Vec(0, 0, 1);
    scenario.goalPoint = gp_Pnt(10, 0, localGoalZ);
    scenario.goalDir = gp_Vec(10, 0, localGoalZ);

    return scenario;
}

VoxelPlanningScenario MakePoleToPoleScenario(
    const TopoDS_Shape& sphere,
    double radius)
{
    VoxelPlanningScenario scenario;
    scenario.name = "smoke_sphere_pole_to_pole";
    scenario.shape = sphere;
    scenario.startPoint = gp_Pnt(0, 0, -radius);
    scenario.startDir = gp_Vec(0, 0, -1);
    scenario.goalPoint = gp_Pnt(0, 0, radius);
    scenario.goalDir = gp_Vec(0, 0, 1);

    return scenario;
}

VoxelPathPlannerOptions MakeSmokeOptions()
{
    VoxelPathPlannerOptions options =
        VoxelPathPlanner::MakeDefaultOptions();

    options.runOptions.exportVtk = false;
    options.runOptions.debugNeighborhood = false;
    options.runOptions.verbose = false;

    return options;
}

VoxelPathPlannerOptions MakeLocalBuildSmokeOptions()
{
    VoxelPathPlannerOptions options = MakeSmokeOptions();
    options.localBuildOptions.regionMode = VoxelBuildRegionMode::StartGoalBox;
    return options;
}

bool Expect(
    bool condition,
    const char* message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << std::endl;
        return false;
    }

    return true;
}
}

int main()
{
    const double radius = 50.0;
    TopoDS_Shape sphere =
        BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), radius).Shape();

    const VoxelPlanningScenario localScenario =
        MakeLocalShortPathScenario(sphere, radius);

    VoxelPathPlannerOptions localOptions = MakeLocalBuildSmokeOptions();
    const VoxelPathPlannerResult localResult =
        VoxelPathPlanner::Plan(localScenario, localOptions);

    bool ok = true;
    ok &= Expect(localResult.success, "local plan should succeed");
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

    VoxelPathPlannerOptions noopHookOptions = MakeLocalBuildSmokeOptions();
    int ensureCallCount = 0;
    noopHookOptions.astarOptions.ensureCellBuilt =
        [&ensureCallCount](VoxelSpace&, const VoxelIndex&)
        {
            ++ensureCallCount;
        };

    const VoxelPathPlannerResult noopHookResult =
        VoxelPathPlanner::Plan(localScenario, noopHookOptions);

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

    const VoxelPlanningScenario poleScenario =
        MakePoleToPoleScenario(sphere, radius);

    const VoxelPathPlannerResult fullPoleResult =
        VoxelPathPlanner::Plan(poleScenario, MakeSmokeOptions());

    VoxelPathPlannerOptions localPoleOptions = MakeLocalBuildSmokeOptions();
    const VoxelPathPlannerResult localPoleResult =
        VoxelPathPlanner::Plan(poleScenario, localPoleOptions);

    ok &= Expect(
        fullPoleResult.success,
        "full-bounds pole-to-pole plan should succeed");
    ok &= Expect(
        localPoleResult.success,
        "local pole-to-pole plan should still succeed for comparison");
    ok &= Expect(
        fullPoleResult.profile.buildRegionMode == "FullMeshBounds",
        "default build mode should be full mesh bounds");
    ok &= Expect(
        fullPoleResult.profile.totalCost < localPoleResult.profile.totalCost,
        "full-bounds pole-to-pole path should avoid local-box clipping");

    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
