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

VoxelPathPlannerOptions MakeSmokeOptions()
{
    VoxelPathPlannerOptions options =
        VoxelPathPlanner::MakeDefaultOptions();

    options.runOptions.exportVtk = false;
    options.runOptions.debugNeighborhood = false;
    options.runOptions.verbose = false;

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

    const VoxelPlanningScenario scenario =
        MakeLocalShortPathScenario(sphere, radius);

    VoxelPathPlannerOptions defaultOptions = MakeSmokeOptions();
    const VoxelPathPlannerResult defaultResult =
        VoxelPathPlanner::Plan(scenario, defaultOptions);

    bool ok = true;
    ok &= Expect(defaultResult.success, "default plan should succeed");
    ok &= Expect(
        defaultResult.profile.astarSucceeded,
        "default A* should succeed");
    ok &= Expect(
        defaultResult.profile.rawPathCount > 0,
        "default path should contain voxels");
    ok &= Expect(
        defaultResult.profile.candidateTriangleCount > 0,
        "default candidate triangle count should be positive");
    ok &= Expect(
        defaultResult.profile.candidateTriangleCount <
            defaultResult.profile.triangleCount,
        "local build should reduce candidate triangle count");

    VoxelPathPlannerOptions noopHookOptions = MakeSmokeOptions();
    int ensureCallCount = 0;
    noopHookOptions.astarOptions.ensureCellBuilt =
        [&ensureCallCount](VoxelSpace&, const VoxelIndex&)
        {
            ++ensureCallCount;
        };

    const VoxelPathPlannerResult noopHookResult =
        VoxelPathPlanner::Plan(scenario, noopHookOptions);

    ok &= Expect(noopHookResult.success, "no-op hook plan should succeed");
    ok &= Expect(ensureCallCount > 0, "ensure hook should be exercised");
    ok &= Expect(
        noopHookResult.profile.rawPathCount ==
            defaultResult.profile.rawPathCount,
        "no-op ensure hook should not change raw path count");
    ok &= Expect(
        noopHookResult.profile.optimizedPathCount ==
            defaultResult.profile.optimizedPathCount,
        "no-op ensure hook should not change optimized path count");
    ok &= Expect(
        std::abs(
            noopHookResult.profile.totalCost -
            defaultResult.profile.totalCost) < 1.0e-9,
        "no-op ensure hook should not change total cost");

    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
