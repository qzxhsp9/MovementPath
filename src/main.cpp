#include "VoxelPathPlanner.h"

#include <BRepPrimAPI_MakeSphere.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
constexpr bool kRunUserBrepScenario = true;
constexpr const char* kUserBrepPath = R"(D:\primitives.brep)";

const gp_Pnt kUserStartPoint(6.0, 0.0, -9.0);
const gp_Pnt kUserGoalPoint(3, 0, 17);
const gp_Vec kUserStartSnapDirection(0.0, 0.0, -1.0);
const gp_Vec kUserGoalSnapDirection(0.0, 0.0, 1.0);

VoxelPlanningScenario MakePoleToPoleScenario(
    const TopoDS_Shape& sphere)
{
    VoxelPlanningScenario scenario;
    scenario.name = "sphere_pole_to_pole";
    scenario.shape = sphere;
    scenario.startPoint = gp_Pnt(0, 0, -50);
    scenario.startDir = gp_Vec(0, 0, -1);
    scenario.goalPoint = gp_Pnt(0, 0, 50);
    scenario.goalDir = gp_Vec(0, 0, 1);

    return scenario;
}

VoxelPlanningScenario MakeLocalShortPathScenario(
    const TopoDS_Shape& sphere,
    double radius)
{
    const double localGoalZ =
        std::sqrt(radius * radius - 10.0 * 10.0);

    VoxelPlanningScenario scenario;
    scenario.name = "sphere_local_short_path";
    scenario.shape = sphere;
    scenario.startPoint = gp_Pnt(0, 0, radius);
    scenario.startDir = gp_Vec(0, 0, 1);
    scenario.goalPoint = gp_Pnt(10, 0, localGoalZ);
    scenario.goalDir = gp_Vec(10, 0, localGoalZ);

    return scenario;
}

void RunScenario(
    const VoxelPlanningScenario& scenario,
    const VoxelPathPlannerOptions& options)
{
    std::cout << "===== Scenario: "
        << scenario.name << " =====" << std::endl;

    const VoxelPathPlannerResult result =
        VoxelPathPlanner::Plan(scenario, options);

    VoxelPathPlanner::PrintProfile(result.profile);
}

bool RunUserBrepScenario()
{
    VoxelBrepScenarioRequest request;
    request.name = "user_brep";
    request.brepPath = kUserBrepPath;
    request.startPoint = kUserStartPoint;
    request.startDir = kUserStartSnapDirection;
    request.goalPoint = kUserGoalPoint;
    request.goalDir = kUserGoalSnapDirection;

    VoxelPlanningScenario scenario;
    std::string errorMessage;

    if (!VoxelPathPlanner::MakeScenarioFromBrepFile(
            request,
            scenario,
            &errorMessage))
    {
        std::cerr << errorMessage << std::endl;
        return false;
    }

    const VoxelPathPlannerOptions options =
        VoxelPathPlanner::MakeBrepBaselineOptions("D:/user_brep");

    RunScenario(scenario, options);
    return true;
}
}

int main()
{
    if (kRunUserBrepScenario)
    {
        return RunUserBrepScenario() ? 0 : 1;
    }

    const double radius = 50.0;
    TopoDS_Shape sphere =
        BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), radius).Shape();

    VoxelPathPlannerOptions fullDebugOptions =
        VoxelPathPlanner::MakeDefaultOptions();
    fullDebugOptions.runOptions.exportVtk = true;
    fullDebugOptions.runOptions.debugNeighborhood = true;
    fullDebugOptions.runOptions.verbose = true;
    fullDebugOptions.lazyBuildOptions.enabled = true;

    RunScenario(
        MakePoleToPoleScenario(sphere),
        fullDebugOptions
    );

    VoxelPathPlannerOptions fastOptions =
        VoxelPathPlanner::MakeDefaultOptions();
    fastOptions.localBuildOptions.regionMode = VoxelBuildRegionMode::StartGoalBox;
    fastOptions.runOptions.exportVtk = true;
    fastOptions.runOptions.debugNeighborhood = false;
    fastOptions.runOptions.verbose = true;
    fastOptions.lazyBuildOptions.enabled = true;

    RunScenario(
        MakeLocalShortPathScenario(sphere, radius),
        fastOptions
    );

    return 0;
}
