#pragma once

#include "VoxelPathPlanner.h"

#include <BRepPrimAPI_MakeSphere.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

inline bool Expect(
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

inline TopoDS_Shape MakeSmokeSphere(double radius)
{
    return BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), radius).Shape();
}

inline VoxelPlanningScenario MakeLocalShortPathScenario(
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

inline VoxelPlanningScenario MakePoleToPoleScenario(
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

inline VoxelPathPlannerOptions MakeSmokeOptions()
{
    VoxelPathPlannerOptions options =
        VoxelPathPlanner::MakeDefaultOptions();

    options.runOptions.exportVtk = false;
    options.runOptions.debugNeighborhood = false;
    options.runOptions.verbose = false;

    return options;
}

inline VoxelPathPlannerOptions MakeLocalBuildSmokeOptions()
{
    VoxelPathPlannerOptions options = MakeSmokeOptions();
    options.localBuildOptions.regionMode = VoxelBuildRegionMode::StartGoalBox;
    return options;
}

inline VoxelPathPlannerOptions MakeLazySmokeOptions()
{
    VoxelPathPlannerOptions options = MakeSmokeOptions();
    options.lazyBuildOptions.enabled = true;
    options.lazyBuildOptions.chunkCacheOptions.chunkVoxelSize = 16;
    options.lazyBuildOptions.chunkCacheOptions.buildPadding =
        options.meshBuildOptions.clearance +
        0.5 * std::sqrt(3.0) * options.meshBuildOptions.voxelSize;
    options.lazyBuildOptions.maxChunkBuildCount = 128;
    options.lazyBuildOptions.fallbackPolicy =
        VoxelLazyFallbackPolicy::FullMeshBoundsOnFailure;
    return options;
}

inline MeshAABB ComputeTriangleAABBForTest(
    const MeshTriangle& tri)
{
    MeshAABB box;
    box.minP = Vec(
        std::min({ tri.p0.x, tri.p1.x, tri.p2.x }),
        std::min({ tri.p0.y, tri.p1.y, tri.p2.y }),
        std::min({ tri.p0.z, tri.p1.z, tri.p2.z })
    );
    box.maxP = Vec(
        std::max({ tri.p0.x, tri.p1.x, tri.p2.x }),
        std::max({ tri.p0.y, tri.p1.y, tri.p2.y }),
        std::max({ tri.p0.z, tri.p1.z, tri.p2.z })
    );
    return box;
}

inline bool IntersectsAABBForTest(
    const MeshAABB& a,
    const MeshAABB& b)
{
    return a.minP.x <= b.maxP.x && a.maxP.x >= b.minP.x &&
        a.minP.y <= b.maxP.y && a.maxP.y >= b.minP.y &&
        a.minP.z <= b.maxP.z && a.maxP.z >= b.minP.z;
}

inline std::vector<MeshTriangle> MakeSeparatedTriangles()
{
    return {
        { Vec(0, 0, 0), Vec(4, 0, 0), Vec(0, 4, 0) },
        { Vec(20, 0, 0), Vec(24, 0, 0), Vec(20, 4, 0) },
        { Vec(0, 20, 0), Vec(4, 20, 0), Vec(0, 24, 0) }
    };
}
