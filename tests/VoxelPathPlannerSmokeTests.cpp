#include "VoxelPathPlanner.h"
#include "VoxelChunkCache.h"

#include <BRepPrimAPI_MakeSphere.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <vector>

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

MeshAABB ComputeTriangleAABB(
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

bool IntersectsAABB(
    const MeshAABB& a,
    const MeshAABB& b)
{
    return a.minP.x <= b.maxP.x && a.maxP.x >= b.minP.x &&
        a.minP.y <= b.maxP.y && a.maxP.y >= b.minP.y &&
        a.minP.z <= b.maxP.z && a.maxP.z >= b.minP.z;
}

std::vector<MeshTriangle> MakeSeparatedTriangles()
{
    return {
        { Vec(0, 0, 0), Vec(4, 0, 0), Vec(0, 4, 0) },
        { Vec(20, 0, 0), Vec(24, 0, 0), Vec(20, 4, 0) },
        { Vec(0, 20, 0), Vec(4, 20, 0), Vec(0, 24, 0) }
    };
}

bool TestTriangleSpatialHashCoversBruteForce()
{
    const std::vector<MeshTriangle> triangles = MakeSeparatedTriangles();

    TriangleSpatialHashOptions options;
    options.cellSize = 5.0;

    TriangleSpatialHash hash;
    bool ok = Expect(hash.Build(triangles, options), "hash should build");

    MeshAABB queryBox;
    queryBox.minP = Vec(18, -1, -1);
    queryBox.maxP = Vec(25, 5, 1);

    std::vector<int> hashIds;
    TriangleSpatialHashStats stats;
    hash.Query(queryBox, hashIds, stats);

    std::set<int> hashIdSet(hashIds.begin(), hashIds.end());

    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        if (IntersectsAABB(ComputeTriangleAABB(triangles[i]), queryBox))
        {
            ok &= Expect(
                hashIdSet.find(static_cast<int>(i)) != hashIdSet.end(),
                "hash query should contain every brute-force AABB hit");
        }
    }

    ok &= Expect(stats.queryCellCount > 0, "hash should report query cells");
    ok &= Expect(
        stats.queryUniqueTriangleCount == hashIds.size(),
        "hash stats should report unique query ids");

    return ok;
}

bool TestAppendVoxelSpaceExpandsBoundsAndPreservesState()
{
    std::vector<MeshTriangle> triangles;
    triangles.push_back(
        { Vec(0, 0, 0), Vec(10, 0, 0), Vec(0, 10, 0) }
    );

    VoxelMeshBuildOptions options;
    options.voxelSize = 1.0;
    options.clearance = 2.0;
    options.conservativeClearance = true;
    options.storeFreeCells = false;

    TriangleSpatialHashOptions hashOptions;
    hashOptions.cellSize = 4.0;
    TriangleSpatialHash hash;
    bool ok = Expect(hash.Build(triangles, hashOptions), "hash should build");

    MeshAABB baseBox;
    baseBox.minP = Vec(-1, -1, -1);
    baseBox.maxP = Vec(2, 2, 1);

    VoxelSpace space;
    const VoxelMeshBuildResult baseResult =
        VoxelMeshBuilder::BuildVoxelSpaceFromTrianglesInBox(
            triangles,
            &hash,
            baseBox,
            options,
            space
        );

    ok &= Expect(baseResult.success, "base voxel build should succeed");

    const VoxelIndex preservedIndex = space.WorldToIndex(Vec(0, 0, 0));
    const VoxelState preservedState = space.GetCellState(preservedIndex);

    const VoxelIndex appendOnlyIndex = space.WorldToIndex(Vec(5, 0, 0));
    ok &= Expect(
        !space.IsInsideSearchBounds(appendOnlyIndex),
        "append-only index should start outside base bounds");

    MeshAABB appendBox;
    appendBox.minP = Vec(4, -1, -1);
    appendBox.maxP = Vec(7, 2, 1);

    const VoxelMeshBuildResult appendResult =
        VoxelMeshBuilder::AppendVoxelSpaceFromTrianglesInBox(
            triangles,
            &hash,
            appendBox,
            options,
            space
        );

    ok &= Expect(appendResult.success, "append voxel build should succeed");
    ok &= Expect(
        space.IsInsideSearchBounds(appendOnlyIndex),
        "append should expand search bounds to include appended box");
    ok &= Expect(
        space.GetCellState(appendOnlyIndex) != VoxelState::Free,
        "append should populate voxels outside the original bounds");
    ok &= Expect(
        space.GetCellState(preservedIndex) == preservedState,
        "append should preserve existing voxel state");

    return ok;
}

bool TestVoxelChunkCacheBuildsEachChunkOnce()
{
    std::vector<MeshTriangle> triangles;
    triangles.push_back(
        { Vec(0, 0, 0), Vec(10, 0, 0), Vec(0, 10, 0) }
    );

    VoxelMeshBuildOptions buildOptions;
    buildOptions.voxelSize = 1.0;
    buildOptions.clearance = 2.0;
    buildOptions.conservativeClearance = true;

    TriangleSpatialHashOptions hashOptions;
    hashOptions.cellSize = 4.0;
    TriangleSpatialHash hash;

    bool ok = Expect(hash.Build(triangles, hashOptions), "hash should build");

    VoxelChunkCacheOptions cacheOptions;
    cacheOptions.chunkVoxelSize = 4;
    cacheOptions.buildPadding = 0.0;

    VoxelChunkCache cache;
    ok &= Expect(
        cache.Configure(
            &triangles,
            &hash,
            buildOptions,
            cacheOptions),
        "chunk cache should configure");

    VoxelSpace space(Vec(-1, -1, -1), buildOptions.voxelSize);
    const VoxelIndex targetIndex = space.WorldToIndex(Vec(1, 1, 0));

    ok &= Expect(
        cache.EnsureChunkForIndex(space, targetIndex),
        "chunk cache should build target chunk");

    const VoxelChunkCacheStats firstStats = cache.GetStats();
    ok &= Expect(
        firstStats.chunkBuildCount == 1,
        "first ensure should build one chunk");
    ok &= Expect(
        space.GetCellState(targetIndex) != VoxelState::Free,
        "built chunk should populate target voxel");

    ok &= Expect(
        cache.EnsureChunkForIndex(space, targetIndex),
        "second ensure should hit cache");

    const VoxelChunkCacheStats secondStats = cache.GetStats();
    ok &= Expect(
        secondStats.chunkBuildCount == 1,
        "cache hit should not rebuild chunk");
    ok &= Expect(
        secondStats.cacheHitCount == 1,
        "second ensure should increment cache hit count");

    return ok;
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
    ok &= Expect(
        localResult.optimizeResult.voxelPath.size() ==
            localResult.profile.optimizedPathCount,
        "planner result should expose optimized voxel path");
    ok &= Expect(
        localResult.hasFinalSearchBounds,
        "planner result should expose final search bounds");

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

    ok &= TestTriangleSpatialHashCoversBruteForce();
    ok &= TestAppendVoxelSpaceExpandsBoundsAndPreservesState();
    ok &= TestVoxelChunkCacheBuildsEachChunkOnce();

    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
