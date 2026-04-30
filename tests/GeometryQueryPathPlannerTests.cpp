#include "GeometryQueryPathPlanner.h"
#include "GeometryQueryContext.h"
#include "GeometryQueries.h"
#include "TriangleAabbTree.h"
#include "TriangleMesh.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <vector>

using namespace movement_path::geometry;

namespace
{
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

bool Near(
    double lhs,
    double rhs,
    double tolerance = 1.0e-9)
{
    return std::abs(lhs - rhs) <= tolerance;
}

bool NearVec(
    const Vec3& lhs,
    const Vec3& rhs,
    double tolerance = 1.0e-9)
{
    return Near(lhs.x, rhs.x, tolerance) &&
        Near(lhs.y, rhs.y, tolerance) &&
        Near(lhs.z, rhs.z, tolerance);
}

Triangle MakeUnitRightTriangle()
{
    return {
        Vec3(0.0, 0.0, 0.0),
        Vec3(1.0, 0.0, 0.0),
        Vec3(0.0, 1.0, 0.0)
    };
}

std::vector<Triangle> MakeSeparatedTriangles()
{
    return {
        {
            Vec3(0.0, 0.0, 0.0),
            Vec3(1.0, 0.0, 0.0),
            Vec3(0.0, 1.0, 0.0)
        },
        {
            Vec3(10.0, 0.0, 0.0),
            Vec3(11.0, 0.0, 0.0),
            Vec3(10.0, 1.0, 0.0)
        },
        {
            Vec3(0.0, 10.0, 0.0),
            Vec3(1.0, 10.0, 0.0),
            Vec3(0.0, 11.0, 0.0)
        }
    };
}

std::vector<Triangle> MakeGridTriangles(int count)
{
    std::vector<Triangle> triangles;
    triangles.reserve(static_cast<std::size_t>(count));

    for (int i = 0; i < count; ++i)
    {
        const double x = static_cast<double>(i * 3);
        triangles.push_back({
            Vec3(x, 0.0, 0.0),
            Vec3(x + 1.0, 0.0, 0.0),
            Vec3(x, 1.0, 0.0)
        });
    }

    return triangles;
}

bool TestPlannerScaffoldStatus()
{
    GeometryQueryPathPlanner planner;
    GeometryPathOptions options;

    GeometryPathRequest emptyRequest;
    GeometryPathResult emptyResult = planner.Plan(emptyRequest, options);

    bool ok = Expect(
        emptyResult.status == GeometryPathStatus::InvalidInput,
        "empty mesh should be invalid input");
    ok &= Expect(
        emptyResult.profile.triangleCount == 0,
        "empty mesh profile should report zero triangles");

    GeometryPathRequest request;
    request.triangles.push_back(MakeUnitRightTriangle());
    request.startPoint = Vec3(0.0, 0.0, 1.0);
    request.goalPoint = Vec3(1.0, 1.0, 1.0);

    GeometryPathResult result = planner.Plan(request, options);

    ok &= Expect(
        result.status == GeometryPathStatus::NotImplemented,
        "non-empty mesh should keep current scaffold status");
    ok &= Expect(
        result.profile.triangleCount == 1,
        "non-empty mesh profile should report triangle count");
    ok &= Expect(
        result.profile.spatialIndexNodeCount > 0,
        "non-empty mesh should build spatial index before scaffold exit");
    ok &= Expect(
        result.profile.spatialIndexBuildMs >= 0.0,
        "non-empty mesh should report spatial index build time");

    return ok;
}

bool TestPointToTriangleDistance()
{
    const Triangle tri = MakeUnitRightTriangle();
    bool ok = true;

    ok &= Expect(
        Near(DistancePointToTriangle(Vec3(0.25, 0.25, 1.0), tri), 1.0),
        "point above triangle face should project to face");
    ok &= Expect(
        NearVec(
            ClosestPointOnTriangle(Vec3(0.25, 0.25, 1.0), tri),
            Vec3(0.25, 0.25, 0.0)),
        "closest face point should preserve x/y");

    ok &= Expect(
        Near(DistancePointToTriangle(Vec3(0.5, -1.0, 0.0), tri), 1.0),
        "point outside edge should project to edge");
    ok &= Expect(
        NearVec(
            ClosestPointOnTriangle(Vec3(0.5, -1.0, 0.0), tri),
            Vec3(0.5, 0.0, 0.0)),
        "closest edge point should clamp to edge");

    ok &= Expect(
        Near(
            DistancePointToTriangle(Vec3(-1.0, -1.0, 0.0), tri),
            std::sqrt(2.0)),
        "point outside vertex should project to vertex");
    ok &= Expect(
        NearVec(
            ClosestPointOnTriangle(Vec3(-1.0, -1.0, 0.0), tri),
            Vec3(0.0, 0.0, 0.0)),
        "closest vertex point should be p0");

    const Triangle degenerate{
        Vec3(1.0, 2.0, 3.0),
        Vec3(1.0, 2.0, 3.0),
        Vec3(1.0, 2.0, 3.0)
    };

    ok &= Expect(
        Near(
            DistancePointToTriangle(Vec3(1.0, 2.0, 5.0), degenerate),
            2.0),
        "degenerate triangle should behave like a point");

    return ok;
}

bool TestBruteForceClosestPointToMesh()
{
    const std::vector<Triangle> triangles = MakeSeparatedTriangles();
    const ClosestPointResult result =
        ClosestPointToMeshBruteForce(Vec3(10.25, 0.25, 2.0), triangles);

    bool ok = Expect(result.hit, "brute-force closest query should hit mesh");
    ok &= Expect(result.triangleId == 1, "closest triangle should be second");
    ok &= Expect(Near(result.distance, 2.0), "closest distance should match");
    ok &= Expect(
        NearVec(result.closestPoint, Vec3(10.25, 0.25, 0.0)),
        "closest point should lie on second triangle");
    ok &= Expect(
        result.testedTriangleCount == triangles.size(),
        "brute-force query should test every triangle");

    const ClosestPointResult empty =
        ClosestPointToMeshBruteForce(Vec3(), std::vector<Triangle>());
    ok &= Expect(!empty.hit, "empty brute-force query should miss");

    return ok;
}

bool TestTriangleMeshBuildsBounds()
{
    TriangleMesh emptyMesh;
    bool ok = Expect(
        !emptyMesh.Build(std::vector<Triangle>()),
        "empty triangle mesh should not build as valid");
    ok &= Expect(!emptyMesh.IsValid(), "empty triangle mesh should be invalid");

    TriangleMesh mesh;
    const std::vector<Triangle> triangles = MakeSeparatedTriangles();
    ok &= Expect(mesh.Build(triangles), "triangle mesh should build");
    ok &= Expect(mesh.IsValid(), "triangle mesh should be valid");
    ok &= Expect(
        mesh.Triangles().size() == triangles.size(),
        "triangle mesh should retain triangles");
    ok &= Expect(
        NearVec(mesh.Bounds().min, Vec3(0.0, 0.0, 0.0)),
        "triangle mesh bounds min should match");
    ok &= Expect(
        NearVec(mesh.Bounds().max, Vec3(11.0, 11.0, 0.0)),
        "triangle mesh bounds max should match");

    return ok;
}

bool TestGeometryQueryContextAccumulatesProfile()
{
    const std::vector<Triangle> triangles = MakeSeparatedTriangles();

    GeometryQueryContext context;
    bool ok = Expect(context.Build(triangles), "query context should build");
    ok &= Expect(context.IsValid(), "query context should be valid");
    ok &= Expect(
        context.Profile().triangleCount == triangles.size(),
        "query context profile should report triangle count");
    ok &= Expect(
        context.Profile().spatialIndexNodeCount > 0,
        "query context profile should report index nodes");

    const ClosestPointResult closest =
        context.ClosestPoint(Vec3(10.25, 0.25, 2.0));
    ok &= Expect(closest.hit, "context closest query should hit");
    ok &= Expect(
        context.Profile().geometryQueryCount == 1,
        "context should count closest query as geometry query");
    ok &= Expect(
        context.Profile().collisionQueryCount == 0,
        "context should not count collision before segment query");
    ok &= Expect(
        context.Profile().geometryCandidateTriangleCount > 0,
        "context should accumulate candidate triangle count");
    ok &= Expect(
        context.Profile().maxCandidateTriangleCount > 0,
        "context should record max candidate triangle count");
    ok &= Expect(
        context.Profile().spatialIndexVisitedNodeCount > 0,
        "context should accumulate visited index nodes");

    const SegmentClearanceResult segment =
        context.SegmentClearance(
            Vec3(2.0, 2.0, 1.0),
            Vec3(8.0, 2.0, 1.0),
            0.5);
    ok &= Expect(segment.hit, "context segment query should hit mesh");
    ok &= Expect(
        context.Profile().geometryQueryCount == 1,
        "context geometry query count should remain stable");
    ok &= Expect(
        context.Profile().collisionQueryCount == 1,
        "context should count segment query as collision query");

    GeometryQueryContext emptyContext;
    ok &= Expect(
        !emptyContext.Build(std::vector<Triangle>()),
        "empty query context should fail to build");
    ok &= Expect(
        !emptyContext.IsValid(),
        "empty query context should be invalid");

    return ok;
}

bool TestTriangleAabbTreeQueryMatchesBruteForceAabb()
{
    const std::vector<Triangle> triangles = MakeSeparatedTriangles();

    TriangleAabbTree tree;
    bool ok = Expect(tree.Build(triangles), "tree should build");
    ok &= Expect(tree.IsValid(), "tree should be valid after build");
    ok &= Expect(
        tree.BuildStats().triangleCount == triangles.size(),
        "tree build stats should record triangle count");

    Aabb queryBox;
    queryBox.Expand(Vec3(9.5, -0.5, -0.5));
    queryBox.Expand(Vec3(10.5, 0.5, 0.5));

    std::vector<int> treeIds;
    tree.Query(queryBox, treeIds);
    std::set<int> treeIdSet(treeIds.begin(), treeIds.end());

    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        if (ComputeTriangleAabb(triangles[i]).Intersects(queryBox))
        {
            ok &= Expect(
                treeIdSet.find(static_cast<int>(i)) != treeIdSet.end(),
                "tree AABB query should contain every brute-force hit");
        }
    }

    return ok;
}

bool TestTriangleAabbTreeBoundaryCases()
{
    TriangleAabbTree emptyTree;
    bool ok = Expect(
        !emptyTree.Build(std::vector<Triangle>()),
        "empty tree build should fail");
    ok &= Expect(!emptyTree.IsValid(), "empty tree should be invalid");

    std::vector<int> ids;
    Aabb queryBox;
    queryBox.Expand(Vec3(-1.0, -1.0, -1.0));
    queryBox.Expand(Vec3(1.0, 1.0, 1.0));
    emptyTree.Query(queryBox, ids);
    ok &= Expect(ids.empty(), "empty tree query should return no ids");

    TriangleAabbTree singleTree;
    const std::vector<Triangle> single = { MakeUnitRightTriangle() };
    ok &= Expect(singleTree.Build(single), "single triangle tree should build");
    singleTree.Query(queryBox, ids);
    ok &= Expect(ids.size() == 1, "boundary query should include triangle");
    ok &= Expect(ids[0] == 0, "single triangle id should be zero");

    TriangleAabbTree multiTree;
    const std::vector<Triangle> many = MakeGridTriangles(16);
    ok &= Expect(multiTree.Build(many), "multi-level tree should build");
    ok &= Expect(
        multiTree.BuildStats().nodeCount > 1,
        "multi-level tree should contain internal nodes");

    Aabb boundaryBox;
    boundaryBox.Expand(Vec3(3.0, 0.0, 0.0));
    boundaryBox.Expand(Vec3(3.0, 0.0, 0.0));
    multiTree.Query(boundaryBox, ids);
    std::set<int> idSet(ids.begin(), ids.end());
    ok &= Expect(
        idSet.find(1) != idSet.end(),
        "point-on-boundary query should include touching triangle");

    return ok;
}

bool TestTriangleAabbTreeClosestPointMatchesBruteForce()
{
    const std::vector<Triangle> triangles = MakeSeparatedTriangles();

    TriangleAabbTree tree;
    bool ok = Expect(tree.Build(triangles), "tree should build for closest");

    const std::vector<Vec3> queryPoints = {
        Vec3(0.2, 0.2, 3.0),
        Vec3(10.25, 0.25, 2.0),
        Vec3(0.25, 10.25, -4.0),
        Vec3(5.0, 5.0, 1.0)
    };

    for (const Vec3& point : queryPoints)
    {
        const ClosestPointResult brute =
            ClosestPointToMeshBruteForce(point, triangles);
        TriangleAabbTreeStats stats;
        const ClosestPointResult indexed = tree.ClosestPoint(point, &stats);

        ok &= Expect(indexed.hit == brute.hit, "tree closest hit mismatch");
        ok &= Expect(
            Near(indexed.distance, brute.distance),
            "tree closest distance should match brute force");
        ok &= Expect(
            NearVec(indexed.closestPoint, brute.closestPoint),
            "tree closest point should match brute force");
        ok &= Expect(
            stats.visitedNodeCount > 0,
            "tree closest query should report visited nodes");
        ok &= Expect(
            stats.testedTriangleCount <= triangles.size(),
            "tree closest query should not test more than brute force");
    }

    return ok;
}

bool TestSegmentClearanceMatchesBruteForce()
{
    const std::vector<Triangle> triangles = MakeSeparatedTriangles();

    TriangleAabbTree tree;
    bool ok = Expect(tree.Build(triangles), "tree should build for segment");

    struct SegmentCase
    {
        Vec3 start;
        Vec3 end;
        double clearance;
    };

    const std::vector<SegmentCase> cases = {
        { Vec3(0.25, 0.25, 2.0), Vec3(0.25, 0.25, -2.0), 0.5 },
        { Vec3(2.0, 2.0, 1.0), Vec3(8.0, 2.0, 1.0), 0.5 },
        { Vec3(10.25, 0.25, 3.0), Vec3(10.25, 0.25, 2.0), 1.5 },
        { Vec3(-2.0, -2.0, 0.0), Vec3(-1.0, -1.0, 0.0), 0.25 }
    };

    for (const SegmentCase& item : cases)
    {
        const SegmentClearanceResult brute =
            SegmentClearanceToMeshBruteForce(
                item.start,
                item.end,
                item.clearance,
                triangles);
        TriangleAabbTreeStats stats;
        const SegmentClearanceResult indexed =
            tree.SegmentClearance(
                item.start,
                item.end,
                item.clearance,
                0.0,
                &stats);

        ok &= Expect(indexed.hit == brute.hit, "segment hit should match");
        ok &= Expect(indexed.pass == brute.pass, "segment pass should match");
        ok &= Expect(
            Near(indexed.minDistance, brute.minDistance),
            "segment min distance should match");
        ok &= Expect(
            indexed.triangleId == brute.triangleId,
            "segment closest triangle should match");
        ok &= Expect(
            stats.visitedNodeCount > 0,
            "segment query should report visited nodes");
        ok &= Expect(
            stats.testedTriangleCount == triangles.size(),
            "conservative segment tree query should test every triangle");
    }

    const SegmentClearanceResult empty =
        SegmentClearanceToMeshBruteForce(
            Vec3(),
            Vec3(1.0, 0.0, 0.0),
            0.1,
            std::vector<Triangle>());
    ok &= Expect(!empty.hit, "empty segment brute force should miss");
    ok &= Expect(empty.pass, "empty segment brute force should pass");

    return ok;
}

bool TestSegmentClearanceRadiusAndDryRunStats()
{
    const std::vector<Triangle> triangles = MakeSeparatedTriangles();

    TriangleAabbTree tree;
    bool ok = Expect(tree.Build(triangles), "tree should build for radius");

    const Vec3 start(10.25, 0.25, 3.0);
    const Vec3 end(10.25, 0.25, 2.0);
    const double clearance = 1.5;
    const double radius = 0.6;

    const SegmentClearanceResult brute =
        SegmentClearanceToMeshBruteForce(
            start,
            end,
            clearance,
            triangles,
            radius);

    TriangleAabbTreeStats stats;
    const SegmentClearanceResult indexed =
        tree.SegmentClearance(start, end, clearance, radius, &stats);

    ok &= Expect(indexed.hit == brute.hit, "radius hit should match");
    ok &= Expect(indexed.pass == brute.pass, "radius pass should match");
    ok &= Expect(
        Near(indexed.minDistance, brute.minDistance),
        "radius min distance should match");
    ok &= Expect(
        Near(indexed.requiredDistance, clearance + radius),
        "required distance should include radius");
    ok &= Expect(
        !indexed.pass,
        "segment with radius should fail when min distance is too small");
    ok &= Expect(
        stats.dryRunClearanceSafeNodeCount <= stats.visitedNodeCount,
        "dry-run safe node count should not exceed visited nodes");
    ok &= Expect(
        stats.dryRunPrunableNodeCount <= stats.visitedNodeCount,
        "dry-run prunable node count should not exceed visited nodes");

    GeometryQueryContext context;
    ok &= Expect(context.Build(triangles), "context should build for radius");
    const SegmentClearanceResult contextResult =
        context.SegmentClearance(start, end, clearance, radius);
    ok &= Expect(
        contextResult.pass == indexed.pass,
        "context radius result should match tree");
    ok &= Expect(
        context.Profile().spatialIndexDryRunClearanceSafeNodeCount <=
            context.Profile().spatialIndexVisitedNodeCount,
        "context dry-run safe count should not exceed visited nodes");

    return ok;
}
}

int main()
{
    bool ok = true;
    ok &= TestPlannerScaffoldStatus();
    ok &= TestPointToTriangleDistance();
    ok &= TestBruteForceClosestPointToMesh();
    ok &= TestTriangleMeshBuildsBounds();
    ok &= TestGeometryQueryContextAccumulatesProfile();
    ok &= TestTriangleAabbTreeQueryMatchesBruteForceAabb();
    ok &= TestTriangleAabbTreeBoundaryCases();
    ok &= TestTriangleAabbTreeClosestPointMatchesBruteForce();
    ok &= TestSegmentClearanceMatchesBruteForce();
    ok &= TestSegmentClearanceRadiusAndDryRunStats();
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
