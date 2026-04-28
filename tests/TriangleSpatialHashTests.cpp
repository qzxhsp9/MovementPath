#include "TestCommon.h"

#include <cstdlib>
#include <set>

namespace
{
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
        if (IntersectsAABBForTest(
                ComputeTriangleAABBForTest(triangles[i]),
                queryBox))
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
}

int main()
{
    bool ok = true;
    ok &= TestTriangleSpatialHashCoversBruteForce();
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
