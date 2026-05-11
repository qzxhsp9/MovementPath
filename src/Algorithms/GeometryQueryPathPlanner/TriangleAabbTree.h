#pragma once

#include "GeometryQueries.h"

#include <cstddef>
#include <vector>

namespace movement_path::geometry
{

struct TriangleAabbTreeStats
{
    std::size_t nodeCount = 0;
    std::size_t triangleCount = 0;
    std::size_t visitedNodeCount = 0;
    std::size_t testedTriangleCount = 0;
    std::size_t dryRunPrunableNodeCount = 0;
    std::size_t dryRunClearanceSafeNodeCount = 0;
    std::size_t dryRunEstimatedVisitedNodeCount = 0;
    std::size_t dryRunEstimatedTestedTriangleCount = 0;
    std::size_t dryRunEstimatedSkippedTriangleCount = 0;
    std::size_t dryRunEstimatedBestDistancePruneCount = 0;
    std::size_t dryRunEstimatedClearanceSafePruneCount = 0;
};

class TriangleAabbTree
{
public:
    bool Build(const std::vector<Triangle>& triangles);

    bool IsValid() const;

    void Query(
        const Aabb& box,
        std::vector<int>& outTriangleIds) const;

    ClosestPointResult ClosestPoint(
        const Vec3& point,
        TriangleAabbTreeStats* stats = nullptr) const;

    SegmentClearanceResult SegmentClearance(
        const Vec3& start,
        const Vec3& end,
        double clearance,
        double radius = 0.0,
        TriangleAabbTreeStats* stats = nullptr) const;

    TriangleAabbTreeStats EstimateSegmentClearancePruning(
        const Vec3& start,
        const Vec3& end,
        double clearance,
        double radius = 0.0) const;

    const TriangleAabbTreeStats& BuildStats() const;

private:
    struct Node
    {
        Aabb bounds;
        int left = -1;
        int right = -1;
        std::size_t subtreeTriangleCount = 0;
        std::vector<int> triangleIds;

        bool IsLeaf() const
        {
            return left < 0 && right < 0;
        }
    };

    int BuildNode(
        std::vector<int>& triangleIds,
        int begin,
        int end);

    void QueryNode(
        int nodeIndex,
        const Aabb& box,
        std::vector<int>& outTriangleIds) const;

    void ClosestPointNode(
        int nodeIndex,
        const Vec3& point,
        double& bestSquared,
        ClosestPointResult& result,
        TriangleAabbTreeStats* stats) const;

    void SegmentClearanceNode(
        int nodeIndex,
        const Vec3& start,
        const Vec3& end,
        const Aabb& segmentBounds,
        double requiredDistance,
        double& bestDistance,
        SegmentClearanceResult& result,
        TriangleAabbTreeStats* stats) const;

    void EstimateSegmentClearancePruningNode(
        int nodeIndex,
        const Vec3& start,
        const Vec3& end,
        const Aabb& segmentBounds,
        double requiredDistance,
        double& bestDistance,
        TriangleAabbTreeStats& stats) const;

    std::vector<Triangle> m_triangles;
    std::vector<Node> m_nodes;
    TriangleAabbTreeStats m_buildStats;
};

} // namespace movement_path::geometry
