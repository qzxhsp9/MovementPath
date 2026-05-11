#include "TriangleAabbTree.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace movement_path::geometry
{
namespace
{
constexpr int kLeafTriangleCount = 4;

Vec3 TriangleCentroid(const Triangle& tri)
{
    return Vec3(
        (tri.p0.x + tri.p1.x + tri.p2.x) / 3.0,
        (tri.p0.y + tri.p1.y + tri.p2.y) / 3.0,
        (tri.p0.z + tri.p1.z + tri.p2.z) / 3.0);
}
}

bool TriangleAabbTree::Build(const std::vector<Triangle>& triangles)
{
    m_triangles = triangles;
    m_nodes.clear();
    m_buildStats = TriangleAabbTreeStats();
    m_buildStats.triangleCount = triangles.size();

    if (triangles.empty())
    {
        return false;
    }

    std::vector<int> ids(triangles.size());

    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        ids[i] = static_cast<int>(i);
    }

    BuildNode(ids, 0, static_cast<int>(ids.size()));
    m_buildStats.nodeCount = m_nodes.size();
    return true;
}

bool TriangleAabbTree::IsValid() const
{
    return !m_triangles.empty() && !m_nodes.empty();
}

void TriangleAabbTree::Query(
    const Aabb& box,
    std::vector<int>& outTriangleIds) const
{
    outTriangleIds.clear();

    if (!IsValid() || !box.IsValid())
    {
        return;
    }

    QueryNode(0, box, outTriangleIds);
}

ClosestPointResult TriangleAabbTree::ClosestPoint(
    const Vec3& point,
    TriangleAabbTreeStats* stats) const
{
    if (stats != nullptr)
    {
        *stats = TriangleAabbTreeStats();
        stats->nodeCount = m_nodes.size();
        stats->triangleCount = m_triangles.size();
    }

    ClosestPointResult result;

    if (!IsValid())
    {
        return result;
    }

    double bestSquared = std::numeric_limits<double>::infinity();
    ClosestPointNode(0, point, bestSquared, result, stats);

    if (result.hit)
    {
        result.distance = std::sqrt(bestSquared);
    }

    if (stats != nullptr)
    {
        result.testedTriangleCount = stats->testedTriangleCount;
    }

    return result;
}

SegmentClearanceResult TriangleAabbTree::SegmentClearance(
    const Vec3& start,
    const Vec3& end,
    double clearance,
    double radius,
    TriangleAabbTreeStats* stats) const
{
    if (stats != nullptr)
    {
        *stats = TriangleAabbTreeStats();
        stats->nodeCount = m_nodes.size();
        stats->triangleCount = m_triangles.size();
    }

    SegmentClearanceResult result;
    result.pass = !IsValid();
    result.clearance = clearance;
    result.radius = radius;
    result.requiredDistance = clearance + radius;

    if (!IsValid())
    {
        return result;
    }

    double bestDistance = std::numeric_limits<double>::infinity();
    const Aabb segmentBounds = ComputeSegmentAabb(start, end);
    SegmentClearanceNode(
        0,
        start,
        end,
        segmentBounds,
        result.requiredDistance,
        bestDistance,
        result,
        stats);

    if (stats != nullptr)
    {
        TriangleAabbTreeStats estimate =
            EstimateSegmentClearancePruning(
                start,
                end,
                clearance,
                radius);
        stats->dryRunEstimatedVisitedNodeCount =
            estimate.dryRunEstimatedVisitedNodeCount;
        stats->dryRunEstimatedTestedTriangleCount =
            estimate.dryRunEstimatedTestedTriangleCount;
        stats->dryRunEstimatedSkippedTriangleCount =
            estimate.dryRunEstimatedSkippedTriangleCount;
        stats->dryRunEstimatedBestDistancePruneCount =
            estimate.dryRunEstimatedBestDistancePruneCount;
        stats->dryRunEstimatedClearanceSafePruneCount =
            estimate.dryRunEstimatedClearanceSafePruneCount;
    }

    if (result.hit)
    {
        result.minDistance = bestDistance;
        result.pass = bestDistance >= result.requiredDistance;
    }

    if (stats != nullptr)
    {
        result.testedTriangleCount = stats->testedTriangleCount;
    }

    return result;
}

TriangleAabbTreeStats TriangleAabbTree::EstimateSegmentClearancePruning(
    const Vec3& start,
    const Vec3& end,
    double clearance,
    double radius) const
{
    TriangleAabbTreeStats stats;
    stats.nodeCount = m_nodes.size();
    stats.triangleCount = m_triangles.size();

    if (!IsValid())
    {
        return stats;
    }

    const Aabb segmentBounds = ComputeSegmentAabb(start, end);
    const double requiredDistance = clearance + radius;
    double bestDistance = std::numeric_limits<double>::infinity();
    EstimateSegmentClearancePruningNode(
        0,
        start,
        end,
        segmentBounds,
        requiredDistance,
        bestDistance,
        stats);

    return stats;
}

const TriangleAabbTreeStats& TriangleAabbTree::BuildStats() const
{
    return m_buildStats;
}

int TriangleAabbTree::BuildNode(
    std::vector<int>& triangleIds,
    int begin,
    int end)
{
    Node node;

    for (int i = begin; i < end; ++i)
    {
        node.bounds.Expand(ComputeTriangleAabb(m_triangles[triangleIds[i]]));
    }

    const int count = end - begin;
    const int nodeIndex = static_cast<int>(m_nodes.size());
    node.subtreeTriangleCount = static_cast<std::size_t>(count);
    m_nodes.push_back(node);

    if (count <= kLeafTriangleCount)
    {
        m_nodes[nodeIndex].triangleIds.assign(
            triangleIds.begin() + begin,
            triangleIds.begin() + end);
        return nodeIndex;
    }

    const double extentX = node.bounds.max.x - node.bounds.min.x;
    const double extentY = node.bounds.max.y - node.bounds.min.y;
    const double extentZ = node.bounds.max.z - node.bounds.min.z;

    int axis = 0;
    if (extentY > extentX && extentY >= extentZ)
    {
        axis = 1;
    }
    else if (extentZ > extentX && extentZ > extentY)
    {
        axis = 2;
    }

    const int mid = begin + count / 2;

    std::nth_element(
        triangleIds.begin() + begin,
        triangleIds.begin() + mid,
        triangleIds.begin() + end,
        [this, axis](int lhs, int rhs)
        {
            const Vec3 lc = TriangleCentroid(m_triangles[lhs]);
            const Vec3 rc = TriangleCentroid(m_triangles[rhs]);
            if (axis == 0)
            {
                return lc.x < rc.x;
            }
            if (axis == 1)
            {
                return lc.y < rc.y;
            }
            return lc.z < rc.z;
        });

    m_nodes[nodeIndex].left = BuildNode(triangleIds, begin, mid);
    m_nodes[nodeIndex].right = BuildNode(triangleIds, mid, end);
    return nodeIndex;
}

void TriangleAabbTree::QueryNode(
    int nodeIndex,
    const Aabb& box,
    std::vector<int>& outTriangleIds) const
{
    const Node& node = m_nodes[nodeIndex];

    if (!node.bounds.Intersects(box))
    {
        return;
    }

    if (node.IsLeaf())
    {
        for (int id : node.triangleIds)
        {
            if (ComputeTriangleAabb(m_triangles[id]).Intersects(box))
            {
                outTriangleIds.push_back(id);
            }
        }
        return;
    }

    QueryNode(node.left, box, outTriangleIds);
    QueryNode(node.right, box, outTriangleIds);
}

void TriangleAabbTree::ClosestPointNode(
    int nodeIndex,
    const Vec3& point,
    double& bestSquared,
    ClosestPointResult& result,
    TriangleAabbTreeStats* stats) const
{
    const Node& node = m_nodes[nodeIndex];
    const double nodeDistance = SquaredDistancePointToAabb(point, node.bounds);

    if (nodeDistance > bestSquared)
    {
        return;
    }

    if (stats != nullptr)
    {
        ++stats->visitedNodeCount;
    }

    if (node.IsLeaf())
    {
        for (int id : node.triangleIds)
        {
            if (stats != nullptr)
            {
                ++stats->testedTriangleCount;
            }

            const Vec3 closest =
                ClosestPointOnTriangle(point, m_triangles[id]);
            const double squared = (point - closest).SquaredLength();

            if (squared < bestSquared)
            {
                bestSquared = squared;
                result.hit = true;
                result.closestPoint = closest;
                result.triangleId = id;
            }
        }
        return;
    }

    const double leftDistance =
        SquaredDistancePointToAabb(point, m_nodes[node.left].bounds);
    const double rightDistance =
        SquaredDistancePointToAabb(point, m_nodes[node.right].bounds);

    if (leftDistance <= rightDistance)
    {
        ClosestPointNode(node.left, point, bestSquared, result, stats);
        ClosestPointNode(node.right, point, bestSquared, result, stats);
    }
    else
    {
        ClosestPointNode(node.right, point, bestSquared, result, stats);
        ClosestPointNode(node.left, point, bestSquared, result, stats);
    }
}

void TriangleAabbTree::SegmentClearanceNode(
    int nodeIndex,
    const Vec3& start,
    const Vec3& end,
    const Aabb& segmentBounds,
    double requiredDistance,
    double& bestDistance,
    SegmentClearanceResult& result,
    TriangleAabbTreeStats* stats) const
{
    const Node& node = m_nodes[nodeIndex];
    const double nodeLowerBound =
        std::sqrt(SquaredDistanceAabbToAabb(segmentBounds, node.bounds));

    if (stats != nullptr)
    {
        ++stats->visitedNodeCount;
        if (nodeLowerBound > bestDistance)
        {
            ++stats->dryRunPrunableNodeCount;
        }
        if (nodeLowerBound >= requiredDistance)
        {
            ++stats->dryRunClearanceSafeNodeCount;
        }
    }

    if (node.IsLeaf())
    {
        for (int id : node.triangleIds)
        {
            if (stats != nullptr)
            {
                ++stats->testedTriangleCount;
            }

            const double distance =
                DistanceSegmentToTriangle(start, end, m_triangles[id]);

            if (distance < bestDistance)
            {
                bestDistance = distance;
                result.hit = true;
                result.triangleId = id;
            }
        }
        return;
    }

    // Conservative first version: traverse both children to preserve exact
    // parity with brute force before adding distance-based pruning.
    SegmentClearanceNode(
        node.left,
        start,
        end,
        segmentBounds,
        requiredDistance,
        bestDistance,
        result,
        stats);
    SegmentClearanceNode(
        node.right,
        start,
        end,
        segmentBounds,
        requiredDistance,
        bestDistance,
        result,
        stats);
}

void TriangleAabbTree::EstimateSegmentClearancePruningNode(
    int nodeIndex,
    const Vec3& start,
    const Vec3& end,
    const Aabb& segmentBounds,
    double requiredDistance,
    double& bestDistance,
    TriangleAabbTreeStats& stats) const
{
    const Node& node = m_nodes[nodeIndex];
    ++stats.dryRunEstimatedVisitedNodeCount;

    const double nodeLowerBound =
        std::sqrt(SquaredDistanceAabbToAabb(segmentBounds, node.bounds));

    if (nodeLowerBound > bestDistance)
    {
        ++stats.dryRunEstimatedBestDistancePruneCount;
        stats.dryRunEstimatedSkippedTriangleCount +=
            node.subtreeTriangleCount;
        return;
    }

    if (nodeLowerBound >= requiredDistance)
    {
        ++stats.dryRunEstimatedClearanceSafePruneCount;
        stats.dryRunEstimatedSkippedTriangleCount +=
            node.subtreeTriangleCount;
        return;
    }

    if (node.IsLeaf())
    {
        stats.dryRunEstimatedTestedTriangleCount += node.triangleIds.size();

        for (int id : node.triangleIds)
        {
            const double distance =
                DistanceSegmentToTriangle(start, end, m_triangles[id]);

            if (distance < bestDistance)
            {
                bestDistance = distance;
            }
        }
        return;
    }

    const double leftDistance =
        SquaredDistanceAabbToAabb(segmentBounds, m_nodes[node.left].bounds);
    const double rightDistance =
        SquaredDistanceAabbToAabb(segmentBounds, m_nodes[node.right].bounds);

    if (leftDistance <= rightDistance)
    {
        EstimateSegmentClearancePruningNode(
            node.left,
            start,
            end,
            segmentBounds,
            requiredDistance,
            bestDistance,
            stats);
        EstimateSegmentClearancePruningNode(
            node.right,
            start,
            end,
            segmentBounds,
            requiredDistance,
            bestDistance,
            stats);
    }
    else
    {
        EstimateSegmentClearancePruningNode(
            node.right,
            start,
            end,
            segmentBounds,
            requiredDistance,
            bestDistance,
            stats);
        EstimateSegmentClearancePruningNode(
            node.left,
            start,
            end,
            segmentBounds,
            requiredDistance,
            bestDistance,
            stats);
    }
}

} // namespace movement_path::geometry
