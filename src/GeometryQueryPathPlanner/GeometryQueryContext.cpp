#include "GeometryQueryContext.h"

#include <chrono>
#include <algorithm>

namespace movement_path::geometry
{

bool GeometryQueryContext::Build(const std::vector<Triangle>& triangles)
{
    m_profile = GeometryPathProfile();
    m_profile.triangleCount = triangles.size();

    const auto start = std::chrono::steady_clock::now();
    const bool meshBuilt = m_mesh.Build(triangles);
    const bool indexBuilt = meshBuilt && m_index.Build(m_mesh.Triangles());
    const auto end = std::chrono::steady_clock::now();

    m_profile.spatialIndexBuildMs =
        std::chrono::duration<double, std::milli>(end - start).count();
    m_profile.spatialIndexNodeCount = m_index.BuildStats().nodeCount;

    return meshBuilt && indexBuilt;
}

bool GeometryQueryContext::IsValid() const
{
    return m_mesh.IsValid() && m_index.IsValid();
}

const TriangleMesh& GeometryQueryContext::Mesh() const
{
    return m_mesh;
}

const TriangleAabbTree& GeometryQueryContext::Index() const
{
    return m_index;
}

ClosestPointResult GeometryQueryContext::ClosestPoint(const Vec3& point)
{
    TriangleAabbTreeStats stats;
    ClosestPointResult result = m_index.ClosestPoint(point, &stats);
    AccumulateIndexStats(stats, false);
    return result;
}

SegmentClearanceResult GeometryQueryContext::SegmentClearance(
    const Vec3& start,
    const Vec3& end,
    double clearance,
    double radius)
{
    TriangleAabbTreeStats stats;
    SegmentClearanceResult result =
        m_index.SegmentClearance(start, end, clearance, radius, &stats);
    AccumulateIndexStats(stats, true);
    return result;
}

const GeometryPathProfile& GeometryQueryContext::Profile() const
{
    return m_profile;
}

void GeometryQueryContext::AccumulateIndexStats(
    const TriangleAabbTreeStats& stats,
    bool collisionQuery)
{
    if (collisionQuery)
    {
        ++m_profile.collisionQueryCount;
    }
    else
    {
        ++m_profile.geometryQueryCount;
    }

    m_profile.geometryCandidateTriangleCount += stats.testedTriangleCount;
    m_profile.maxCandidateTriangleCount =
        std::max(m_profile.maxCandidateTriangleCount, stats.testedTriangleCount);
    m_profile.spatialIndexVisitedNodeCount += stats.visitedNodeCount;
    m_profile.spatialIndexDryRunPrunableNodeCount +=
        stats.dryRunPrunableNodeCount;
    m_profile.spatialIndexDryRunClearanceSafeNodeCount +=
        stats.dryRunClearanceSafeNodeCount;
}

} // namespace movement_path::geometry
