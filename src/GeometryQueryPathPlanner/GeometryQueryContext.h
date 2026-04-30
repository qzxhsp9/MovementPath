#pragma once

#include "GeometryQueryPathPlanner.h"
#include "TriangleAabbTree.h"
#include "TriangleMesh.h"

namespace movement_path::geometry
{

class GeometryQueryContext
{
public:
    bool Build(const std::vector<Triangle>& triangles);

    bool IsValid() const;

    const TriangleMesh& Mesh() const;

    const TriangleAabbTree& Index() const;

    ClosestPointResult ClosestPoint(const Vec3& point);

    SegmentClearanceResult SegmentClearance(
        const Vec3& start,
        const Vec3& end,
        double clearance,
        double radius = 0.0);

    const GeometryPathProfile& Profile() const;

private:
    void AccumulateIndexStats(
        const TriangleAabbTreeStats& stats,
        bool collisionQuery);

    TriangleMesh m_mesh;
    TriangleAabbTree m_index;
    GeometryPathProfile m_profile;
};

} // namespace movement_path::geometry
