#include "TriangleMesh.h"

namespace movement_path::geometry
{

bool TriangleMesh::Build(const std::vector<Triangle>& triangles)
{
    m_triangles = triangles;
    m_bounds = Aabb();

    for (const Triangle& tri : m_triangles)
    {
        m_bounds.Expand(ComputeTriangleAabb(tri));
    }

    return !m_triangles.empty();
}

bool TriangleMesh::IsValid() const
{
    return !m_triangles.empty() && m_bounds.IsValid();
}

const std::vector<Triangle>& TriangleMesh::Triangles() const
{
    return m_triangles;
}

const Aabb& TriangleMesh::Bounds() const
{
    return m_bounds;
}

} // namespace movement_path::geometry
