#pragma once

#include "GeometryQueries.h"

#include <vector>

namespace movement_path::geometry
{

class TriangleMesh
{
public:
    bool Build(const std::vector<Triangle>& triangles);

    bool IsValid() const;

    const std::vector<Triangle>& Triangles() const;

    const Aabb& Bounds() const;

private:
    std::vector<Triangle> m_triangles;
    Aabb m_bounds;
};

} // namespace movement_path::geometry
