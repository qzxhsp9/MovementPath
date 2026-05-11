#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace movement_path::geometry
{

// Independent geometry primitives for the long-term planner module.
// Do not include VoxelPathPlanner types here; the voxel planner remains a
// baseline implementation and should not leak into this module.
struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;

    Vec3(double ix, double iy, double iz)
        : x(ix), y(iy), z(iz)
    {
    }

    Vec3 operator+(const Vec3& other) const
    {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator-(const Vec3& other) const
    {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator*(double scalar) const
    {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    double Dot(const Vec3& other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    Vec3 Cross(const Vec3& other) const
    {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x);
    }

    double SquaredLength() const
    {
        return Dot(*this);
    }

    double Distance(const Vec3& other) const
    {
        return std::sqrt((*this - other).SquaredLength());
    }

    bool IsFinite() const
    {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

struct Triangle
{
    Vec3 p0;
    Vec3 p1;
    Vec3 p2;
};

struct Aabb
{
    Vec3 min{
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()};
    Vec3 max{
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest()};

    bool IsValid() const
    {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }

    void Expand(const Vec3& p)
    {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }

    void Expand(const Aabb& other)
    {
        if (!other.IsValid())
        {
            return;
        }

        Expand(other.min);
        Expand(other.max);
    }

    bool Intersects(const Aabb& other) const
    {
        return IsValid() && other.IsValid() &&
            min.x <= other.max.x && max.x >= other.min.x &&
            min.y <= other.max.y && max.y >= other.min.y &&
            min.z <= other.max.z && max.z >= other.min.z;
    }
};

} // namespace movement_path::geometry
