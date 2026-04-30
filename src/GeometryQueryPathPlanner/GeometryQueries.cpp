#include "GeometryQueries.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace movement_path::geometry
{
namespace
{
constexpr double kDegenerateEpsilon = 1.0e-12;
constexpr double kPointOnTriangleEpsilonSquared = 1.0e-24;

Vec3 ClosestPointOnSegment(
    const Vec3& point,
    const Vec3& a,
    const Vec3& b)
{
    const Vec3 ab = b - a;
    const double denom = ab.SquaredLength();

    if (denom <= kDegenerateEpsilon)
    {
        return a;
    }

    const double t = std::clamp((point - a).Dot(ab) / denom, 0.0, 1.0);
    return a + ab * t;
}

bool PointInsideTriangle(
    const Vec3& point,
    const Triangle& tri)
{
    const Vec3 closest = ClosestPointOnTriangle(point, tri);
    return (point - closest).SquaredLength() <= kPointOnTriangleEpsilonSquared;
}

bool SegmentIntersectsTriangle(
    const Vec3& start,
    const Vec3& end,
    const Triangle& tri)
{
    const Vec3 direction = end - start;
    const Vec3 edge1 = tri.p1 - tri.p0;
    const Vec3 edge2 = tri.p2 - tri.p0;
    const Vec3 pvec = direction.Cross(edge2);
    const double det = edge1.Dot(pvec);

    if (std::abs(det) <= kDegenerateEpsilon)
    {
        return PointInsideTriangle(start, tri) ||
            PointInsideTriangle(end, tri);
    }

    const double invDet = 1.0 / det;
    const Vec3 tvec = start - tri.p0;
    const double u = tvec.Dot(pvec) * invDet;

    if (u < 0.0 || u > 1.0)
    {
        return false;
    }

    const Vec3 qvec = tvec.Cross(edge1);
    const double v = direction.Dot(qvec) * invDet;

    if (v < 0.0 || u + v > 1.0)
    {
        return false;
    }

    const double t = edge2.Dot(qvec) * invDet;
    return t >= 0.0 && t <= 1.0;
}

double SquaredDistanceSegmentToSegment(
    const Vec3& p1,
    const Vec3& q1,
    const Vec3& p2,
    const Vec3& q2)
{
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const double a = d1.Dot(d1);
    const double e = d2.Dot(d2);
    const double f = d2.Dot(r);

    double s = 0.0;
    double t = 0.0;

    if (a <= kDegenerateEpsilon && e <= kDegenerateEpsilon)
    {
        return (p1 - p2).SquaredLength();
    }

    if (a <= kDegenerateEpsilon)
    {
        t = std::clamp(f / e, 0.0, 1.0);
    }
    else
    {
        const double c = d1.Dot(r);

        if (e <= kDegenerateEpsilon)
        {
            s = std::clamp(-c / a, 0.0, 1.0);
        }
        else
        {
            const double b = d1.Dot(d2);
            const double denom = a * e - b * b;

            if (denom > kDegenerateEpsilon)
            {
                s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
            }

            t = (b * s + f) / e;

            if (t < 0.0)
            {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            }
            else if (t > 1.0)
            {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }

    const Vec3 c1 = p1 + d1 * s;
    const Vec3 c2 = p2 + d2 * t;
    return (c1 - c2).SquaredLength();
}
}

Aabb ComputeTriangleAabb(const Triangle& tri)
{
    Aabb box;
    box.Expand(tri.p0);
    box.Expand(tri.p1);
    box.Expand(tri.p2);
    return box;
}

Aabb ComputeSegmentAabb(
    const Vec3& start,
    const Vec3& end)
{
    Aabb box;
    box.Expand(start);
    box.Expand(end);
    return box;
}

double SquaredDistancePointToAabb(
    const Vec3& point,
    const Aabb& box)
{
    if (!box.IsValid())
    {
        return std::numeric_limits<double>::infinity();
    }

    const double dx =
        point.x < box.min.x ? box.min.x - point.x :
        point.x > box.max.x ? point.x - box.max.x :
        0.0;
    const double dy =
        point.y < box.min.y ? box.min.y - point.y :
        point.y > box.max.y ? point.y - box.max.y :
        0.0;
    const double dz =
        point.z < box.min.z ? box.min.z - point.z :
        point.z > box.max.z ? point.z - box.max.z :
        0.0;

    return dx * dx + dy * dy + dz * dz;
}

double SquaredDistanceAabbToAabb(
    const Aabb& lhs,
    const Aabb& rhs)
{
    if (!lhs.IsValid() || !rhs.IsValid())
    {
        return std::numeric_limits<double>::infinity();
    }

    const double dx =
        lhs.max.x < rhs.min.x ? rhs.min.x - lhs.max.x :
        rhs.max.x < lhs.min.x ? lhs.min.x - rhs.max.x :
        0.0;
    const double dy =
        lhs.max.y < rhs.min.y ? rhs.min.y - lhs.max.y :
        rhs.max.y < lhs.min.y ? lhs.min.y - rhs.max.y :
        0.0;
    const double dz =
        lhs.max.z < rhs.min.z ? rhs.min.z - lhs.max.z :
        rhs.max.z < lhs.min.z ? lhs.min.z - rhs.max.z :
        0.0;

    return dx * dx + dy * dy + dz * dz;
}

Vec3 ClosestPointOnTriangle(
    const Vec3& point,
    const Triangle& tri)
{
    const Vec3 ab = tri.p1 - tri.p0;
    const Vec3 ac = tri.p2 - tri.p0;
    const Vec3 ap = point - tri.p0;
    const double d1 = ab.Dot(ap);
    const double d2 = ac.Dot(ap);

    if (d1 <= 0.0 && d2 <= 0.0)
    {
        return tri.p0;
    }

    const Vec3 bp = point - tri.p1;
    const double d3 = ab.Dot(bp);
    const double d4 = ac.Dot(bp);

    if (d3 >= 0.0 && d4 <= d3)
    {
        return tri.p1;
    }

    const double vc = d1 * d4 - d3 * d2;

    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        const double v = d1 / (d1 - d3);
        return tri.p0 + ab * v;
    }

    const Vec3 cp = point - tri.p2;
    const double d5 = ab.Dot(cp);
    const double d6 = ac.Dot(cp);

    if (d6 >= 0.0 && d5 <= d6)
    {
        return tri.p2;
    }

    const double vb = d5 * d2 - d1 * d6;

    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        const double w = d2 / (d2 - d6);
        return tri.p0 + ac * w;
    }

    const double va = d3 * d6 - d5 * d4;

    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        const Vec3 bc = tri.p2 - tri.p1;
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return tri.p1 + bc * w;
    }

    const double denom = va + vb + vc;

    if (std::abs(denom) <= kDegenerateEpsilon)
    {
        Vec3 best = tri.p0;
        double bestSquared = (point - best).SquaredLength();

        const Vec3 segmentPoints[] = {
            ClosestPointOnSegment(point, tri.p0, tri.p1),
            ClosestPointOnSegment(point, tri.p1, tri.p2),
            ClosestPointOnSegment(point, tri.p2, tri.p0)
        };

        for (const Vec3& candidate : segmentPoints)
        {
            const double squared = (point - candidate).SquaredLength();
            if (squared < bestSquared)
            {
                best = candidate;
                bestSquared = squared;
            }
        }

        return best;
    }

    const double v = vb / denom;
    const double w = vc / denom;
    return tri.p0 + ab * v + ac * w;
}

double DistancePointToTriangle(
    const Vec3& point,
    const Triangle& tri)
{
    return point.Distance(ClosestPointOnTriangle(point, tri));
}

double DistanceSegmentToTriangle(
    const Vec3& start,
    const Vec3& end,
    const Triangle& tri)
{
    if (SegmentIntersectsTriangle(start, end, tri))
    {
        return 0.0;
    }

    double bestSquared = std::min(
        (start - ClosestPointOnTriangle(start, tri)).SquaredLength(),
        (end - ClosestPointOnTriangle(end, tri)).SquaredLength());

    bestSquared = std::min(
        bestSquared,
        SquaredDistanceSegmentToSegment(start, end, tri.p0, tri.p1));
    bestSquared = std::min(
        bestSquared,
        SquaredDistanceSegmentToSegment(start, end, tri.p1, tri.p2));
    bestSquared = std::min(
        bestSquared,
        SquaredDistanceSegmentToSegment(start, end, tri.p2, tri.p0));

    return std::sqrt(bestSquared);
}

ClosestPointResult ClosestPointToMeshBruteForce(
    const Vec3& point,
    const std::vector<Triangle>& triangles)
{
    ClosestPointResult result;
    result.testedTriangleCount = triangles.size();

    if (triangles.empty())
    {
        return result;
    }

    double bestSquared = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        const Vec3 closest = ClosestPointOnTriangle(point, triangles[i]);
        const double squared = (point - closest).SquaredLength();

        if (squared < bestSquared)
        {
            result.hit = true;
            result.closestPoint = closest;
            result.triangleId = static_cast<int>(i);
            bestSquared = squared;
        }
    }

    result.distance = std::sqrt(bestSquared);
    return result;
}

SegmentClearanceResult SegmentClearanceToMeshBruteForce(
    const Vec3& start,
    const Vec3& end,
    double clearance,
    const std::vector<Triangle>& triangles,
    double radius)
{
    SegmentClearanceResult result;
    result.testedTriangleCount = triangles.size();
    result.pass = triangles.empty();
    result.clearance = clearance;
    result.radius = radius;
    result.requiredDistance = clearance + radius;

    if (triangles.empty())
    {
        return result;
    }

    result.hit = true;
    double bestDistance = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        const double distance =
            DistanceSegmentToTriangle(start, end, triangles[i]);

        if (distance < bestDistance)
        {
            bestDistance = distance;
            result.triangleId = static_cast<int>(i);
        }
    }

    result.minDistance = bestDistance;
    result.pass = bestDistance >= result.requiredDistance;
    return result;
}

} // namespace movement_path::geometry
