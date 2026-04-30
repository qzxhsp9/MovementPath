#pragma once

#include "GeometryPrimitives.h"

#include <cstddef>
#include <vector>

namespace movement_path::geometry
{

struct ClosestPointResult
{
    bool hit = false;
    double distance = 0.0;
    Vec3 closestPoint;
    int triangleId = -1;
    std::size_t testedTriangleCount = 0;
};

struct SegmentClearanceResult
{
    bool hit = false;
    bool pass = false;
    double minDistance = 0.0;
    double clearance = 0.0;
    double radius = 0.0;
    double requiredDistance = 0.0;
    int triangleId = -1;
    std::size_t testedTriangleCount = 0;
};

Aabb ComputeTriangleAabb(const Triangle& tri);

Aabb ComputeSegmentAabb(
    const Vec3& start,
    const Vec3& end);

double SquaredDistancePointToAabb(
    const Vec3& point,
    const Aabb& box);

double SquaredDistanceAabbToAabb(
    const Aabb& lhs,
    const Aabb& rhs);

Vec3 ClosestPointOnTriangle(
    const Vec3& point,
    const Triangle& tri);

double DistancePointToTriangle(
    const Vec3& point,
    const Triangle& tri);

double DistanceSegmentToTriangle(
    const Vec3& start,
    const Vec3& end,
    const Triangle& tri);

ClosestPointResult ClosestPointToMeshBruteForce(
    const Vec3& point,
    const std::vector<Triangle>& triangles);

SegmentClearanceResult SegmentClearanceToMeshBruteForce(
    const Vec3& start,
    const Vec3& end,
    double clearance,
    const std::vector<Triangle>& triangles,
    double radius = 0.0);

} // namespace movement_path::geometry
