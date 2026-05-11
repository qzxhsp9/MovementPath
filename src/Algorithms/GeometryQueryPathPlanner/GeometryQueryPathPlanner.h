#pragma once

#include "GeometryPrimitives.h"

#include <cstddef>
#include <string>
#include <vector>

namespace movement_path::geometry
{

enum class GeometryPathStatus
{
    NotImplemented,
    InvalidInput,
    Succeeded,
    Failed
};

struct GeometryPathOptions
{
    // Desired clearance from mesh surfaces. Future implementations should use
    // this in closest-point or signed-distance queries instead of prebuilding
    // a dense voxel clearance band.
    double clearance = 0.0;

    // Upper bound for future graph expansion or sampling iterations.
    std::size_t maxExpansionCount = 0;
};

struct GeometryPathRequest
{
    std::vector<Triangle> triangles;
    Vec3 startPoint;
    Vec3 goalPoint;
};

struct GeometryPathProfile
{
    std::size_t triangleCount = 0;
    std::size_t geometryQueryCount = 0;
    std::size_t collisionQueryCount = 0;
    std::size_t geometryCandidateTriangleCount = 0;
    std::size_t maxCandidateTriangleCount = 0;
    std::size_t spatialIndexNodeCount = 0;
    std::size_t spatialIndexVisitedNodeCount = 0;
    std::size_t spatialIndexDryRunPrunableNodeCount = 0;
    std::size_t spatialIndexDryRunClearanceSafeNodeCount = 0;
    std::size_t spatialIndexDryRunEstimatedVisitedNodeCount = 0;
    std::size_t spatialIndexDryRunEstimatedTestedTriangleCount = 0;
    std::size_t spatialIndexDryRunEstimatedSkippedTriangleCount = 0;
    std::size_t spatialIndexDryRunEstimatedBestDistancePruneCount = 0;
    std::size_t spatialIndexDryRunEstimatedClearanceSafePruneCount = 0;
    std::size_t searchNodeCount = 0;
    std::size_t searchEdgeCount = 0;
    std::size_t feasibleSearchEdgeCount = 0;
    std::size_t blockedSearchEdgeCount = 0;
    double spatialIndexBuildMs = 0.0;
    double searchMs = 0.0;
    double optimizeMs = 0.0;
};

struct GeometryPathResult
{
    GeometryPathStatus status = GeometryPathStatus::NotImplemented;
    std::string message;
    std::vector<Vec3> path;
    GeometryPathProfile profile;
};

// Long-term planner boundary for geometry-query-driven path planning.
// This module intentionally does not depend on VoxelPathPlanner. The current
// voxel implementation should be treated as baseline and integration should
// happen through explicit adapters after this planner has its own tests.
class GeometryQueryPathPlanner
{
public:
    GeometryPathResult Plan(
        const GeometryPathRequest& request,
        const GeometryPathOptions& options) const;
};

} // namespace movement_path::geometry
