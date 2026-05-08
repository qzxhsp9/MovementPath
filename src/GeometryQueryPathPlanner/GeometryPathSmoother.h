#pragma once

#include "GeometryPrimitives.h"
#include "GeometryQueryContext.h"

#include <cstddef>
#include <string>
#include <vector>

namespace movement_path::geometry
{

enum class GeometryPathSmoothStatus
{
    InvalidInput,
    Succeeded,
    FailedConstraint
};

struct GeometryPathSmoothingOptions
{
    double clearance = 0.0;
    double radius = 0.0;
    double maxDeviation = 0.0;
    double maxCurvature = 0.0;
    double sampleSpacing = 0.0;
    std::size_t samplesPerSegment = 8;
};

struct GeometryPathSmoothingProfile
{
    std::size_t rawPointCount = 0;
    std::size_t smoothedPointCount = 0;
    std::size_t collisionCheckCount = 0;
    std::size_t sampleClearanceCheckCount = 0;
    double rawLength = 0.0;
    double smoothedLength = 0.0;
    double maxDeviation = 0.0;
    double maxCurvature = 0.0;
    double minClearance = 0.0;
};

struct GeometryPathSmoothingResult
{
    GeometryPathSmoothStatus status = GeometryPathSmoothStatus::InvalidInput;
    std::string message;
    std::vector<Vec3> path;
    GeometryPathSmoothingProfile profile;
};

GeometryPathSmoothingResult SmoothPathCatmullRom(
    const std::vector<Vec3>& rawPath,
    GeometryQueryContext& queryContext,
    const GeometryPathSmoothingOptions& options);

} // namespace movement_path::geometry
