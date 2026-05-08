#include "GeometryPathSmoother.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace movement_path::geometry
{
namespace
{
constexpr double kEpsilon = 1.0e-12;

bool IsFiniteNonNegative(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

double Length(const Vec3& value)
{
    return std::sqrt(value.SquaredLength());
}

Vec3 Lerp(
    const Vec3& lhs,
    const Vec3& rhs,
    double t)
{
    return lhs * (1.0 - t) + rhs * t;
}

double PathLength(const std::vector<Vec3>& path)
{
    double length = 0.0;

    for (std::size_t i = 1; i < path.size(); ++i)
    {
        length += path[i - 1].Distance(path[i]);
    }

    return length;
}

double DistancePointToSegment(
    const Vec3& point,
    const Vec3& start,
    const Vec3& end)
{
    const Vec3 segment = end - start;
    const double segmentLengthSquared = segment.SquaredLength();

    if (segmentLengthSquared <= kEpsilon)
    {
        return point.Distance(start);
    }

    const double t = std::clamp(
        (point - start).Dot(segment) / segmentLengthSquared,
        0.0,
        1.0);
    return point.Distance(start + segment * t);
}

double DistancePointToPolyline(
    const Vec3& point,
    const std::vector<Vec3>& polyline)
{
    double best = std::numeric_limits<double>::max();

    for (std::size_t i = 1; i < polyline.size(); ++i)
    {
        best = std::min(
            best,
            DistancePointToSegment(point, polyline[i - 1], polyline[i]));
    }

    return best;
}

double DiscreteCurvature(
    const Vec3& previous,
    const Vec3& current,
    const Vec3& next)
{
    const double a = previous.Distance(current);
    const double b = current.Distance(next);
    const double c = previous.Distance(next);

    if (a <= kEpsilon || b <= kEpsilon || c <= kEpsilon)
    {
        return 0.0;
    }

    const Vec3 cross = (current - previous).Cross(next - previous);
    return 2.0 * Length(cross) / (a * b * c);
}

std::size_t SegmentSampleCount(
    const Vec3& start,
    const Vec3& end,
    const GeometryPathSmoothingOptions& options)
{
    std::size_t count = std::max<std::size_t>(1, options.samplesPerSegment);

    if (options.sampleSpacing > kEpsilon)
    {
        count = std::max<std::size_t>(
            count,
            static_cast<std::size_t>(
                std::ceil(start.Distance(end) / options.sampleSpacing)));
    }

    return count;
}

Vec3 CatmullRom(
    const Vec3& p0,
    const Vec3& p1,
    const Vec3& p2,
    const Vec3& p3,
    double t)
{
    const double t2 = t * t;
    const double t3 = t2 * t;

    return (p1 * 2.0 +
        (p2 - p0) * t +
        (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2 +
        (p0 * -1.0 + p1 * 3.0 - p2 * 3.0 + p3) * t3) *
        0.5;
}

bool IsValidInput(
    const std::vector<Vec3>& rawPath,
    const GeometryQueryContext& queryContext,
    const GeometryPathSmoothingOptions& options,
    std::string& message)
{
    if (rawPath.size() < 2)
    {
        message = "Path smoothing requires at least two path points.";
        return false;
    }

    if (!queryContext.IsValid())
    {
        message = "Path smoothing requires a valid geometry query context.";
        return false;
    }

    if (!IsFiniteNonNegative(options.clearance) ||
        !IsFiniteNonNegative(options.radius) ||
        !IsFiniteNonNegative(options.maxDeviation) ||
        !IsFiniteNonNegative(options.maxCurvature) ||
        !IsFiniteNonNegative(options.sampleSpacing) ||
        options.samplesPerSegment == 0)
    {
        message = "Path smoothing received invalid options.";
        return false;
    }

    for (const Vec3& point : rawPath)
    {
        if (!point.IsFinite())
        {
            message = "Path smoothing requires finite path points.";
            return false;
        }
    }

    return true;
}

std::vector<Vec3> BuildCatmullRomSamples(
    const std::vector<Vec3>& rawPath,
    const GeometryPathSmoothingOptions& options)
{
    std::vector<Vec3> sampled;
    sampled.push_back(rawPath.front());

    for (std::size_t i = 0; i + 1 < rawPath.size(); ++i)
    {
        const Vec3& p0 = rawPath[i == 0 ? i : i - 1];
        const Vec3& p1 = rawPath[i];
        const Vec3& p2 = rawPath[i + 1];
        const Vec3& p3 = rawPath[i + 2 < rawPath.size() ? i + 2 : i + 1];
        const std::size_t sampleCount =
            SegmentSampleCount(p1, p2, options);

        for (std::size_t sample = 1; sample <= sampleCount; ++sample)
        {
            const double t =
                static_cast<double>(sample) / static_cast<double>(sampleCount);
            sampled.push_back(CatmullRom(p0, p1, p2, p3, t));
        }
    }

    sampled.front() = rawPath.front();
    sampled.back() = rawPath.back();
    return sampled;
}
}

GeometryPathSmoothingResult SmoothPathCatmullRom(
    const std::vector<Vec3>& rawPath,
    GeometryQueryContext& queryContext,
    const GeometryPathSmoothingOptions& options)
{
    GeometryPathSmoothingResult result;
    result.profile.rawPointCount = rawPath.size();
    result.profile.rawLength = PathLength(rawPath);

    std::string invalidMessage;
    if (!IsValidInput(rawPath, queryContext, options, invalidMessage))
    {
        result.status = GeometryPathSmoothStatus::InvalidInput;
        result.message = invalidMessage;
        return result;
    }

    result.path = BuildCatmullRomSamples(rawPath, options);
    result.profile.smoothedPointCount = result.path.size();
    result.profile.smoothedLength = PathLength(result.path);
    result.profile.minClearance = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < result.path.size(); ++i)
    {
        const ClosestPointResult closest = queryContext.ClosestPoint(result.path[i]);
        ++result.profile.sampleClearanceCheckCount;

        if (!closest.hit)
        {
            result.status = GeometryPathSmoothStatus::FailedConstraint;
            result.message = "Smoothed path sample clearance query missed mesh.";
            return result;
        }

        result.profile.minClearance =
            std::min(result.profile.minClearance, closest.distance);

        const double deviation =
            DistancePointToPolyline(result.path[i], rawPath);
        result.profile.maxDeviation =
            std::max(result.profile.maxDeviation, deviation);

        if (options.maxDeviation > 0.0 &&
            deviation > options.maxDeviation)
        {
            result.status = GeometryPathSmoothStatus::FailedConstraint;
            result.message = "Smoothed path exceeds max deviation.";
            return result;
        }

        if (closest.distance + kEpsilon < options.clearance + options.radius)
        {
            result.status = GeometryPathSmoothStatus::FailedConstraint;
            result.message = "Smoothed path sample violates clearance.";
            return result;
        }

        if (i > 0)
        {
            const SegmentClearanceResult segment =
                queryContext.SegmentClearance(
                    result.path[i - 1],
                    result.path[i],
                    options.clearance,
                    options.radius);
            ++result.profile.collisionCheckCount;

            if (!segment.pass)
            {
                result.status = GeometryPathSmoothStatus::FailedConstraint;
                result.message = "Smoothed path segment violates clearance.";
                return result;
            }
        }
    }

    for (std::size_t i = 1; i + 1 < result.path.size(); ++i)
    {
        const double curvature =
            DiscreteCurvature(result.path[i - 1], result.path[i], result.path[i + 1]);
        result.profile.maxCurvature =
            std::max(result.profile.maxCurvature, curvature);

        if (options.maxCurvature > 0.0 &&
            curvature > options.maxCurvature)
        {
            result.status = GeometryPathSmoothStatus::FailedConstraint;
            result.message = "Smoothed path exceeds max curvature.";
            return result;
        }
    }

    result.status = GeometryPathSmoothStatus::Succeeded;
    result.message = "Path smoothing succeeded.";
    return result;
}

} // namespace movement_path::geometry
