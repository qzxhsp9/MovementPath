#include "VoxelPathOptimizer.h"

#include <GeomAPI_Interpolate.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Standard_Failure.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double kEpsilon = 1.0e-12;

void PushDistinctPoint(std::vector<Vec>& points, const Vec& point)
{
    if (points.empty() || points.back().Distance(point) > 1.0e-9)
    {
        points.push_back(point);
    }
}

bool TryNormalize(Vec& direction)
{
    if (direction.SquareMagnitude() <= kEpsilon)
    {
        return false;
    }

    direction.Normalize();
    return true;
}

gp_Pnt ToGpPoint(const Vec& point)
{
    return gp_Pnt(point.x, point.y, point.z);
}

Vec FromGpPoint(const gp_Pnt& point)
{
    return Vec(point.X(), point.Y(), point.Z());
}

gp_Vec ToGpVec(const Vec& direction)
{
    return gp_Vec(direction.x, direction.y, direction.z);
}

double DirectionAlignment(
    const Vec& actualDirection,
    const Vec& expectedDirection)
{
    Vec actual = actualDirection;
    Vec expected = expectedDirection;

    if (!TryNormalize(actual) || !TryNormalize(expected))
    {
        return 1.0;
    }

    return std::clamp(actual.Dot(expected), -1.0, 1.0);
}

double EndpointGuideDistance(
    const Vec& endpoint,
    const Vec& nearestSearchPoint,
    const VoxelPathOptimizeOptions& options)
{
    const double searchPointDistance = endpoint.Distance(nearestSearchPoint);
    double guideDistance = searchPointDistance * 0.5;

    if (options.endpointCollisionExemptRadius > kEpsilon)
    {
        guideDistance = options.endpointCollisionExemptRadius * 0.75;
        if (searchPointDistance > kEpsilon)
        {
            guideDistance = std::min(guideDistance, searchPointDistance);
        }
    }

    return std::max(guideDistance, 1.0e-6);
}

bool ShouldSkipEndpointSearchPoint(
    const Vec& endpoint,
    const Vec& endpointToPathDirection,
    const Vec& point,
    double guideDistance)
{
    const Vec offset = point - endpoint;
    const double distanceSquared = offset.SquareMagnitude();
    if (distanceSquared <= kEpsilon)
    {
        return true;
    }

    const double projection = offset.Dot(endpointToPathDirection);
    const double perpendicularSquared =
        std::max(0.0, distanceSquared - projection * projection);
    const double perpendicularDistance = std::sqrt(perpendicularSquared);
    const double distance = std::sqrt(distanceSquared);
    const double lateralTolerance = std::max(guideDistance * 0.35, 1.0e-6);

    // Search endpoints are only connectivity anchors. If they are near the
    // real endpoint or do not advance along the requested tangent, keeping
    // them as smoothing controls bends the curve away from the endpoint
    // direction. The guide distance is only a filtering scale; it is not
    // inserted as an actual curve control point.
    return distance <= guideDistance * 1.25 ||
        projection <= guideDistance * 0.25 ||
        (distance <= guideDistance * 3.0 &&
            perpendicularDistance > lateralTolerance);
}

Vec CentripetalCatmullRom(
    const Vec& p0,
    const Vec& p1,
    const Vec& p2,
    const Vec& p3,
    double t)
{
    auto Knot = [](double previous, const Vec& lhs, const Vec& rhs)
        {
            return previous +
                std::sqrt(std::max(lhs.Distance(rhs), kEpsilon));
        };

    const double t0 = 0.0;
    const double t1 = Knot(t0, p0, p1);
    const double t2 = Knot(t1, p1, p2);
    const double t3 = Knot(t2, p2, p3);
    const double u = t1 + (t2 - t1) * t;

    if (t2 - t1 <= kEpsilon)
    {
        return p1;
    }

    auto Blend = [](const Vec& lhs, const Vec& rhs, double lhsT, double rhsT, double value)
        {
            const double denominator = rhsT - lhsT;
            if (denominator <= kEpsilon)
            {
                return lhs;
            }
            return lhs * ((rhsT - value) / denominator) +
                rhs * ((value - lhsT) / denominator);
        };

    const Vec a1 = Blend(p0, p1, t0, t1, u);
    const Vec a2 = Blend(p1, p2, t1, t2, u);
    const Vec a3 = Blend(p2, p3, t2, t3, u);
    const Vec b1 = Blend(a1, a2, t0, t2, u);
    const Vec b2 = Blend(a2, a3, t1, t3, u);
    return Blend(b1, b2, t1, t2, u);
}

std::size_t SmoothSegmentSampleCount(
    const Vec& start,
    const Vec& end,
    const VoxelPathOptimizeOptions& options)
{
    std::size_t count = static_cast<std::size_t>(
        std::max(1, options.curveSamplesPerSegment));

    if (options.curveSampleSpacing > kEpsilon)
    {
        count = std::max<std::size_t>(
            count,
            static_cast<std::size_t>(
                std::ceil(start.Distance(end) / options.curveSampleSpacing)));
    }

    return count;
}

int LocalSign(int value)
{
    if (value > 0)
    {
        return 1;
    }

    if (value < 0)
    {
        return -1;
    }

    return 0;
}

bool IsIndexCollisionFree(
    const VoxelSpace& space,
    const VoxelIndex& index,
    const std::vector<VoxelRestrictedHalfSpace>& restrictedHalfSpaces)
{
    if (!space.IsInsideSearchBounds(index))
    {
        return false;
    }

    if (VoxelWalkability::IsIndexRestricted(
            space,
            index,
            restrictedHalfSpaces))
    {
        return false;
    }

    return VoxelWalkability::IsStateFreeSpaceWalkable(
        space.GetCellState(index));
}

bool IsLineCollisionFree(
    const VoxelSpace& space,
    const VoxelIndex& from,
    const VoxelIndex& to,
    const std::vector<VoxelRestrictedHalfSpace>& restrictedHalfSpaces)
{
    if (!IsIndexCollisionFree(space, from, restrictedHalfSpaces) ||
        !IsIndexCollisionFree(space, to, restrictedHalfSpaces))
    {
        return false;
    }

    if (from == to)
    {
        return true;
    }

    const Vec p0 = space.IndexToCenter(from);
    const Vec p1 = space.IndexToCenter(to);

    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double dz = p1.z - p0.z;
    const double voxelSize = space.GetVoxelSize();

    if (voxelSize <= 0.0)
    {
        return false;
    }

    VoxelIndex current = from;

    const int stepX = LocalSign(to.x - from.x);
    const int stepY = LocalSign(to.y - from.y);
    const int stepZ = LocalSign(to.z - from.z);
    const double inf = std::numeric_limits<double>::infinity();

    auto ComputeTDelta = [](double d, double cellSize) -> double
        {
            if (std::abs(d) <= 1.0e-15)
            {
                return std::numeric_limits<double>::infinity();
            }
            return std::abs(cellSize / d);
        };

    auto ComputeTMax = [](
        double p,
        double d,
        double nextBoundary) -> double
        {
            if (std::abs(d) <= 1.0e-15)
            {
                return std::numeric_limits<double>::infinity();
            }
            return (nextBoundary - p) / d;
        };

    const Vec origin = space.GetOrigin();

    auto BoundaryForAxis = [](
        double originCoord,
        int indexCoord,
        double cellSize,
        int step) -> double
        {
            if (step > 0)
            {
                return originCoord +
                    static_cast<double>(indexCoord + 1) * cellSize;
            }

            if (step < 0)
            {
                return originCoord +
                    static_cast<double>(indexCoord) * cellSize;
            }

            return 0.0;
        };

    double tMaxX = stepX == 0 ? inf :
        ComputeTMax(p0.x, dx, BoundaryForAxis(origin.x, from.x, voxelSize, stepX));
    double tMaxY = stepY == 0 ? inf :
        ComputeTMax(p0.y, dy, BoundaryForAxis(origin.y, from.y, voxelSize, stepY));
    double tMaxZ = stepZ == 0 ? inf :
        ComputeTMax(p0.z, dz, BoundaryForAxis(origin.z, from.z, voxelSize, stepZ));

    if (tMaxX < 0.0) tMaxX = 0.0;
    if (tMaxY < 0.0) tMaxY = 0.0;
    if (tMaxZ < 0.0) tMaxZ = 0.0;

    const double tDeltaX = ComputeTDelta(dx, voxelSize);
    const double tDeltaY = ComputeTDelta(dy, voxelSize);
    const double tDeltaZ = ComputeTDelta(dz, voxelSize);

    const int maxStepCount =
        std::abs(to.x - from.x) +
        std::abs(to.y - from.y) +
        std::abs(to.z - from.z) +
        3;

    int stepCount = 0;

    while (current != to)
    {
        if (stepCount++ > maxStepCount)
        {
            return false;
        }

        const double tMin = std::min(tMaxX, std::min(tMaxY, tMaxZ));
        const double eps = 1.0e-12;

        if (std::abs(tMaxX - tMin) <= eps)
        {
            current.x += stepX;
            tMaxX += tDeltaX;
        }

        if (std::abs(tMaxY - tMin) <= eps)
        {
            current.y += stepY;
            tMaxY += tDeltaY;
        }

        if (std::abs(tMaxZ - tMin) <= eps)
        {
            current.z += stepZ;
            tMaxZ += tDeltaZ;
        }

        if (!IsIndexCollisionFree(space, current, restrictedHalfSpaces))
        {
            return false;
        }
    }

    return true;
}

bool IsPointInEndpointExemptZone(
    const Vec& point,
    const VoxelPathOptimizeOptions& options)
{
    if (!options.useRealEndpointsForSmoothing ||
        options.endpointCollisionExemptRadius <= 0.0)
    {
        return false;
    }

    return point.Distance(options.realStartPoint) <
            options.endpointCollisionExemptRadius ||
        point.Distance(options.realGoalPoint) <
            options.endpointCollisionExemptRadius;
}

bool IsSmoothedSampleCollisionFree(
    const VoxelSpace& space,
    const Vec& point,
    const VoxelPathOptimizeOptions& options)
{
    if (VoxelWalkability::IsPointRestricted(
            point,
            options.restrictedHalfSpaces))
    {
        return false;
    }

    if (IsPointInEndpointExemptZone(point, options))
    {
        return true;
    }

    return IsIndexCollisionFree(
        space,
        space.WorldToIndex(point),
        options.restrictedHalfSpaces);
}

bool IsSmoothedSegmentCollisionFree(
    const VoxelSpace& space,
    const Vec& from,
    const Vec& to,
    const VoxelPathOptimizeOptions& options)
{
    const double voxelSize = space.GetVoxelSize();
    if (voxelSize <= 0.0)
    {
        return false;
    }

    const double length = from.Distance(to);
    const int sampleCount = std::max(
        1,
        static_cast<int>(std::ceil(length / std::max(voxelSize * 0.5, kEpsilon))));

    for (int sample = 1; sample <= sampleCount; ++sample)
    {
        const double t =
            static_cast<double>(sample) / static_cast<double>(sampleCount);
        const Vec point = from * (1.0 - t) + to * t;
        if (!IsSmoothedSampleCollisionFree(space, point, options))
        {
            return false;
        }
    }

    return true;
}

double DirectionChangeSeverity(
    const Vec& previous,
    const Vec& current,
    const Vec& next)
{
    const Vec incoming = current - previous;
    const Vec outgoing = next - current;
    const double incomingLength = std::sqrt(incoming.SquareMagnitude());
    const double outgoingLength = std::sqrt(outgoing.SquareMagnitude());

    if (incomingLength <= kEpsilon || outgoingLength <= kEpsilon)
    {
        return 0.0;
    }

    const double cosTheta = std::clamp(
        incoming.Dot(outgoing) / (incomingLength * outgoingLength),
        -1.0,
        1.0);
    return 1.0 - cosTheta;
}

double DirectionChangeSeverity(
    const VoxelIndex& previous,
    const VoxelIndex& current,
    const VoxelIndex& next)
{
    return DirectionChangeSeverity(
        Vec(
            static_cast<double>(previous.x),
            static_cast<double>(previous.y),
            static_cast<double>(previous.z)),
        Vec(
            static_cast<double>(current.x),
            static_cast<double>(current.y),
            static_cast<double>(current.z)),
        Vec(
            static_cast<double>(next.x),
            static_cast<double>(next.y),
            static_cast<double>(next.z)));
}

double PathLengthBetween(
    const VoxelSpace& space,
    const std::vector<VoxelIndex>& path,
    std::size_t from,
    std::size_t to)
{
    double length = 0.0;

    for (std::size_t i = from + 1; i <= to; ++i)
    {
        length += space.GetMoveCost(path[i - 1], path[i]);
    }

    return length;
}

double ShortcutScore(
    const VoxelSpace& space,
    const std::vector<VoxelIndex>& path,
    const std::vector<VoxelIndex>& optimized,
    std::size_t from,
    std::size_t to,
    double shortcutTurnPenalty)
{
    const double originalLength = PathLengthBetween(space, path, from, to);
    const double shortcutLength = space.GetMoveCost(path[from], path[to]);
    double score = shortcutLength - originalLength;

    if (shortcutTurnPenalty <= 0.0)
    {
        return score;
    }

    double turnSeverity = 0.0;
    if (optimized.size() >= 2)
    {
        turnSeverity += DirectionChangeSeverity(
            optimized[optimized.size() - 2],
            path[from],
            path[to]);
    }

    if (to + 1 < path.size())
    {
        turnSeverity += DirectionChangeSeverity(
            path[from],
            path[to],
            path[to + 1]);
    }

    return score + shortcutTurnPenalty * turnSeverity;
}

double MaxDirectionChangeSeverity(const std::vector<Vec>& path)
{
    double maxSeverity = 0.0;

    for (std::size_t i = 1; i + 1 < path.size(); ++i)
    {
        maxSeverity = std::max(
            maxSeverity,
            DirectionChangeSeverity(path[i - 1], path[i], path[i + 1]));
    }

    return maxSeverity;
}

double PathLength(const std::vector<Vec>& path)
{
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        length += path[i - 1].Distance(path[i]);
    }
    return length;
}

double TotalDirectionChangeSeverity(const std::vector<Vec>& path)
{
    double totalSeverity = 0.0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i)
    {
        totalSeverity += DirectionChangeSeverity(
            path[i - 1],
            path[i],
            path[i + 1]);
    }
    return totalSeverity;
}

std::size_t SignificantTurnCount(const std::vector<Vec>& path)
{
    std::size_t count = 0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i)
    {
        if (DirectionChangeSeverity(path[i - 1], path[i], path[i + 1]) > 0.02)
        {
            ++count;
        }
    }
    return count;
}

double SmoothingCandidateScore(
    const std::vector<Vec>& candidate,
    const VoxelPathOptimizeOptions& options)
{
    const double length = PathLength(candidate);
    const double directDistance =
        candidate.size() >= 2 ?
            candidate.front().Distance(candidate.back()) :
            0.0;
    const double detour =
        directDistance > kEpsilon ?
            std::max(0.0, length / directDistance - 1.0) :
            0.0;

    return length +
        options.smoothingSignificantTurnWeight *
            static_cast<double>(SignificantTurnCount(candidate)) +
        options.smoothingTotalTurnWeight *
            TotalDirectionChangeSeverity(candidate) +
        options.smoothingMaxTurnWeight *
            MaxDirectionChangeSeverity(candidate) +
        options.smoothingDetourWeight * detour;
}

std::vector<Vec> DensifyPointPath(
    const std::vector<Vec>& path,
    const VoxelPathOptimizeOptions& options,
    double voxelSize)
{
    if (path.size() < 2)
    {
        return path;
    }

    const double spacing =
        options.curveSampleSpacing > 0.0 ?
            options.curveSampleSpacing :
            std::max(voxelSize * 0.25, kEpsilon);

    std::vector<Vec> dense;
    dense.push_back(path.front());

    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const Vec& from = path[i - 1];
        const Vec& to = path[i];
        const double length = from.Distance(to);
        const int sampleCount = std::max(
            1,
            static_cast<int>(std::ceil(length / spacing)));

        for (int sample = 1; sample <= sampleCount; ++sample)
        {
            const double t =
                static_cast<double>(sample) /
                static_cast<double>(sampleCount);
            dense.push_back(from * (1.0 - t) + to * t);
        }
    }

    return dense;
}

std::vector<Vec> BuildCatmullRomSamples(
    const std::vector<Vec>& controlPath,
    const VoxelPathOptimizeOptions& options)
{
    std::vector<Vec> sampled;
    sampled.push_back(controlPath.front());

    for (std::size_t i = 0; i + 1 < controlPath.size(); ++i)
    {
        Vec endpointStartControl;
        Vec endpointEndControl;
        const Vec* p0Override = nullptr;
        const Vec* p3Override = nullptr;

        if (options.useEndpointDirections)
        {
            if (i == 0 && options.startDirection.SquareMagnitude() > kEpsilon)
            {
                Vec dir = options.startDirection;
                dir.Normalize();
                endpointStartControl =
                    controlPath.front() -
                    dir * controlPath.front().Distance(controlPath[1]);
                p0Override = &endpointStartControl;
            }

            if (i + 2 >= controlPath.size() &&
                options.goalDirection.SquareMagnitude() > kEpsilon)
            {
                Vec dir = options.goalDirection;
                dir.Normalize();
                endpointEndControl =
                    controlPath.back() +
                    dir * controlPath[controlPath.size() - 2].Distance(
                        controlPath.back());
                p3Override = &endpointEndControl;
            }
        }

        const Vec& p0 = p0Override != nullptr ?
            *p0Override :
            controlPath[i == 0 ? i : i - 1];
        const Vec& p1 = controlPath[i];
        const Vec& p2 = controlPath[i + 1];
        const Vec& p3 = p3Override != nullptr ?
            *p3Override :
            controlPath[i + 2 < controlPath.size() ? i + 2 : i + 1];
        const std::size_t sampleCount =
            SmoothSegmentSampleCount(p1, p2, options);

        for (std::size_t sample = 1; sample <= sampleCount; ++sample)
        {
            const double t =
                static_cast<double>(sample) / static_cast<double>(sampleCount);
            sampled.push_back(CentripetalCatmullRom(p0, p1, p2, p3, t));
        }
    }

    sampled.front() = controlPath.front();
    sampled.back() = controlPath.back();
    return sampled;
}

std::vector<Vec> BuildOcctBSplineSamples(
    const std::vector<Vec>& controlPath,
    const VoxelPathOptimizeOptions& options)
{
    if (controlPath.size() < 2)
    {
        return std::vector<Vec>();
    }

    if (controlPath.size() == 2)
    {
        return controlPath;
    }

    try
    {
        Handle(TColgp_HArray1OfPnt) points =
            new TColgp_HArray1OfPnt(
                1,
                static_cast<Standard_Integer>(controlPath.size()));
        for (std::size_t i = 0; i < controlPath.size(); ++i)
        {
            points->SetValue(
                static_cast<Standard_Integer>(i + 1),
                ToGpPoint(controlPath[i]));
        }

        GeomAPI_Interpolate interpolate(points, Standard_False, 1.0e-7);
        if (options.useEndpointDirections)
        {
            const bool hasStartDirection =
                options.startDirection.SquareMagnitude() > kEpsilon;
            const bool hasGoalDirection =
                options.goalDirection.SquareMagnitude() > kEpsilon;

            if (hasStartDirection && hasGoalDirection)
            {
                Vec startDirection = options.startDirection;
                Vec goalDirection = options.goalDirection;
                startDirection.Normalize();
                goalDirection.Normalize();
                const double tangentScale =
                    std::max(
                        controlPath.front().Distance(controlPath[1]),
                        controlPath[controlPath.size() - 2].Distance(
                            controlPath.back()));
                interpolate.Load(
                    ToGpVec(startDirection * tangentScale),
                    ToGpVec(goalDirection * tangentScale),
                    Standard_True);
            }
        }

        interpolate.Perform();
        if (!interpolate.IsDone())
        {
            return std::vector<Vec>();
        }

        Handle(Geom_BSplineCurve) curve = interpolate.Curve();
        if (curve.IsNull())
        {
            return std::vector<Vec>();
        }

        std::size_t segmentSamples = 0;
        for (std::size_t i = 1; i < controlPath.size(); ++i)
        {
            segmentSamples += SmoothSegmentSampleCount(
                controlPath[i - 1],
                controlPath[i],
                options);
        }

        const std::size_t sampleCount =
            std::max<std::size_t>(segmentSamples, 1);

        const double first = curve->FirstParameter();
        const double last = curve->LastParameter();
        std::vector<Vec> sampled;
        sampled.reserve(sampleCount + 1);
        for (std::size_t i = 0; i <= sampleCount; ++i)
        {
            const double t =
                static_cast<double>(i) / static_cast<double>(sampleCount);
            gp_Pnt point;
            curve->D0(first * (1.0 - t) + last * t, point);
            sampled.push_back(FromGpPoint(point));
        }

        sampled.front() = controlPath.front();
        sampled.back() = controlPath.back();
        return sampled;
    }
    catch (const Standard_Failure&)
    {
        return std::vector<Vec>();
    }
}

bool ValidateSmoothedPath(
    const VoxelSpace& space,
    const std::vector<Vec>& controlPath,
    const VoxelPathOptimizeOptions& options,
    const std::vector<Vec>& candidate,
    int& lineCheckCount,
    std::string* failureReason = nullptr)
{
    if (candidate.size() < 2)
    {
        if (failureReason != nullptr)
        {
            *failureReason = "too few samples";
        }
        return false;
    }

    if (options.useEndpointDirections)
    {
        const double minAlignment = std::clamp(
            options.minEndpointDirectionAlignment,
            -1.0,
            1.0);

        if (options.startDirection.SquareMagnitude() > kEpsilon)
        {
            const double startAlignment =
                DirectionAlignment(
                    Vec(candidate.front(), candidate[1]),
                    options.startDirection);
            if (startAlignment < minAlignment)
            {
                if (failureReason != nullptr)
                {
                    *failureReason = "start direction misaligned";
                }
                return false;
            }
        }

        if (options.goalDirection.SquareMagnitude() > kEpsilon)
        {
            const double goalAlignment =
                DirectionAlignment(
                    Vec(candidate[candidate.size() - 2], candidate.back()),
                    options.goalDirection);
            if (goalAlignment < minAlignment)
            {
                if (failureReason != nullptr)
                {
                    *failureReason = "goal direction misaligned";
                }
                return false;
            }
        }
    }

    for (std::size_t i = 0; i < candidate.size(); ++i)
    {
        if (!IsSmoothedSampleCollisionFree(space, candidate[i], options))
        {
            if (failureReason != nullptr)
            {
                *failureReason = "sample blocked";
            }
            return false;
        }

        if (i > 0)
        {
            ++lineCheckCount;

            if (!IsSmoothedSegmentCollisionFree(
                    space,
                    candidate[i - 1],
                    candidate[i],
                    options))
            {
                if (failureReason != nullptr)
                {
                    *failureReason = "segment blocked";
                }
                return false;
            }
        }
    }

    return true;
}

std::vector<Vec> BuildSmoothingControlPath(
    const std::vector<Vec>& path,
    const VoxelPathOptimizeOptions& options)
{
    if (!options.useRealEndpointsForSmoothing || path.empty())
    {
        return path;
    }

    constexpr double epsilon = 1.0e-9;
    std::size_t startIndex = 0;
    std::size_t endIndex = path.size();
    std::vector<Vec> controlPath;
    controlPath.reserve(path.size() + 4);

    PushDistinctPoint(controlPath, options.realStartPoint);

    if (options.useEndpointDirections)
    {
        Vec startDirection = options.startDirection;
        if (TryNormalize(startDirection))
        {
            const double startGuideDistance =
                EndpointGuideDistance(
                    options.realStartPoint,
                    path.front(),
                    options);

            while (startIndex < endIndex &&
                ShouldSkipEndpointSearchPoint(
                    options.realStartPoint,
                    startDirection,
                    path[startIndex],
                    startGuideDistance))
            {
                ++startIndex;
            }
        }
    }

    Vec goalDirection = options.goalDirection;
    const bool hasGoalDirection =
        options.useEndpointDirections && TryNormalize(goalDirection);
    Vec goalToPathDirection;
    double goalGuideDistance = 0.0;

    if (hasGoalDirection)
    {
        goalToPathDirection = goalDirection * -1.0;
        goalGuideDistance =
            EndpointGuideDistance(
                options.realGoalPoint,
                path.back(),
                options);

        while (endIndex > startIndex &&
            ShouldSkipEndpointSearchPoint(
                options.realGoalPoint,
                goalToPathDirection,
                path[endIndex - 1],
                goalGuideDistance))
        {
            --endIndex;
        }
    }

    for (std::size_t i = startIndex; i < endIndex; ++i)
    {
        PushDistinctPoint(controlPath, path[i]);
    }

    PushDistinctPoint(controlPath, options.realGoalPoint);

    if (controlPath.size() == 1 && path.front().Distance(path.back()) > epsilon)
    {
        PushDistinctPoint(controlPath, path.back());
        PushDistinctPoint(controlPath, options.realGoalPoint);
    }

    return controlPath;
}

}

// ============================================================
// Small helpers
// ============================================================

int VoxelPathOptimizer::Sign(int v)
{
    if (v > 0)
    {
        return 1;
    }

    if (v < 0)
    {
        return -1;
    }

    return 0;
}

bool VoxelPathOptimizer::SameDirection(
    const VoxelIndex& a,
    const VoxelIndex& b,
    const VoxelIndex& c)
{
    const int dx1 = b.x - a.x;
    const int dy1 = b.y - a.y;
    const int dz1 = b.z - a.z;

    const int dx2 = c.x - b.x;
    const int dy2 = c.y - b.y;
    const int dz2 = c.z - b.z;

    return dx1 == dx2 && dy1 == dy2 && dz1 == dz2;
}

bool VoxelPathOptimizer::IsIndexWalkableForLine(
    const VoxelSpace& space,
    const VoxelIndex& index,
    VoxelAStarSearchMode mode)
{
    return VoxelWalkability::IsIndexWalkable(space, index, mode);
}

namespace
{
bool IsIndexWalkableForLineWithMinDistance(
    VoxelSpace& space,
    const VoxelIndex& index,
    const VoxelPathOptimizeOptions& options)
{
    if (options.ensureCellBuilt)
    {
        options.ensureCellBuilt(space, index);
    }

    return VoxelWalkability::IsIndexWalkableWithMinDistance(
        space,
        index,
        options.searchMode,
        options.minTravelDistanceToSurface,
        options.restrictedHalfSpaces);
}
}

// ============================================================
// Remove intermediate voxels that keep the same discrete step direction.
// ============================================================

std::vector<VoxelIndex> VoxelPathOptimizer::RemoveCollinearVoxels(
    const std::vector<VoxelIndex>& path)
{
    if (path.size() <= 2)
    {
        return path;
    }

    std::vector<VoxelIndex> simplified;
    simplified.reserve(path.size());

    simplified.push_back(path.front());

    for (std::size_t i = 1; i + 1 < path.size(); ++i)
    {
        const VoxelIndex& prev = path[i - 1];
        const VoxelIndex& curr = path[i];
        const VoxelIndex& next = path[i + 1];

        if (!SameDirection(prev, curr, next))
        {
            simplified.push_back(curr);
        }
    }

    simplified.push_back(path.back());

    return simplified;
}

// ============================================================
// VoxelIndex path -> Vec path
// ============================================================

std::vector<Vec> VoxelPathOptimizer::ConvertToPoints(
    const VoxelSpace& space,
    const std::vector<VoxelIndex>& path)
{
    std::vector<Vec> points;
    points.reserve(path.size());

    for (const VoxelIndex& index : path)
    {
        points.push_back(space.IndexToCenter(index));
    }

    return points;
}

namespace
{
struct SmoothPointPathResult
{
    std::vector<Vec> path;
    std::string method = "None";
    int lineCheckCount = 0;
    int candidateCount = 0;
    int acceptedCandidateCount = 0;
    bool catmullRomAccepted = false;
    std::string catmullRomRejectReason;
    double acceptedLength = 0.0;
    double acceptedTotalTurn = 0.0;
    double acceptedMaxTurn = 0.0;
    double startDirectionAlignment = 0.0;
    double goalDirectionAlignment = 0.0;
};

struct SmoothCandidate
{
    std::string method;
    std::vector<Vec> path;
};

void FillAcceptedSmoothingDiagnostics(
    SmoothPointPathResult& result,
    const VoxelPathOptimizeOptions& options,
    int lineCheckCount)
{
    result.lineCheckCount = lineCheckCount;
    result.acceptedLength = PathLength(result.path);
    result.acceptedTotalTurn = TotalDirectionChangeSeverity(result.path);
    result.acceptedMaxTurn = MaxDirectionChangeSeverity(result.path);

    if (result.path.size() >= 2)
    {
        if (options.startDirection.SquareMagnitude() > kEpsilon)
        {
            result.startDirectionAlignment =
                DirectionAlignment(
                    Vec(result.path.front(), result.path[1]),
                    options.startDirection);
        }

        if (options.goalDirection.SquareMagnitude() > kEpsilon)
        {
            result.goalDirectionAlignment =
                DirectionAlignment(
                    Vec(result.path[result.path.size() - 2],
                        result.path.back()),
                    options.goalDirection);
        }
    }
}

SmoothPointPathResult SmoothPointPathDetailed(
    const VoxelSpace& space,
    const std::vector<Vec>& controlPath,
    const VoxelPathOptimizeOptions& options)
{
    SmoothPointPathResult result;

    if (controlPath.size() <= 2 || options.curveSamplesPerSegment <= 0)
    {
        result.method = "Degenerate";
        result.candidateCount = 1;
        int candidateLineCheckCount = 0;
        std::string failureReason;
        if (ValidateSmoothedPath(
                space,
                controlPath,
                options,
                controlPath,
                candidateLineCheckCount,
                &failureReason))
        {
            result.path = controlPath;
            result.acceptedCandidateCount = 1;
            FillAcceptedSmoothingDiagnostics(
                result,
                options,
                candidateLineCheckCount);
        }
        else
        {
            result.catmullRomRejectReason = failureReason;
        }
        return result;
    }

    std::vector<SmoothCandidate> candidates;
    candidates.push_back(SmoothCandidate{
        "CatmullRom",
        BuildCatmullRomSamples(controlPath, options)
    });

    candidates.push_back(SmoothCandidate{
        "OCCT-BSpline",
        BuildOcctBSplineSamples(controlPath, options)
    });

    std::vector<Vec> best;
    double bestScore = std::numeric_limits<double>::max();
    int bestLineCheckCount = 0;
    std::string bestMethod = "None";
    result.candidateCount = static_cast<int>(candidates.size());

    for (const SmoothCandidate& candidate : candidates)
    {
        int candidateLineCheckCount = 0;
        std::string failureReason;
        if (!ValidateSmoothedPath(
                space,
                controlPath,
                options,
                candidate.path,
                candidateLineCheckCount,
                &failureReason))
        {
            if (candidate.method == "CatmullRom")
            {
                result.catmullRomRejectReason = failureReason;
            }
            continue;
        }

        ++result.acceptedCandidateCount;
        if (candidate.method == "CatmullRom")
        {
            result.catmullRomAccepted = true;
            result.path = candidate.path;
            result.method = candidate.method;
            FillAcceptedSmoothingDiagnostics(
                result,
                options,
                candidateLineCheckCount);
            return result;
        }

        const double score = SmoothingCandidateScore(candidate.path, options);
        if (score < bestScore ||
            (std::abs(score - bestScore) <= 1.0e-9 &&
                candidate.path.size() > best.size()))
        {
            best = candidate.path;
            bestScore = score;
            bestLineCheckCount = candidateLineCheckCount;
            bestMethod = candidate.method;
        }
    }

    if (!best.empty())
    {
        result.path = std::move(best);
        result.method = bestMethod;
        FillAcceptedSmoothingDiagnostics(
            result,
            options,
            bestLineCheckCount);
    }

    return result;
}
}

std::vector<Vec> VoxelPathOptimizer::SmoothPointPath(
    const VoxelSpace& space,
    const std::vector<Vec>& controlPath,
    const VoxelPathOptimizeOptions& options,
    int& lineCheckCount)
{
    const SmoothPointPathResult result =
        SmoothPointPathDetailed(space, controlPath, options);
    lineCheckCount = result.lineCheckCount;
    return result.path;
}

// ============================================================
// 直线可通行检测：3D DDA
//
// Check whether the voxel-center line from one voxel to another only crosses walkable voxels.
// This is a discrete voxel line-of-sight check, not an exact continuous geometry collision test.
// ============================================================

namespace
{
struct SmoothingAttemptResult
{
    bool succeeded = false;
    int lineCheckCount = 0;
    std::string sourceName;
    std::vector<VoxelIndex> voxelPath;
    std::vector<Vec> controlPointPath;
    std::vector<Vec> smoothedPath;
    std::string smoothingMethod = "None";
    int smoothingCandidateCount = 0;
    int smoothingAcceptedCandidateCount = 0;
    bool catmullRomAccepted = false;
    std::string catmullRomRejectReason;
    double smoothingAcceptedLength = 0.0;
    double smoothingAcceptedTotalTurn = 0.0;
    double smoothingAcceptedMaxTurn = 0.0;
    double smoothingStartDirectionAlignment = 0.0;
    double smoothingGoalDirectionAlignment = 0.0;
};

SmoothingAttemptResult TryBuildSmoothedCandidate(
    VoxelSpace& space,
    const std::vector<VoxelIndex>& candidateVoxelPath,
    const std::string& sourceName,
    const VoxelPathOptimizeOptions& options)
{
    SmoothingAttemptResult attempt;
    attempt.sourceName = sourceName;
    attempt.voxelPath = candidateVoxelPath;

    const std::vector<Vec> candidatePointPath =
        VoxelPathOptimizer::ConvertToPoints(space, candidateVoxelPath);
    attempt.controlPointPath =
        BuildSmoothingControlPath(candidatePointPath, options);

    const SmoothPointPathResult smoothResult =
        SmoothPointPathDetailed(
            space,
            attempt.controlPointPath,
            options);

    attempt.smoothedPath = smoothResult.path;
    attempt.lineCheckCount = smoothResult.lineCheckCount;
    attempt.smoothingMethod = smoothResult.method;
    attempt.smoothingCandidateCount = smoothResult.candidateCount;
    attempt.smoothingAcceptedCandidateCount =
        smoothResult.acceptedCandidateCount;
    attempt.catmullRomAccepted = smoothResult.catmullRomAccepted;
    attempt.catmullRomRejectReason = smoothResult.catmullRomRejectReason;
    attempt.smoothingAcceptedLength = smoothResult.acceptedLength;
    attempt.smoothingAcceptedTotalTurn = smoothResult.acceptedTotalTurn;
    attempt.smoothingAcceptedMaxTurn = smoothResult.acceptedMaxTurn;
    attempt.smoothingStartDirectionAlignment =
        smoothResult.startDirectionAlignment;
    attempt.smoothingGoalDirectionAlignment =
        smoothResult.goalDirectionAlignment;

    attempt.succeeded = !attempt.smoothedPath.empty();
    return attempt;
}

void AcceptSmoothedCandidate(
    VoxelPathOptimizeResult& result,
    const SmoothingAttemptResult& attempt,
    const VoxelPathOptimizeOptions& options,
    double voxelSize)
{
    result.voxelPath = attempt.voxelPath;
    result.outputCount = result.voxelPath.size();
    result.controlPointPath = attempt.controlPointPath;
    result.pointPath = DensifyPointPath(
        attempt.smoothedPath,
        options,
        voxelSize);

    VoxelPathOptimizeOptions displayOptions = options;
    displayOptions.curveSamplesPerSegment =
        options.displayCurveSamplesPerSegment > 0 ?
            options.displayCurveSamplesPerSegment :
            options.curveSamplesPerSegment;
    result.displayPointPath = DensifyPointPath(
        attempt.smoothedPath,
        displayOptions,
        voxelSize);

    if (options.useRealEndpointsForSmoothing &&
        !result.displayPointPath.empty())
    {
        result.displayPointPath.front() = options.realStartPoint;
        result.displayPointPath.back() = options.realGoalPoint;
    }

    result.smoothedPointCount = result.pointPath.size();
    result.smoothingSucceeded = true;
    result.pathOutputStage = attempt.sourceName + " smoothed";
    result.smoothingMethod = attempt.smoothingMethod;
    result.smoothingCandidateCount = attempt.smoothingCandidateCount;
    result.smoothingAcceptedCandidateCount =
        attempt.smoothingAcceptedCandidateCount;
    result.catmullRomAccepted = attempt.catmullRomAccepted;
    result.catmullRomRejectReason = attempt.catmullRomRejectReason;
    result.smoothingAcceptedLength = attempt.smoothingAcceptedLength;
    result.smoothingAcceptedTotalTurn = attempt.smoothingAcceptedTotalTurn;
    result.smoothingAcceptedMaxTurn = attempt.smoothingAcceptedMaxTurn;
    result.smoothingStartDirectionAlignment =
        attempt.smoothingStartDirectionAlignment;
    result.smoothingGoalDirectionAlignment =
        attempt.smoothingGoalDirectionAlignment;
}

std::vector<VoxelIndex> BuildLineOfSightShortcutPath(
    VoxelSpace& space,
    const std::vector<VoxelIndex>& path,
    const VoxelPathOptimizeOptions& options,
    int& lineCheckCount)
{
    lineCheckCount = 0;

    if (path.size() <= 2 || !options.enableLineOfSightShortcut)
    {
        return path;
    }

    std::vector<VoxelIndex> optimized;
    optimized.reserve(path.size());

    std::size_t i = 0;
    optimized.push_back(path.front());

    while (i + 1 < path.size())
    {
        double bestScore = std::numeric_limits<double>::max();
        const std::size_t lastIndex = path.size() - 1;
        std::size_t maxJ = lastIndex;

        if (options.maxShortcutLookAhead > 0)
        {
            maxJ = std::min<std::size_t>(
                lastIndex,
                i + static_cast<std::size_t>(options.maxShortcutLookAhead));
        }

        std::size_t bestJ = i + 1;

        for (std::size_t j = maxJ; j > i + 1; --j)
        {
            if (options.shouldCancel && options.shouldCancel())
            {
                return path;
            }

            ++lineCheckCount;

            if (VoxelPathOptimizer::IsLineWalkable(
                    space,
                    path[i],
                    path[j],
                    options))
            {
                if (options.shortcutTurnPenalty <= 0.0)
                {
                    bestJ = j;
                    break;
                }

                const double score =
                    ShortcutScore(
                        space,
                        path,
                        optimized,
                        i,
                        j,
                        options.shortcutTurnPenalty);
                if (score < bestScore)
                {
                    bestScore = score;
                    bestJ = j;
                }
            }
        }

        if (bestJ == i + 1)
        {
            ++lineCheckCount;
            VoxelPathOptimizer::IsLineWalkable(
                space,
                path[i],
                path[i + 1],
                options);
        }

        optimized.push_back(path[bestJ]);
        i = bestJ;
    }

    return optimized;
}
}

bool VoxelPathOptimizer::IsLineWalkable(
    const VoxelSpace& space,
    const VoxelIndex& from,
    const VoxelIndex& to,
    VoxelAStarSearchMode mode)
{
    if (!IsIndexWalkableForLine(space, from, mode))
    {
        return false;
    }

    if (!IsIndexWalkableForLine(space, to, mode))
    {
        return false;
    }

    if (from == to)
    {
        return true;
    }

    const Vec p0 = space.IndexToCenter(from);
    const Vec p1 = space.IndexToCenter(to);

    const double x0 = p0.x;
    const double y0 = p0.y;
    const double z0 = p0.z;

    const double x1 = p1.x;
    const double y1 = p1.y;
    const double z1 = p1.z;

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;

    const double voxelSize = space.GetVoxelSize();

    if (voxelSize <= 0.0)
    {
        return false;
    }

    VoxelIndex current = from;

    const int stepX = Sign(to.x - from.x);
    const int stepY = Sign(to.y - from.y);
    const int stepZ = Sign(to.z - from.z);

    const double inf = std::numeric_limits<double>::infinity();

    auto ComputeTDelta = [](double d, double cellSize) -> double
        {
            if (std::abs(d) <= 1.0e-15)
            {
                return std::numeric_limits<double>::infinity();
            }

            return std::abs(cellSize / d);
        };

    auto ComputeTMax = [](
        double p,
        double d,
        double nextBoundary) -> double
        {
            if (std::abs(d) <= 1.0e-15)
            {
                return std::numeric_limits<double>::infinity();
            }

            return (nextBoundary - p) / d;
        };

    const Vec origin = space.GetOrigin();

    auto BoundaryForAxis = [](
        double originCoord,
        int indexCoord,
        double voxelSize,
        int step) -> double
        {
            if (step > 0)
            {
                return originCoord +
                    static_cast<double>(indexCoord + 1) * voxelSize;
            }

            if (step < 0)
            {
                return originCoord +
                    static_cast<double>(indexCoord) * voxelSize;
            }

            return 0.0;
        };

    const double nextX =
        BoundaryForAxis(origin.x, from.x, voxelSize, stepX);
    const double nextY =
        BoundaryForAxis(origin.y, from.y, voxelSize, stepY);
    const double nextZ =
        BoundaryForAxis(origin.z, from.z, voxelSize, stepZ);

    double tMaxX =
        stepX == 0 ? inf : ComputeTMax(x0, dx, nextX);
    double tMaxY =
        stepY == 0 ? inf : ComputeTMax(y0, dy, nextY);
    double tMaxZ =
        stepZ == 0 ? inf : ComputeTMax(z0, dz, nextZ);

    // Numeric guard.
    if (tMaxX < 0.0) tMaxX = 0.0;
    if (tMaxY < 0.0) tMaxY = 0.0;
    if (tMaxZ < 0.0) tMaxZ = 0.0;

    const double tDeltaX = ComputeTDelta(dx, voxelSize);
    const double tDeltaY = ComputeTDelta(dy, voxelSize);
    const double tDeltaZ = ComputeTDelta(dz, voxelSize);

    const int maxStepCount =
        std::abs(to.x - from.x) +
        std::abs(to.y - from.y) +
        std::abs(to.z - from.z) +
        3;

    int stepCount = 0;

    while (current != to)
    {
        if (stepCount++ > maxStepCount)
        {
            return false;
        }

        // Handle tied minimum t values so lines through voxel edges/corners do not skip neighbor cells.
        const double tMin = std::min(tMaxX, std::min(tMaxY, tMaxZ));

        const double eps = 1.0e-12;

        if (std::abs(tMaxX - tMin) <= eps)
        {
            current.x += stepX;
            tMaxX += tDeltaX;
        }

        if (std::abs(tMaxY - tMin) <= eps)
        {
            current.y += stepY;
            tMaxY += tDeltaY;
        }

        if (std::abs(tMaxZ - tMin) <= eps)
        {
            current.z += stepZ;
            tMaxZ += tDeltaZ;
        }

        if (!IsIndexWalkableForLine(space, current, mode))
        {
            return false;
        }
    }

    return true;
}

bool VoxelPathOptimizer::IsLineWalkable(
    VoxelSpace& space,
    const VoxelIndex& from,
    const VoxelIndex& to,
    const VoxelPathOptimizeOptions& options)
{
    if (!IsIndexWalkableForLineWithMinDistance(space, from, options))
    {
        return false;
    }

    if (!IsIndexWalkableForLineWithMinDistance(space, to, options))
    {
        return false;
    }

    if (from == to)
    {
        return true;
    }

    const Vec p0 = space.IndexToCenter(from);
    const Vec p1 = space.IndexToCenter(to);

    const double x0 = p0.x;
    const double y0 = p0.y;
    const double z0 = p0.z;

    const double x1 = p1.x;
    const double y1 = p1.y;
    const double z1 = p1.z;

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;

    const double voxelSize = space.GetVoxelSize();

    if (voxelSize <= 0.0)
    {
        return false;
    }

    VoxelIndex current = from;

    const int stepX = Sign(to.x - from.x);
    const int stepY = Sign(to.y - from.y);
    const int stepZ = Sign(to.z - from.z);

    const double inf = std::numeric_limits<double>::infinity();

    auto ComputeTDelta = [](double d, double cellSize) -> double
        {
            if (std::abs(d) <= 1.0e-15)
            {
                return std::numeric_limits<double>::infinity();
            }

            return std::abs(cellSize / d);
        };

    auto ComputeTMax = [](
        double p,
        double d,
        double nextBoundary) -> double
        {
            if (std::abs(d) <= 1.0e-15)
            {
                return std::numeric_limits<double>::infinity();
            }

            return (nextBoundary - p) / d;
        };

    const Vec origin = space.GetOrigin();

    auto BoundaryForAxis = [](
        double originCoord,
        int indexCoord,
        double voxelSize,
        int step) -> double
        {
            if (step > 0)
            {
                return originCoord +
                    static_cast<double>(indexCoord + 1) * voxelSize;
            }

            if (step < 0)
            {
                return originCoord +
                    static_cast<double>(indexCoord) * voxelSize;
            }

            return 0.0;
        };

    const double nextX =
        BoundaryForAxis(origin.x, from.x, voxelSize, stepX);
    const double nextY =
        BoundaryForAxis(origin.y, from.y, voxelSize, stepY);
    const double nextZ =
        BoundaryForAxis(origin.z, from.z, voxelSize, stepZ);

    double tMaxX =
        stepX == 0 ? inf : ComputeTMax(x0, dx, nextX);
    double tMaxY =
        stepY == 0 ? inf : ComputeTMax(y0, dy, nextY);
    double tMaxZ =
        stepZ == 0 ? inf : ComputeTMax(z0, dz, nextZ);

    if (tMaxX < 0.0) tMaxX = 0.0;
    if (tMaxY < 0.0) tMaxY = 0.0;
    if (tMaxZ < 0.0) tMaxZ = 0.0;

    const double tDeltaX = ComputeTDelta(dx, voxelSize);
    const double tDeltaY = ComputeTDelta(dy, voxelSize);
    const double tDeltaZ = ComputeTDelta(dz, voxelSize);

    const int maxStepCount =
        std::abs(to.x - from.x) +
        std::abs(to.y - from.y) +
        std::abs(to.z - from.z) +
        3;

    int stepCount = 0;

    while (current != to)
    {
        if (options.shouldCancel && options.shouldCancel())
        {
            return false;
        }

        if (stepCount++ > maxStepCount)
        {
            return false;
        }

        const double tMin = std::min(tMaxX, std::min(tMaxY, tMaxZ));
        const double eps = 1.0e-12;

        if (std::abs(tMaxX - tMin) <= eps)
        {
            current.x += stepX;
            tMaxX += tDeltaX;
        }

        if (std::abs(tMaxY - tMin) <= eps)
        {
            current.y += stepY;
            tMaxY += tDeltaY;
        }

        if (std::abs(tMaxZ - tMin) <= eps)
        {
            current.z += stepZ;
            tMaxZ += tDeltaZ;
        }

        if (!IsIndexWalkableForLineWithMinDistance(space, current, options))
        {
            return false;
        }
    }

    return true;
}

// ============================================================
// Main optimization entry point.
// ============================================================

VoxelPathOptimizeResult VoxelPathOptimizer::Optimize(
    VoxelSpace& space,
    const std::vector<VoxelIndex>& inputPath,
    const VoxelPathOptimizeOptions& options)
{
    VoxelPathOptimizeResult result;
    result.inputCount = inputPath.size();

    if (inputPath.empty())
    {
        return result;
    }

    std::vector<VoxelIndex> path2 = inputPath;
    if (options.removeCollinear)
    {
        path2 = RemoveCollinearVoxels(path2);
    }

    result.afterCollinearCount = path2.size();
    result.voxelPath = path2;
    result.pointPath = ConvertToPoints(space, result.voxelPath);
    result.outputCount = result.voxelPath.size();
    result.smoothedPointCount = result.pointPath.size();
    result.pathOutputStage = "Path2";

    if (!options.enableCurveSmoothing)
    {
        return result;
    }

    int shortcutLineCheckCount = 0;
    const std::vector<VoxelIndex> path3 =
        BuildLineOfSightShortcutPath(
            space,
            path2,
            options,
            shortcutLineCheckCount);
    result.lineCheckCount += shortcutLineCheckCount;

    struct SmoothingSource
    {
        const char* name = "";
        const std::vector<VoxelIndex>* path = nullptr;
    };

    std::vector<SmoothingSource> smoothingOrder;
    smoothingOrder.reserve(3);
    auto addSmoothingSource =
        [&smoothingOrder](
            const char* name,
            const std::vector<VoxelIndex>& candidate)
        {
            for (const SmoothingSource& existing : smoothingOrder)
            {
                if (*existing.path == candidate)
                {
                    return;
                }
            }
            smoothingOrder.push_back(SmoothingSource{ name, &candidate });
        };

    if (path3 != path2)
    {
        addSmoothingSource("Path3", path3);
    }
    addSmoothingSource("Path2", path2);
    addSmoothingSource("Path1", inputPath);

    for (const SmoothingSource& candidate : smoothingOrder)
    {
        SmoothingAttemptResult attempt =
            TryBuildSmoothedCandidate(
                space,
                *candidate.path,
                candidate.name,
                options);
        result.smoothingLineCheckCount += attempt.lineCheckCount;

        if (attempt.succeeded)
        {
            AcceptSmoothedCandidate(
                result,
                attempt,
                options,
                space.GetVoxelSize());
            break;
        }
    }

    return result;
}
