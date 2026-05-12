#include "VoxelPathOptimizer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double kEpsilon = 1.0e-12;

double DistancePointToSegment(
    const Vec& point,
    const Vec& start,
    const Vec& end)
{
    const Vec segment = end - start;
    const double segmentLengthSquared = segment.SquareMagnitude();

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
    const Vec& point,
    const std::vector<Vec>& polyline)
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
        totalSeverity +=
            DirectionChangeSeverity(path[i - 1], path[i], path[i + 1]);
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

double SmoothedPathQualityScore(
    const std::vector<Vec>& path,
    double voxelSize,
    const VoxelPathOptimizeOptions& options)
{
    if (path.size() < 2)
    {
        return std::numeric_limits<double>::max();
    }

    const double length = PathLength(path);
    const double directDistance = std::max(
        path.front().Distance(path.back()),
        kEpsilon);
    const double detour = std::max(0.0, length / directDistance - 1.0);
    const double turnScale = std::max(voxelSize, directDistance * 0.01);

    return length +
        static_cast<double>(SignificantTurnCount(path)) * turnScale *
            options.smoothingSignificantTurnWeight +
        TotalDirectionChangeSeverity(path) * turnScale *
            options.smoothingTotalTurnWeight +
        MaxDirectionChangeSeverity(path) * turnScale *
            options.smoothingMaxTurnWeight +
        detour * directDistance * options.smoothingDetourWeight;
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

std::vector<Vec> BuildChaikinSamples(
    const std::vector<Vec>& controlPath,
    int iterationCount)
{
    std::vector<Vec> smoothed = controlPath;
    constexpr double cutRatio = 0.25;

    for (int iteration = 0; iteration < iterationCount; ++iteration)
    {
        if (smoothed.size() <= 2)
        {
            break;
        }

        std::vector<Vec> next;
        next.reserve(smoothed.size() * 2);
        next.push_back(smoothed.front());

        for (std::size_t i = 0; i + 1 < smoothed.size(); ++i)
        {
            const Vec& p0 = smoothed[i];
            const Vec& p1 = smoothed[i + 1];
            next.push_back(p0 * (1.0 - cutRatio) + p1 * cutRatio);
            next.push_back(p0 * cutRatio + p1 * (1.0 - cutRatio));
        }

        next.push_back(smoothed.back());
        smoothed = std::move(next);
    }

    return smoothed;
}

bool ValidateSmoothedPath(
    const VoxelSpace& space,
    const std::vector<Vec>& controlPath,
    const VoxelPathOptimizeOptions& options,
    const std::vector<Vec>& candidate,
    int& lineCheckCount)
{
    if (candidate.size() < 2)
    {
        return false;
    }

    for (std::size_t i = 0; i < candidate.size(); ++i)
    {
        if (options.maxCurveDeviation > 0.0 &&
            DistancePointToPolyline(candidate[i], controlPath) >
                options.maxCurveDeviation)
        {
            return false;
        }

        if (VoxelWalkability::IsPointRestricted(
                candidate[i],
                options.restrictedHalfSpaces))
        {
            return false;
        }

        const VoxelIndex index = space.WorldToIndex(candidate[i]);
        if (!IsIndexCollisionFree(
                space,
                index,
                options.restrictedHalfSpaces))
        {
            return false;
        }

        if (i > 0)
        {
            ++lineCheckCount;

            const VoxelIndex prevIndex = space.WorldToIndex(candidate[i - 1]);
            if (!IsLineCollisionFree(
                    space,
                    prevIndex,
                    index,
                    options.restrictedHalfSpaces))
            {
                return false;
            }
        }
    }

    return true;
}

struct ControlPathCandidate
{
    std::vector<VoxelIndex> voxelPath;
    std::vector<Vec> pointPath;
};

void AddUniqueControlPathCandidate(
    std::vector<ControlPathCandidate>& controlPaths,
    std::vector<VoxelIndex> voxelPath,
    std::vector<Vec> pointPath)
{
    if (pointPath.size() < 2)
    {
        return;
    }

    for (const ControlPathCandidate& existing : controlPaths)
    {
        if (existing.pointPath.size() != pointPath.size())
        {
            continue;
        }

        bool same = true;
        for (std::size_t i = 0; i < existing.pointPath.size(); ++i)
        {
            if (existing.pointPath[i].Distance(pointPath[i]) > 1.0e-9)
            {
                same = false;
                break;
            }
        }

        if (same)
        {
            return;
        }
    }

    controlPaths.push_back({
        std::move(voxelPath),
        std::move(pointPath)
    });
}

std::vector<VoxelIndex> BuildLineOfSightShortcutPath(
    VoxelSpace& space,
    const std::vector<VoxelIndex>& inputPath,
    const VoxelPathOptimizeOptions& options,
    int maxShortcutLookAhead,
    int& lineCheckCount)
{
    lineCheckCount = 0;

    if (inputPath.size() <= 2)
    {
        return inputPath;
    }

    std::vector<VoxelIndex> optimized;
    optimized.reserve(inputPath.size());
    optimized.push_back(inputPath.front());

    std::size_t i = 0;
    while (i + 1 < inputPath.size())
    {
        const std::size_t lastIndex = inputPath.size() - 1;
        std::size_t maxJ = lastIndex;

        if (maxShortcutLookAhead > 0)
        {
            maxJ = std::min<std::size_t>(
                lastIndex,
                i + static_cast<std::size_t>(maxShortcutLookAhead));
        }

        std::size_t bestJ = i + 1;

        for (std::size_t j = maxJ; j > i + 1; --j)
        {
            if (options.shouldCancel && options.shouldCancel())
            {
                return inputPath;
            }

            ++lineCheckCount;

            if (VoxelPathOptimizer::IsLineWalkable(
                    space,
                    inputPath[i],
                    inputPath[j],
                    options))
            {
                bestJ = j;
                break;
            }
        }

        if (bestJ == i + 1)
        {
            ++lineCheckCount;
            VoxelPathOptimizer::IsLineWalkable(
                space,
                inputPath[i],
                inputPath[i + 1],
                options);
        }

        optimized.push_back(inputPath[bestJ]);
        i = bestJ;
    }

    return optimized;
}
}

// ============================================================
// 小工具
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
// 去除共线体素点
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

std::vector<Vec> VoxelPathOptimizer::SmoothPointPath(
    const VoxelSpace& space,
    const std::vector<Vec>& controlPath,
    const VoxelPathOptimizeOptions& options,
    int& lineCheckCount)
{
    lineCheckCount = 0;

    if (controlPath.size() <= 2 || options.curveSamplesPerSegment <= 0)
    {
        return controlPath;
    }

    std::vector<std::vector<Vec>> candidates;
    candidates.push_back(BuildCatmullRomSamples(controlPath, options));

    for (int iterations = 5; iterations >= 1; --iterations)
    {
        candidates.push_back(BuildChaikinSamples(controlPath, iterations));
    }

    std::vector<Vec> best;
    double bestSeverity = std::numeric_limits<double>::max();
    int bestLineCheckCount = 0;

    for (const std::vector<Vec>& candidate : candidates)
    {
        int candidateLineCheckCount = 0;
        if (!ValidateSmoothedPath(
                space,
                controlPath,
                options,
                candidate,
                candidateLineCheckCount))
        {
            continue;
        }

        const double severity = MaxDirectionChangeSeverity(candidate);
        if (severity < bestSeverity ||
            (std::abs(severity - bestSeverity) <= 1.0e-9 &&
                candidate.size() > best.size()))
        {
            best = candidate;
            bestSeverity = severity;
            bestLineCheckCount = candidateLineCheckCount;
        }
    }

    lineCheckCount = bestLineCheckCount;

    if (!best.empty())
    {
        return best;
    }

    return std::vector<Vec>();
}

// ============================================================
// 直线可通行检测：3D DDA
//
// 检测 from -> to 的体素中心连线是否经过的体素均可通行。
// 注意：这是离散体素层面的 line-of-sight，不是连续几何层面的精确碰撞检测。
// ============================================================

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

    // 数值保护
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

        // 为了避免直线正好穿过体素边/角时漏检，这里处理并列最小 t。
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
// 主优化入口
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

    std::vector<VoxelIndex> workingPath = inputPath;

    if (options.removeCollinear)
    {
        workingPath = RemoveCollinearVoxels(workingPath);
    }

    result.afterCollinearCount = workingPath.size();

    if (!options.enableLineOfSightShortcut ||
        workingPath.size() <= 2)
    {
        result.voxelPath = workingPath;
        result.pointPath = ConvertToPoints(space, result.voxelPath);
        result.outputCount = result.voxelPath.size();
        result.smoothedPointCount = result.pointPath.size();
        result.smoothingSucceeded =
            options.enableCurveSmoothing && result.pointPath.size() <= 2;
        return result;
    }

    std::vector<VoxelIndex> optimized;
    optimized.reserve(workingPath.size());

    std::size_t i = 0;
    optimized.push_back(workingPath.front());

    while (i + 1 < workingPath.size())
    {
        const std::size_t lastIndex = workingPath.size() - 1;

        std::size_t maxJ = lastIndex;

        if (options.maxShortcutLookAhead > 0)
        {
            maxJ = std::min<std::size_t>(
                lastIndex,
                i + static_cast<std::size_t>(options.maxShortcutLookAhead)
            );
        }

        std::size_t bestJ = i + 1;

        // 从远到近尝试，找到最远可直连点
        for (std::size_t j = maxJ; j > i + 1; --j)
        {
            if (options.shouldCancel && options.shouldCancel())
            {
                result.voxelPath = workingPath;
                result.pointPath = ConvertToPoints(space, result.voxelPath);
                result.outputCount = result.voxelPath.size();
                result.smoothedPointCount = result.pointPath.size();
                return result;
            }

            ++result.lineCheckCount;

            if (IsLineWalkable(
                space,
                workingPath[i],
                workingPath[j],
                options))
            {
                bestJ = j;
                break;
            }
        }

        if (bestJ == i + 1)
        {
            ++result.lineCheckCount;

            // 相邻点理论上应可通行；如果失败，也保守保留相邻点。
            IsLineWalkable(
                space,
                workingPath[i],
                workingPath[i + 1],
                options);
        }

        optimized.push_back(workingPath[bestJ]);
        i = bestJ;
    }

    result.voxelPath = optimized;
    result.pointPath = ConvertToPoints(space, result.voxelPath);
    result.outputCount = result.voxelPath.size();
    result.smoothedPointCount = result.pointPath.size();

    if (options.enableCurveSmoothing && result.pointPath.size() <= 2)
    {
        result.smoothingSucceeded = true;
    }
    else if (options.enableCurveSmoothing)
    {
        std::vector<ControlPathCandidate> controlPaths;
        AddUniqueControlPathCandidate(
            controlPaths,
            result.voxelPath,
            result.pointPath);

        const int shortcutWindows[] = { 16, 32, 64, 96, 128 };
        for (int shortcutWindow : shortcutWindows)
        {
            if (options.maxShortcutLookAhead > 0 &&
                shortcutWindow >= options.maxShortcutLookAhead)
            {
                continue;
            }

            int lineCheckCount = 0;
            std::vector<VoxelIndex> shortcutPath =
                BuildLineOfSightShortcutPath(
                    space,
                    workingPath,
                    options,
                    shortcutWindow,
                    lineCheckCount);
            result.lineCheckCount += lineCheckCount;
            AddUniqueControlPathCandidate(
                controlPaths,
                shortcutPath,
                ConvertToPoints(space, shortcutPath));
        }

        AddUniqueControlPathCandidate(
            controlPaths,
            workingPath,
            ConvertToPoints(space, workingPath));
        AddUniqueControlPathCandidate(
            controlPaths,
            inputPath,
            ConvertToPoints(space, inputPath));

        std::vector<Vec> bestSmoothed;
        std::vector<VoxelIndex> bestControlVoxelPath;
        double bestScore = std::numeric_limits<double>::max();
        int bestSmoothingLineCheckCount = 0;

        for (const ControlPathCandidate& controlPath : controlPaths)
        {
            int smoothingLineCheckCount = 0;
            std::vector<Vec> smoothed =
                SmoothPointPath(
                    space,
                    controlPath.pointPath,
                    options,
                    smoothingLineCheckCount);

            result.smoothingLineCheckCount += smoothingLineCheckCount;

            if (smoothed.empty())
            {
                continue;
            }

            const double score =
                SmoothedPathQualityScore(
                    smoothed,
                    space.GetVoxelSize(),
                    options);
            if (score < bestScore ||
                (std::abs(score - bestScore) <= 1.0e-9 &&
                    smoothed.size() > bestSmoothed.size()))
            {
                bestSmoothed = std::move(smoothed);
                bestControlVoxelPath = controlPath.voxelPath;
                bestScore = score;
                bestSmoothingLineCheckCount = smoothingLineCheckCount;
            }
        }

        if (!bestSmoothed.empty())
        {
            if (!bestControlVoxelPath.empty())
            {
                result.voxelPath = bestControlVoxelPath;
                result.outputCount = result.voxelPath.size();
            }
            result.pointPath = bestSmoothed;
            result.smoothedPointCount = result.pointPath.size();
            result.smoothingLineCheckCount = bestSmoothingLineCheckCount;
            result.smoothingSucceeded = true;
        }
    }

    return result;
}
