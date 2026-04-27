#include "VoxelPathOptimizer.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

// ============================================================
// 主优化入口
// ============================================================

VoxelPathOptimizeResult VoxelPathOptimizer::Optimize(
    const VoxelSpace& space,
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
            ++result.lineCheckCount;

            if (IsLineWalkable(
                space,
                workingPath[i],
                workingPath[j],
                options.searchMode))
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
                options.searchMode);
        }

        optimized.push_back(workingPath[bestJ]);
        i = bestJ;
    }

    result.voxelPath = optimized;
    result.pointPath = ConvertToPoints(space, result.voxelPath);
    result.outputCount = result.voxelPath.size();

    return result;
}