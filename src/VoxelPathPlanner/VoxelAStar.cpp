#include "VoxelAStar.h"

#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>

// ============================================================
// open set 节点
// ============================================================

struct VoxelAStarOpenNode
{
    VoxelIndex index;
    double fCost = std::numeric_limits<double>::max();
    double hCost = 0.0;
};

struct VoxelAStarOpenNodeGreater
{
    bool operator()(
        const VoxelAStarOpenNode& a,
        const VoxelAStarOpenNode& b) const
    {
        if (a.fCost == b.fCost)
        {
            return a.hCost > b.hCost;
        }

        return a.fCost > b.fCost;
    }
};

static void EnsureCellBuilt(
    VoxelSpace& space,
    const VoxelIndex& index,
    const VoxelAStarOptions& options)
{
    if (options.ensureCellBuilt)
    {
        options.ensureCellBuilt(space, index);
    }
}

// ============================================================
// 找最近可通行体素
// ============================================================

bool VoxelAStar::FindNearestWalkableIndex(
    VoxelSpace& space,
    const VoxelIndex& seed,
    const VoxelAStarOptions& options,
    VoxelIndex& outIndex)
{
    const int maxRadius = options.snapMaxRadius;

    if (maxRadius < 0)
    {
        return false;
    }

    if (space.IsInsideSearchBounds(seed))
    {
        EnsureCellBuilt(space, seed, options);
    }

    if (space.IsInsideSearchBounds(seed) &&
        VoxelWalkability::IsStateWalkable(
            space.GetCellState(seed),
            options.searchMode))
    {
        outIndex = seed;
        return true;
    }

    double bestDist2 = std::numeric_limits<double>::max();
    bool found = false;

    for (int r = 1; r <= maxRadius; ++r)
    {
        for (int dx = -r; dx <= r; ++dx)
        {
            for (int dy = -r; dy <= r; ++dy)
            {
                for (int dz = -r; dz <= r; ++dz)
                {
                    // 只检查当前立方壳层，避免重复检查内部
                    if (std::abs(dx) != r &&
                        std::abs(dy) != r &&
                        std::abs(dz) != r)
                    {
                        continue;
                    }

                    VoxelIndex index(
                        seed.x + dx,
                        seed.y + dy,
                        seed.z + dz
                    );

                    if (!space.IsInsideSearchBounds(index))
                    {
                        continue;
                    }

                    EnsureCellBuilt(space, index, options);

                    VoxelState state = space.GetCellState(index);

                    if (!VoxelWalkability::IsStateWalkable(
                        state,
                        options.searchMode))
                    {
                        continue;
                    }

                    const double dist2 =
                        static_cast<double>(dx * dx + dy * dy + dz * dz);

                    if (dist2 < bestDist2)
                    {
                        bestDist2 = dist2;
                        outIndex = index;
                        found = true;
                    }
                }
            }
        }

        if (found)
        {
            return true;
        }
    }

    return false;
}

// ============================================================
// 启发函数
// ============================================================

double VoxelAStar::Heuristic(
    const VoxelSpace& space,
    const VoxelIndex& a,
    const VoxelIndex& b)
{
    return space.GetMoveCost(a, b);
}

// ============================================================
// 判断方向是否改变
// ============================================================

bool VoxelAStar::HasDirectionChanged(
    const VoxelIndex& prev,
    const VoxelIndex& curr,
    const VoxelIndex& next)
{
    const int dx1 = curr.x - prev.x;
    const int dy1 = curr.y - prev.y;
    const int dz1 = curr.z - prev.z;

    const int dx2 = next.x - curr.x;
    const int dy2 = next.y - curr.y;
    const int dz2 = next.z - curr.z;

    return dx1 != dx2 || dy1 != dy2 || dz1 != dz2;
}

// ============================================================
// 回溯路径
// ============================================================

std::vector<VoxelIndex> VoxelAStar::ReconstructPath(
    const VoxelSpace& space,
    const VoxelIndex& startIndex,
    const VoxelIndex& goalIndex)
{
    std::vector<VoxelIndex> path;

    VoxelIndex current = goalIndex;

    while (true)
    {
        path.push_back(current);

        if (current == startIndex)
        {
            break;
        }

        const VoxelCell* cell = space.FindCell(current);

        if (cell == nullptr || !cell->hasParent)
        {
            path.clear();
            return path;
        }

        current = cell->parent;
    }

    std::reverse(path.begin(), path.end());

    return path;
}

// ============================================================
// VoxelIndex path -> vec path
// ============================================================

std::vector<Vec> VoxelAStar::ConvertPathToPoints(
    const VoxelSpace& space,
    const std::vector<VoxelIndex>& voxelPath)
{
    std::vector<Vec> points;
    points.reserve(voxelPath.size());

    for (const VoxelIndex& index : voxelPath)
    {
        points.push_back(space.IndexToCenter(index));
    }

    return points;
}

// ============================================================
// 标记路径
// ============================================================

void VoxelAStar::MarkPath(
    VoxelSpace& space,
    const std::vector<VoxelIndex>& voxelPath,
    const VoxelIndex& startIndex,
    const VoxelIndex& goalIndex)
{
    for (const VoxelIndex& index : voxelPath)
    {
        if (index == startIndex)
        {
            space.SetCellState(index, VoxelState::Start);
        }
        else if (index == goalIndex)
        {
            space.SetCellState(index, VoxelState::Goal);
        }
        else
        {
            VoxelState state = space.GetCellState(index);

            if (state != VoxelState::Occupied)
            {
                space.SetCellState(index, VoxelState::Path);
            }
        }
    }
}

bool VoxelAStar::FindNearestWalkableIndexWithDirection(
    VoxelSpace& space,
    const VoxelIndex& seed,
    const Vec& seedPoint,
    const Vec& preferredDirection,
    const VoxelAStarOptions& options,
    VoxelIndex& outIndex)
{
    const int maxRadius = options.snapMaxRadius;

    if (maxRadius < 0)
    {
        return false;
    }

    Vec dir = preferredDirection;

    if (dir.SquareMagnitude() <= 1.0e-20)
    {
        return FindNearestWalkableIndex(
            space,
            seed,
            options,
            outIndex
        );
    }

    dir.Normalize();

    double bestScore = std::numeric_limits<double>::max();
    bool found = false;

    for (int r = 0; r <= maxRadius; ++r)
    {
        for (int dx = -r; dx <= r; ++dx)
        {
            for (int dy = -r; dy <= r; ++dy)
            {
                for (int dz = -r; dz <= r; ++dz)
                {
                    if (r > 0)
                    {
                        // 只检查当前外壳层
                        if (std::abs(dx) != r &&
                            std::abs(dy) != r &&
                            std::abs(dz) != r)
                        {
                            continue;
                        }
                    }

                    VoxelIndex index(
                        seed.x + dx,
                        seed.y + dy,
                        seed.z + dz
                    );

                    if (!space.IsInsideSearchBounds(index))
                    {
                        continue;
                    }

                    EnsureCellBuilt(space, index, options);

                    VoxelState state = space.GetCellState(index);

                    if (!VoxelWalkability::IsStateWalkable(
                        state,
                        options.searchMode))
                    {
                        continue;
                    }

                    Vec candidateCenter = space.IndexToCenter(index);
                    Vec offset(seedPoint, candidateCenter);

                    if (offset.SquareMagnitude() <= 1.0e-20)
                    {
                        continue;
                    }

                    // 要求候选体素大致位于指定方向的半空间内
                    const double dirDot = offset.Dot(dir);

                    if (dirDot <= 0.0)
                    {
                        continue;
                    }

                    const double dist2 =
                        offset.SquareMagnitude();

                    // 分数：距离越近越好，方向越一致越好
                    // dirDot 越大表示越沿 preferredDirection
                    const double score = dist2 - 0.25 * dirDot * dirDot;

                    if (score < bestScore)
                    {
                        bestScore = score;
                        outIndex = index;
                        found = true;
                    }
                }
            }
        }

        if (found)
        {
            return true;
        }
    }

    return false;
}

// ============================================================
// A* 主入口
// ============================================================

bool VoxelAStar::FindFirstWalkableIndexAlongDirection(
    VoxelSpace& space,
    const VoxelIndex& seed,
    const Vec& seedPoint,
    const Vec& preferredDirection,
    const VoxelAStarOptions& options,
    VoxelIndex& outIndex)
{
    const int maxRadius = options.snapMaxRadius;

    if (maxRadius < 0)
    {
        return false;
    }

    Vec dir = preferredDirection;

    if (dir.SquareMagnitude() <= 1.0e-20)
    {
        return FindNearestWalkableIndex(
            space,
            seed,
            options,
            outIndex
        );
    }

    dir.Normalize();

    const double voxelSize = space.GetVoxelSize();
    const double maxDistance =
        static_cast<double>(maxRadius) * voxelSize;
    const double step = std::max(voxelSize * 0.25, 1.0e-6);

    VoxelIndex lastIndex(
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min()
    );

    for (double distance = 0.0;
        distance <= maxDistance + 1.0e-9;
        distance += step)
    {
        Vec samplePoint = seedPoint + dir * distance;
        VoxelIndex index = space.WorldToIndex(samplePoint);

        if (index == lastIndex)
        {
            continue;
        }

        lastIndex = index;

        if (!space.IsInsideSearchBounds(index))
        {
            continue;
        }

        EnsureCellBuilt(space, index, options);

        if (VoxelWalkability::IsStateWalkable(
            space.GetCellState(index),
            options.searchMode))
        {
            outIndex = index;
            return true;
        }
    }

    return false;
}

VoxelAStarResult VoxelAStar::Search(
    VoxelSpace& space,
    const Vec& startPoint,
    const Vec& goalPoint,
    const VoxelAStarOptions& options)
{
    VoxelAStarResult result;

    if (!space.IsValid())
    {
        result.failReason = VoxelAStarFailReason::InvalidVoxelSpace;
        return result;
    }

    VoxelIndex startIndex = space.WorldToIndex(startPoint);
    VoxelIndex goalIndex = space.WorldToIndex(goalPoint);

    result.inputStartIndex = startIndex;
    result.inputGoalIndex = goalIndex;

    // 先赋值，避免失败时显示 0,0,0
    result.startIndex = startIndex;
    result.goalIndex = goalIndex;

    if (!space.IsInsideSearchBounds(startIndex) ||
        !space.IsInsideSearchBounds(goalIndex))
    {
        result.failReason = VoxelAStarFailReason::StartOrGoalOutsideBounds;
        return result;
    }

    if (options.snapStartGoalToWalkable)
    {
        VoxelIndex snappedStart;
        VoxelIndex snappedGoal;

        bool startOk = false;
        bool goalOk = false;

        if (options.useStartSnapDirection)
        {
            if (options.forceStartSnapAlongDirection)
            {
                startOk = FindFirstWalkableIndexAlongDirection(
                    space,
                    startIndex,
                    startPoint,
                    options.startSnapDirection,
                    options,
                    snappedStart
                );
            }
            else
            {
                startOk = FindNearestWalkableIndexWithDirection(
                    space,
                    startIndex,
                    startPoint,
                    options.startSnapDirection,
                    options,
                    snappedStart
                );
            }
        }
        else
        {
            startOk = FindNearestWalkableIndex(
                space,
                startIndex,
                options,
                snappedStart
            );
        }

        result.startIndex = snappedStart;
        result.startSnapped = startOk && snappedStart != startIndex;

        if (!startOk)
        {
            result.failReason = VoxelAStarFailReason::SnapStartFailed;
            return result;
        }

        if (options.useGoalSnapDirection)
        {
            if (options.forceGoalSnapAlongDirection)
            {
                goalOk = FindFirstWalkableIndexAlongDirection(
                    space,
                    goalIndex,
                    goalPoint,
                    options.goalSnapDirection,
                    options,
                    snappedGoal
                );
            }
            else
            {
                goalOk = FindNearestWalkableIndexWithDirection(
                    space,
                    goalIndex,
                    goalPoint,
                    options.goalSnapDirection,
                    options,
                    snappedGoal
                );
            }
        }
        else
        {
            goalOk = FindNearestWalkableIndex(
                space,
                goalIndex,
                options,
                snappedGoal
            );
        }

        result.goalIndex = snappedGoal;
        result.goalSnapped = goalOk && snappedGoal != goalIndex;

        if (!goalOk)
        {
            result.failReason = VoxelAStarFailReason::SnapGoalFailed;
            return result;
        }

        startIndex = snappedStart;
        goalIndex = snappedGoal;
    }
    else
    {
        EnsureCellBuilt(space, startIndex, options);

        if (!VoxelWalkability::IsStateWalkable(
            space.GetCellState(startIndex),
            options.searchMode))
        {
            result.failReason = VoxelAStarFailReason::StartNotWalkable;
            return result;
        }

        EnsureCellBuilt(space, goalIndex, options);

        if (!VoxelWalkability::IsStateWalkable(
            space.GetCellState(goalIndex),
            options.searchMode))
        {
            result.failReason = VoxelAStarFailReason::GoalNotWalkable;
            return result;
        }
    }

    result.startIndex = startIndex;
    result.goalIndex = goalIndex;

    space.ResetSearchData();

    {
        VoxelCell& startCell = space.GetOrCreateCell(startIndex);
        startCell.state = VoxelState::Start;
        startCell.gCost = 0.0;
        startCell.hCost =
            options.heuristicWeight *
            Heuristic(space, startIndex, goalIndex);
        startCell.fCost = startCell.gCost + startCell.hCost;
        startCell.opened = true;
    }

    if (startIndex != goalIndex)
    {
        VoxelCell& goalCell = space.GetOrCreateCell(goalIndex);
        goalCell.state = VoxelState::Goal;
    }

    std::priority_queue<
        VoxelAStarOpenNode,
        std::vector<VoxelAStarOpenNode>,
        VoxelAStarOpenNodeGreater> openQueue;

    {
        const VoxelCell* startCell = space.FindCell(startIndex);

        VoxelAStarOpenNode node;
        node.index = startIndex;
        node.fCost = startCell->fCost;
        node.hCost = startCell->hCost;

        openQueue.push(node);
    }

    int visitedCount = 0;

    while (!openQueue.empty())
    {
        if (options.shouldCancel && options.shouldCancel())
        {
            result.visitedCount = visitedCount;
            result.failReason = VoxelAStarFailReason::Cancelled;
            return result;
        }

        VoxelAStarOpenNode openNode = openQueue.top();
        openQueue.pop();

        VoxelCell* currentCell = space.FindCell(openNode.index);

        if (currentCell == nullptr)
        {
            continue;
        }

        if (currentCell->closed)
        {
            continue;
        }

        currentCell->closed = true;
        ++visitedCount;

        if (options.maxVisitedCount > 0 &&
            visitedCount > options.maxVisitedCount)
        {
            result.visitedCount = visitedCount;
            result.failReason = VoxelAStarFailReason::MaxVisitedExceeded;
            return result;
        }

        if (openNode.index == goalIndex)
        {
            result.success = true;
            result.visitedCount = visitedCount;
            result.totalCost = currentCell->gCost;

            result.voxelPath =
                ReconstructPath(space, startIndex, goalIndex);

            result.pointPath =
                ConvertPathToPoints(space, result.voxelPath);

            if (options.markPathToVoxelSpace)
            {
                MarkPath(
                    space,
                    result.voxelPath,
                    startIndex,
                    goalIndex
                );
            }

            return result;
        }

        const std::vector<VoxelIndex> neighbors =
            space.GetNeighborIndices(
                openNode.index,
                options.neighborType
            );

        for (const VoxelIndex& neighborIndex : neighbors)
        {
            if (!space.IsInsideSearchBounds(neighborIndex))
            {
                continue;
            }

            const bool isGoal = neighborIndex == goalIndex;
            const bool isStart = neighborIndex == startIndex;

            EnsureCellBuilt(space, neighborIndex, options);

            VoxelState neighborState =
                space.GetCellState(neighborIndex);

            if (!isGoal && !isStart)
            {
                if (!VoxelWalkability::IsStateWalkable(
                    neighborState,
                    options.searchMode))
                {
                    continue;
                }
            }

            VoxelCell& neighborCell =
                space.GetOrCreateCell(neighborIndex);

            if (neighborCell.closed)
            {
                continue;
            }

            double moveCost =
                space.GetMoveCost(openNode.index, neighborIndex);

            if (options.turnPenalty > 0.0 && currentCell->hasParent)
            {
                if (HasDirectionChanged(
                    currentCell->parent,
                    openNode.index,
                    neighborIndex))
                {
                    moveCost += options.turnPenalty;
                }
            }

            const double tentativeG =
                currentCell->gCost + moveCost;

            if (!neighborCell.opened ||
                tentativeG < neighborCell.gCost)
            {
                neighborCell.parent = openNode.index;
                neighborCell.hasParent = true;

                neighborCell.gCost = tentativeG;
                neighborCell.hCost =
                    options.heuristicWeight *
                    Heuristic(space, neighborIndex, goalIndex);
                neighborCell.fCost =
                    neighborCell.gCost + neighborCell.hCost;

                neighborCell.opened = true;

                VoxelAStarOpenNode nextNode;
                nextNode.index = neighborIndex;
                nextNode.fCost = neighborCell.fCost;
                nextNode.hCost = neighborCell.hCost;

                openQueue.push(nextNode);
            }
        }
    }

    result.success = false;
    result.visitedCount = visitedCount;
    result.failReason = VoxelAStarFailReason::OpenSetEmpty;

    return result;
}
