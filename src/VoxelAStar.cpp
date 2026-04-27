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

// ============================================================
// 状态是否可通行
// ============================================================

bool VoxelAStar::IsStateWalkableForMode(
    VoxelState state,
    VoxelAStarSearchMode mode)
{
    if (state == VoxelState::Start ||
        state == VoxelState::Goal ||
        state == VoxelState::Path)
    {
        return true;
    }

    if (mode == VoxelAStarSearchMode::FreeSpace)
    {
        return state == VoxelState::Free;
    }

    if (mode == VoxelAStarSearchMode::ClearanceBand)
    {
        return state == VoxelState::ClearanceBand;
    }

    return false;
}

// ============================================================
// 找最近可通行体素
// ============================================================

bool VoxelAStar::FindNearestWalkableIndex(
    const VoxelSpace& space,
    const VoxelIndex& seed,
    VoxelAStarSearchMode mode,
    int maxRadius,
    VoxelIndex& outIndex)
{
    if (maxRadius < 0)
    {
        return false;
    }

    if (space.IsInsideSearchBounds(seed) &&
        IsStateWalkableForMode(space.GetCellState(seed), mode))
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

                    VoxelState state = space.GetCellState(index);

                    if (!IsStateWalkableForMode(state, mode))
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
// VoxelIndex path -> gp_Pnt path
// ============================================================

std::vector<gp_Pnt> VoxelAStar::ConvertPathToPoints(
    const VoxelSpace& space,
    const std::vector<VoxelIndex>& voxelPath)
{
    std::vector<gp_Pnt> points;
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

// ============================================================
// A* 主入口
// ============================================================

VoxelAStarResult VoxelAStar::Search(
    VoxelSpace& space,
    const gp_Pnt& startPoint,
    const gp_Pnt& goalPoint,
    const VoxelAStarOptions& options)
{
    VoxelAStarResult result;

    if (!space.IsValid())
    {
        return result;
    }

    VoxelIndex startIndex = space.WorldToIndex(startPoint);
    VoxelIndex goalIndex = space.WorldToIndex(goalPoint);

    result.inputStartIndex = startIndex;
    result.inputGoalIndex = goalIndex;

    if (!space.IsInsideSearchBounds(startIndex) ||
        !space.IsInsideSearchBounds(goalIndex))
    {
        return result;
    }

    if (options.snapStartGoalToWalkable)
    {
        VoxelIndex snappedStart;
        VoxelIndex snappedGoal;

        if (!FindNearestWalkableIndex(
            space,
            startIndex,
            options.searchMode,
            options.snapMaxRadius,
            snappedStart))
        {
            return result;
        }

        if (!FindNearestWalkableIndex(
            space,
            goalIndex,
            options.searchMode,
            options.snapMaxRadius,
            snappedGoal))
        {
            return result;
        }

        startIndex = snappedStart;
        goalIndex = snappedGoal;
    }
    else
    {
        if (!IsStateWalkableForMode(
            space.GetCellState(startIndex),
            options.searchMode))
        {
            return result;
        }

        if (!IsStateWalkableForMode(
            space.GetCellState(goalIndex),
            options.searchMode))
        {
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

            VoxelState neighborState =
                space.GetCellState(neighborIndex);

            if (!isGoal && !isStart)
            {
                if (!IsStateWalkableForMode(
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

    return result;
}