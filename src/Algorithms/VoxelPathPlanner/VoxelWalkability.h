#pragma once

#include "VoxelSpace.h"

#include <cmath>
#include <limits>
#include <vector>

struct VoxelRestrictedHalfSpace
{
    Vec point;
    Vec normal;
    double pointNormalDot = std::numeric_limits<double>::quiet_NaN();
};

// ============================================================
// A* 搜索模式
// ============================================================

enum class VoxelAStarSearchMode
{
    // Standard free-space mode:
    // Free and ClearanceBand are equivalent traversable states.
    FreeSpace,

    // 安全距离层模式：
    // 只在 ClearanceBand 中搜索。
    // Occupied 不可走，Free 默认不走。
    ClearanceBand
};

// ============================================================
// 体素可通行判断
// ============================================================

class VoxelWalkability
{
public:
    static double ComputePointNormalDot(
        const VoxelRestrictedHalfSpace& halfSpace)
    {
        return halfSpace.point.Dot(halfSpace.normal);
    }

    static void CachePointNormalDot(
        VoxelRestrictedHalfSpace& halfSpace)
    {
        halfSpace.pointNormalDot = ComputePointNormalDot(halfSpace);
    }

    static bool IsPointInsideRestrictedHalfSpace(
        const Vec& point,
        const VoxelRestrictedHalfSpace& halfSpace)
    {
        const double pointNormalDot =
            std::isfinite(halfSpace.pointNormalDot) ?
                halfSpace.pointNormalDot :
                ComputePointNormalDot(halfSpace);
        return point.Dot(halfSpace.normal) > pointNormalDot;
    }

    static bool IsPointRestricted(
        const Vec& point,
        const std::vector<VoxelRestrictedHalfSpace>& restrictedHalfSpaces)
    {
        for (const VoxelRestrictedHalfSpace& halfSpace : restrictedHalfSpaces)
        {
            if (IsPointInsideRestrictedHalfSpace(point, halfSpace))
            {
                return true;
            }
        }

        return false;
    }

    static bool IsIndexRestricted(
        const VoxelSpace& space,
        const VoxelIndex& index,
        const std::vector<VoxelRestrictedHalfSpace>& restrictedHalfSpaces)
    {
        return IsPointRestricted(
            space.IndexToCenter(index),
            restrictedHalfSpaces);
    }

    static bool IsStateWalkable(
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
            return IsStateFreeSpaceWalkable(state);
        }

        if (mode == VoxelAStarSearchMode::ClearanceBand)
        {
            return state == VoxelState::ClearanceBand;
        }

        return false;
    }

    static bool IsStateFreeSpaceWalkable(VoxelState state)
    {
        return state == VoxelState::Free ||
            state == VoxelState::ClearanceBand ||
            state == VoxelState::Start ||
            state == VoxelState::Goal ||
            state == VoxelState::Path;
    }

    static bool IsIndexWalkable(
        const VoxelSpace& space,
        const VoxelIndex& index,
        VoxelAStarSearchMode mode)
    {
        if (!space.IsInsideSearchBounds(index))
        {
            return false;
        }

        return IsStateWalkable(space.GetCellState(index), mode);
    }

    static bool IsIndexWalkableWithMinDistance(
        const VoxelSpace& space,
        const VoxelIndex& index,
        VoxelAStarSearchMode mode,
        double minDistanceToSurface,
        const std::vector<VoxelRestrictedHalfSpace>& restrictedHalfSpaces = {})
    {
        if (IsIndexRestricted(space, index, restrictedHalfSpaces))
        {
            return false;
        }

        if (!IsIndexWalkable(space, index, mode))
        {
            return false;
        }

        if (minDistanceToSurface <= 0.0)
        {
            return true;
        }

        if (mode == VoxelAStarSearchMode::ClearanceBand)
        {
            return true;
        }

        if (mode == VoxelAStarSearchMode::FreeSpace)
        {
            return true;
        }

        const VoxelCell* cell = space.FindCell(index);
        if (cell == nullptr)
        {
            return true;
        }

        if (cell->state == VoxelState::Start ||
            cell->state == VoxelState::Goal)
        {
            return true;
        }

        return cell->distanceToSurface >= minDistanceToSurface;
    }
};
