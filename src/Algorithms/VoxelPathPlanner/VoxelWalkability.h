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
    // 普通避障模式：
    // Free 可走，Occupied / ClearanceBand 不走。
    // 后续如果需要，也可以改成 Free + ClearanceBand 可走。
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
            return state != VoxelState::Occupied;
        }

        if (mode == VoxelAStarSearchMode::ClearanceBand)
        {
            return state == VoxelState::ClearanceBand;
        }

        return false;
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
