#pragma once

#include "VoxelSpace.h"

#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

// ============================================================
// A* 搜索模式
// ============================================================

enum class VoxelAStarSearchMode
{
    // 普通避障模式：
    // Free 可走，Occupied / ClearanceBand 不走。
    FreeSpace,

    // 安全距离层模式：
    // 只在 ClearanceBand 中搜索。
    // Occupied 不可走，Free 默认不走。
    ClearanceBand
};

// ============================================================
// A* 配置
// ============================================================

struct VoxelAStarOptions
{
    VoxelNeighborType neighborType = VoxelNeighborType::Face6;

    // 默认按你的当前需求：路径应在安全距离层中
    VoxelAStarSearchMode searchMode = VoxelAStarSearchMode::ClearanceBand;

    double heuristicWeight = 1.0;

    // 转折惩罚，越大越倾向少转弯。
    double turnPenalty = 0.0;

    // 起点/终点若不在可走体素上，是否吸附到最近可走体素
    bool snapStartGoalToWalkable = true;

    // 吸附最大搜索半径，单位是体素层数
    int snapMaxRadius = 20;

    // 是否使用吸附方向。
    // 用于避免封闭曲面内外侧吸附错误。
    bool useStartSnapDirection = false;
    bool useGoalSnapDirection = false;

    gp_Vec startSnapDirection;
    gp_Vec goalSnapDirection;

    int maxVisitedCount = 0;

    bool markPathToVoxelSpace = true;
};

enum class VoxelAStarFailReason
{
    None = 0,
    InvalidVoxelSpace,
    StartOrGoalOutsideBounds,
    SnapStartFailed,
    SnapGoalFailed,
    StartNotWalkable,
    GoalNotWalkable,
    MaxVisitedExceeded,
    OpenSetEmpty
};

// ============================================================
// A* 结果
// ============================================================

struct VoxelAStarResult
{
    bool success = false;

    VoxelAStarFailReason failReason = VoxelAStarFailReason::None;

    VoxelIndex inputStartIndex;
    VoxelIndex inputGoalIndex;

    VoxelIndex startIndex;
    VoxelIndex goalIndex;

    bool startSnapped = false;
    bool goalSnapped = false;

    std::vector<VoxelIndex> voxelPath;
    std::vector<gp_Pnt> pointPath;

    int visitedCount = 0;

    double totalCost = 0.0;
};

// ============================================================
// A* 路径搜索器
// ============================================================

class VoxelAStar
{
public:
    static VoxelAStarResult Search(
        VoxelSpace& space,
        const gp_Pnt& startPoint,
        const gp_Pnt& goalPoint,
        const VoxelAStarOptions& options = VoxelAStarOptions());

private:
    static bool IsStateWalkableForMode(
        VoxelState state,
        VoxelAStarSearchMode mode);

    static bool FindNearestWalkableIndex(
        const VoxelSpace& space,
        const VoxelIndex& seed,
        VoxelAStarSearchMode mode,
        int maxRadius,
        VoxelIndex& outIndex);

    static double Heuristic(
        const VoxelSpace& space,
        const VoxelIndex& a,
        const VoxelIndex& b);

    static bool HasDirectionChanged(
        const VoxelIndex& prev,
        const VoxelIndex& curr,
        const VoxelIndex& next);

    static std::vector<VoxelIndex> ReconstructPath(
        const VoxelSpace& space,
        const VoxelIndex& startIndex,
        const VoxelIndex& goalIndex);

    static std::vector<gp_Pnt> ConvertPathToPoints(
        const VoxelSpace& space,
        const std::vector<VoxelIndex>& voxelPath);

    static void MarkPath(
        VoxelSpace& space,
        const std::vector<VoxelIndex>& voxelPath,
        const VoxelIndex& startIndex,
        const VoxelIndex& goalIndex);

    static bool FindNearestWalkableIndexWithDirection(
        const VoxelSpace& space,
        const VoxelIndex& seed,
        const gp_Pnt& seedPoint,
        const gp_Vec& preferredDirection,
        VoxelAStarSearchMode mode,
        int maxRadius,
        VoxelIndex& outIndex);
};