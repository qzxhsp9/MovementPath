#pragma once

#include "VoxelSpace.h"
#include "VoxelWalkability.h"

#include <vector>

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

    Vec startSnapDirection;
    Vec goalSnapDirection;

    int maxVisitedCount = 0;

    bool markPathToVoxelSpace = true;
};

// ============================================================
// A* 失败原因
// ============================================================

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
    std::vector<Vec> pointPath;

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
        const Vec& startPoint,
        const Vec& goalPoint,
        const VoxelAStarOptions& options = VoxelAStarOptions());

private:
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

    static std::vector<Vec> ConvertPathToPoints(
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
        const Vec& seedPoint,
        const Vec& preferredDirection,
        VoxelAStarSearchMode mode,
        int maxRadius,
        VoxelIndex& outIndex);
};