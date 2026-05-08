#pragma once

#include "VoxelSpace.h"
#include "VoxelWalkability.h"

#include <vector>

// ============================================================
// 路径优化配置
// ============================================================

struct VoxelPathOptimizeOptions
{
    VoxelAStarSearchMode searchMode = VoxelAStarSearchMode::ClearanceBand;

    // 直线检测时是否允许经过 Start / Goal / Path
    // 通常应为 true。
    bool allowSpecialStates = true;

    // 最大跳点窗口。
    // <= 0 表示不限制，会尝试尽可能远的跳点，但最坏 O(n^2)。
    // 工程上建议 100 ~ 300。
    int maxShortcutLookAhead = 200;

    // 是否执行共线点删除
    bool removeCollinear = true;

    // 是否执行直线可通行压缩
    bool enableLineOfSightShortcut = true;

    bool enableCurveSmoothing = false;
    int curveSamplesPerSegment = 8;
    double curveSampleSpacing = 0.0;
    double maxCurveDeviation = 0.0;
};

// ============================================================
// 路径优化结果
// ============================================================

struct VoxelPathOptimizeResult
{
    std::vector<VoxelIndex> voxelPath;
    std::vector<Vec> pointPath;

    std::size_t inputCount = 0;
    std::size_t afterCollinearCount = 0;
    std::size_t outputCount = 0;
    std::size_t smoothedPointCount = 0;

    int lineCheckCount = 0;
    int smoothingLineCheckCount = 0;
    bool smoothingSucceeded = false;
};

// ============================================================
// 路径优化器
// ============================================================

class VoxelPathOptimizer
{
public:
    static VoxelPathOptimizeResult Optimize(
        const VoxelSpace& space,
        const std::vector<VoxelIndex>& inputPath,
        const VoxelPathOptimizeOptions& options);

    static std::vector<VoxelIndex> RemoveCollinearVoxels(
        const std::vector<VoxelIndex>& path);

    static bool IsLineWalkable(
        const VoxelSpace& space,
        const VoxelIndex& from,
        const VoxelIndex& to,
        VoxelAStarSearchMode mode);

    static std::vector<Vec> ConvertToPoints(
        const VoxelSpace& space,
        const std::vector<VoxelIndex>& path);

    static std::vector<Vec> SmoothPointPath(
        const VoxelSpace& space,
        const std::vector<Vec>& controlPath,
        const VoxelPathOptimizeOptions& options,
        int& lineCheckCount);

private:
    static bool SameDirection(
        const VoxelIndex& a,
        const VoxelIndex& b,
        const VoxelIndex& c);

    static int Sign(int v);

    static bool IsIndexWalkableForLine(
        const VoxelSpace& space,
        const VoxelIndex& index,
        VoxelAStarSearchMode mode);
};
