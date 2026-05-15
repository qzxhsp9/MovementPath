#pragma once

#include "VoxelSpace.h"
#include "VoxelWalkability.h"

#include <vector>
#include <functional>
#include <string>

// ============================================================
// 路径优化配置
// ============================================================

struct VoxelPathOptimizeOptions
{
    VoxelAStarSearchMode searchMode = VoxelAStarSearchMode::ClearanceBand;
    double minTravelDistanceToSurface = 0.0;
    std::vector<VoxelRestrictedHalfSpace> restrictedHalfSpaces;

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
    // Same unit as A* turnPenalty. When positive, line-of-sight compression
    // considers the entry/exit bend severity instead of always taking the
    // farthest visible point.
    double shortcutTurnPenalty = 0.0;

    bool enableCurveSmoothing = false;
    // Minimum number of samples generated per control segment.
    int curveSamplesPerSegment = 8;
    // Minimum number of samples generated per control segment for UI display.
    // 0 uses curveSamplesPerSegment.
    int displayCurveSamplesPerSegment = 0;
    // Target spacing for generated/densified curve samples. 0 uses a
    // voxel-size-derived default.
    double curveSampleSpacing = 0.0;
    // Deprecated: smoothing no longer rejects candidates by deviation from
    // the control polyline. Collision/restricted-voxel checks are the safety
    // constraints.
    double maxCurveDeviation = 0.0;
    // Valid smoothing candidates are ranked by length plus these weighted
    // shape terms. Higher turn weights prefer straighter, smoother curves;
    // higher detour weight prefers shorter curves.
    double smoothingSignificantTurnWeight = 3.0;
    double smoothingTotalTurnWeight = 2.0;
    double smoothingMaxTurnWeight = 6.0;
    double smoothingDetourWeight = 1.5;
    bool useEndpointDirections = false;
    Vec startDirection;
    Vec goalDirection;
    double minEndpointDirectionAlignment = 0.95;
    bool useRealEndpointsForSmoothing = false;
    Vec realStartPoint;
    Vec realGoalPoint;
    double endpointCollisionExemptRadius = 0.0;

    // Lazy voxel state source. Full-bounds callers leave this empty because
    // every relevant voxel has already been classified.
    std::function<void(VoxelSpace&, const VoxelIndex&)> ensureCellBuilt;
    std::function<bool()> shouldCancel;
};

// ============================================================
// 路径优化结果
// ============================================================

struct VoxelPathOptimizeResult
{
    std::vector<VoxelIndex> voxelPath;
    std::vector<Vec> pointPath;
    std::vector<Vec> displayPointPath;
    std::vector<Vec> controlPointPath;

    std::size_t inputCount = 0;
    std::size_t afterCollinearCount = 0;
    std::size_t outputCount = 0;
    std::size_t smoothedPointCount = 0;

    int lineCheckCount = 0;
    int smoothingLineCheckCount = 0;
    bool smoothingSucceeded = false;
    std::string pathOutputStage = "None";
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
    std::string pathDataText;
};

// ============================================================
// 路径优化器
// ============================================================

class VoxelPathOptimizer
{
public:
    static VoxelPathOptimizeResult Optimize(
        VoxelSpace& space,
        const std::vector<VoxelIndex>& inputPath,
        const VoxelPathOptimizeOptions& options);

    static std::vector<VoxelIndex> RemoveCollinearVoxels(
        const std::vector<VoxelIndex>& path);

    static bool IsLineWalkable(
        const VoxelSpace& space,
        const VoxelIndex& from,
        const VoxelIndex& to,
        VoxelAStarSearchMode mode);

    static bool IsLineWalkable(
        VoxelSpace& space,
        const VoxelIndex& from,
        const VoxelIndex& to,
        const VoxelPathOptimizeOptions& options);

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
