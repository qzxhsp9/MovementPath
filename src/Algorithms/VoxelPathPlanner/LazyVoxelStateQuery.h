#pragma once

#include "VoxelMeshBuilder.h"

#include <chrono>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct LazyVoxelStateQueryStats
{
    std::size_t ensureCallCount = 0;
    std::size_t voxelQueryCount = 0;
    std::size_t voxelCacheHitCount = 0;
    std::size_t candidateTriangleCount = 0;
    std::size_t rawCandidateTriangleCount = 0;
    std::size_t triangleVoxelIntersectTestCount = 0;
    std::size_t triangleVoxelNoIntersectCacheHitCount = 0;
    std::size_t distanceCalculationCount = 0;
    std::size_t occupiedWriteCount = 0;
    std::size_t clearanceWriteCount = 0;
    std::size_t freeWriteCount = 0;
    double candidateQueryMs = 0.0;
    double voxelEvaluateMs = 0.0;
};

class LazyVoxelStateQuery
{
public:
    bool Configure(
        const std::vector<MeshTriangle>* triangles,
        const TriangleSpatialHash* spatialHash,
        const VoxelMeshBuildOptions& buildOptions,
        double timeBudgetMs);

    void StartBudget();

    void EnsureVoxel(
        VoxelSpace& space,
        const VoxelIndex& index);

    bool HasTimedOut() const;

    const LazyVoxelStateQueryStats& GetStats() const;

private:
    struct VoxelRecord
    {
        VoxelState state = VoxelState::Free;
        double distanceToSurface = std::numeric_limits<double>::max();
    };

    struct TriangleVoxelKey
    {
        int triangleId = 0;
        VoxelIndex voxelIndex;

        bool operator==(const TriangleVoxelKey& other) const
        {
            return triangleId == other.triangleId &&
                voxelIndex == other.voxelIndex;
        }
    };

    struct TriangleVoxelKeyHash
    {
        std::size_t operator()(const TriangleVoxelKey& key) const
        {
            const std::size_t h0 =
                static_cast<std::size_t>(key.triangleId) * 2654435761u;
            return h0 ^ VoxelIndexHash()(key.voxelIndex);
        }
    };

    struct TriangleVoxelRecord
    {
        bool intersects = false;
        double distanceToSurface = std::numeric_limits<double>::max();
    };

    bool IsConfigured() const;
    bool IsBudgetExpired() const;
    void ApplyRecordToSpace(
        VoxelSpace& space,
        const VoxelIndex& index,
        const VoxelRecord& record) const;
    void EnsureTriangleIntersections(
        VoxelSpace& space,
        int triangleId);
    static MeshAABB MakeVoxelBox(
        const VoxelSpace& space,
        const VoxelIndex& index);
    static double DistancePointToTriangle(
        const Vec& point,
        const MeshTriangle& triangle);
    static bool TriangleIntersectsAabb(
        const MeshTriangle& triangle,
        const MeshAABB& box);

private:
    const std::vector<MeshTriangle>* m_triangles = nullptr;
    const TriangleSpatialHash* m_spatialHash = nullptr;
    VoxelMeshBuildOptions m_buildOptions;
    double m_timeBudgetMs = 0.0;
    bool m_timedOut = false;
    std::chrono::steady_clock::time_point m_startTime;
    LazyVoxelStateQueryStats m_stats;

    std::unordered_map<VoxelIndex, VoxelRecord, VoxelIndexHash> m_voxelCache;
    std::unordered_map<
        TriangleVoxelKey,
        TriangleVoxelRecord,
        TriangleVoxelKeyHash> m_triangleVoxelCache;
    std::unordered_set<int> m_intersectionProcessedTriangles;
};
