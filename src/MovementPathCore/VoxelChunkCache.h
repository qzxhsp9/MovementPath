#pragma once

#include "VoxelMeshBuilder.h"

#include <cstddef>
#include <unordered_set>
#include <vector>

struct VoxelChunkCacheOptions
{
    int chunkVoxelSize = 16;
    double buildPadding = 0.0;
};

struct VoxelChunkCacheStats
{
    std::size_t chunkBuildCount = 0;
    std::size_t cacheHitCount = 0;
    std::size_t failedBuildCount = 0;
    std::size_t totalCandidateTriangleCount = 0;
    std::size_t totalRawCandidateTriangleCount = 0;
};

struct VoxelChunkIndex
{
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelChunkIndex& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelChunkIndexHash
{
    std::size_t operator()(const VoxelChunkIndex& index) const
    {
        const std::size_t hx =
            static_cast<std::size_t>(index.x) * 73856093u;
        const std::size_t hy =
            static_cast<std::size_t>(index.y) * 19349663u;
        const std::size_t hz =
            static_cast<std::size_t>(index.z) * 83492791u;

        return hx ^ hy ^ hz;
    }
};

class VoxelChunkCache
{
public:
    bool Configure(
        const std::vector<MeshTriangle>* triangles,
        const TriangleSpatialHash* spatialHash,
        const VoxelMeshBuildOptions& buildOptions,
        const VoxelChunkCacheOptions& cacheOptions);

    bool IsConfigured() const;

    bool EnsureChunkForIndex(
        VoxelSpace& space,
        const VoxelIndex& index);

    bool IsChunkBuiltForIndex(
        const VoxelIndex& index) const;

    const VoxelChunkCacheStats& GetStats() const;

    std::vector<VoxelChunkIndex> GetBuiltChunks() const;

    void Clear();

private:
    VoxelChunkIndex ToChunkIndex(
        const VoxelIndex& index) const;

    MeshAABB MakeChunkBuildBox(
        const VoxelSpace& space,
        const VoxelChunkIndex& chunk) const;

    static int FloorDiv(
        int value,
        int divisor);

private:
    const std::vector<MeshTriangle>* m_triangles = nullptr;
    const TriangleSpatialHash* m_spatialHash = nullptr;
    VoxelMeshBuildOptions m_buildOptions;
    VoxelChunkCacheOptions m_cacheOptions;
    VoxelChunkCacheStats m_stats;

    std::unordered_set<
        VoxelChunkIndex,
        VoxelChunkIndexHash> m_builtChunks;
};
