#pragma once

#include "VoxelMeshBuilder.h"

#include <cstddef>
#include <unordered_set>
#include <vector>

struct VoxelChunkCacheOptions
{
    // Edge length of one lazy chunk in voxel-index units.
    int chunkVoxelSize = 16;

    // Extra world-space padding around a chunk build box. This must cover
    // clearance influence near chunk boundaries.
    double buildPadding = 0.0;
};

struct VoxelChunkCacheStats
{
    // ensureCallCount includes both chunk builds and cache hits. It helps
    // quantify lazy hook pressure from A* neighbor expansion.
    std::size_t ensureCallCount = 0;
    std::size_t chunkBuildCount = 0;
    std::size_t cacheHitCount = 0;
    std::size_t failedBuildCount = 0;
    std::size_t totalCandidateTriangleCount = 0;
    std::size_t totalRawCandidateTriangleCount = 0;

    double totalCandidateQueryMs = 0.0;
    double totalCandidateFilterMs = 0.0;
    double totalVoxelMarkMs = 0.0;
    double totalStateCountMs = 0.0;
    double totalBuildMs = 0.0;
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
    // Keeps references to shared mesh/index data; callers must keep those
    // objects alive while the cache is used.
    bool Configure(
        const std::vector<MeshTriangle>* triangles,
        const TriangleSpatialHash* spatialHash,
        const VoxelMeshBuildOptions& buildOptions,
        const VoxelChunkCacheOptions& cacheOptions);

    bool IsConfigured() const;

    // Builds the containing chunk on first access, then returns cache hits for
    // later indices in already-built chunks.
    bool EnsureChunkForIndex(
        VoxelSpace& space,
        const VoxelIndex& index);

    bool IsChunkBuiltForIndex(
        const VoxelIndex& index) const;

    const VoxelChunkCacheStats& GetStats() const;

    std::vector<VoxelChunkIndex> GetBuiltChunks() const;

    void Clear();

private:
    // Convert from global voxel coordinates to signed chunk coordinates.
    // Uses floor division so negative voxel indices map to intuitive chunks.
    VoxelChunkIndex ToChunkIndex(
        const VoxelIndex& index) const;

    // Returns the world-space chunk box with buildPadding already applied, so
    // boundary-adjacent triangle influence is included during append builds.
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
    bool m_hasLastChunk = false;
    VoxelChunkIndex m_lastChunk;

    std::unordered_set<
        VoxelChunkIndex,
        VoxelChunkIndexHash> m_builtChunks;
};
