#include "VoxelChunkCache.h"

#include <algorithm>

bool VoxelChunkCache::Configure(
    const std::vector<MeshTriangle>* triangles,
    const TriangleSpatialHash* spatialHash,
    const VoxelMeshBuildOptions& buildOptions,
    const VoxelChunkCacheOptions& cacheOptions)
{
    Clear();

    if (triangles == nullptr || triangles->empty())
    {
        return false;
    }

    if (buildOptions.voxelSize <= 0.0 ||
        cacheOptions.chunkVoxelSize <= 0)
    {
        return false;
    }

    m_triangles = triangles;
    m_spatialHash = spatialHash;
    m_buildOptions = buildOptions;
    m_cacheOptions = cacheOptions;

    return true;
}

bool VoxelChunkCache::IsConfigured() const
{
    return m_triangles != nullptr &&
        !m_triangles->empty() &&
        m_buildOptions.voxelSize > 0.0 &&
        m_cacheOptions.chunkVoxelSize > 0;
}

bool VoxelChunkCache::EnsureChunkForIndex(
    VoxelSpace& space,
    const VoxelIndex& index)
{
    if (!IsConfigured() || !space.IsValid())
    {
        ++m_stats.failedBuildCount;
        return false;
    }

    const VoxelChunkIndex chunk = ToChunkIndex(index);

    if (m_builtChunks.find(chunk) != m_builtChunks.end())
    {
        ++m_stats.cacheHitCount;
        return true;
    }

    const MeshAABB buildBox = MakeChunkBuildBox(space, chunk);

    const VoxelMeshBuildResult result =
        VoxelMeshBuilder::AppendVoxelSpaceFromTrianglesInBox(
            *m_triangles,
            m_spatialHash,
            buildBox,
            m_buildOptions,
            space
        );

    if (!result.success)
    {
        ++m_stats.failedBuildCount;
        return false;
    }

    m_builtChunks.insert(chunk);
    ++m_stats.chunkBuildCount;
    m_stats.totalCandidateTriangleCount += result.candidateTriangleCount;
    m_stats.totalRawCandidateTriangleCount +=
        result.rawCandidateTriangleCount;

    return true;
}

bool VoxelChunkCache::IsChunkBuiltForIndex(
    const VoxelIndex& index) const
{
    if (!IsConfigured())
    {
        return false;
    }

    const VoxelChunkIndex chunk = ToChunkIndex(index);
    return m_builtChunks.find(chunk) != m_builtChunks.end();
}

const VoxelChunkCacheStats& VoxelChunkCache::GetStats() const
{
    return m_stats;
}

void VoxelChunkCache::Clear()
{
    m_triangles = nullptr;
    m_spatialHash = nullptr;
    m_buildOptions = VoxelMeshBuildOptions();
    m_cacheOptions = VoxelChunkCacheOptions();
    m_stats = VoxelChunkCacheStats();
    m_builtChunks.clear();
}

VoxelChunkIndex VoxelChunkCache::ToChunkIndex(
    const VoxelIndex& index) const
{
    VoxelChunkIndex chunk;
    chunk.x = FloorDiv(index.x, m_cacheOptions.chunkVoxelSize);
    chunk.y = FloorDiv(index.y, m_cacheOptions.chunkVoxelSize);
    chunk.z = FloorDiv(index.z, m_cacheOptions.chunkVoxelSize);
    return chunk;
}

MeshAABB VoxelChunkCache::MakeChunkBuildBox(
    const VoxelSpace& space,
    const VoxelChunkIndex& chunk) const
{
    const int chunkSize = m_cacheOptions.chunkVoxelSize;

    const VoxelIndex minIndex(
        chunk.x * chunkSize,
        chunk.y * chunkSize,
        chunk.z * chunkSize
    );
    const VoxelIndex maxIndex(
        minIndex.x + chunkSize - 1,
        minIndex.y + chunkSize - 1,
        minIndex.z + chunkSize - 1
    );

    MeshAABB box;
    box.minP = space.IndexToMinCorner(minIndex);
    box.maxP = space.IndexToMaxCorner(maxIndex);

    const double padding = std::max(0.0, m_cacheOptions.buildPadding);
    box.minP.x -= padding;
    box.minP.y -= padding;
    box.minP.z -= padding;
    box.maxP.x += padding;
    box.maxP.y += padding;
    box.maxP.z += padding;

    return box;
}

int VoxelChunkCache::FloorDiv(
    int value,
    int divisor)
{
    int quotient = value / divisor;
    const int remainder = value % divisor;

    if (remainder != 0 &&
        ((remainder < 0) != (divisor < 0)))
    {
        --quotient;
    }

    return quotient;
}
