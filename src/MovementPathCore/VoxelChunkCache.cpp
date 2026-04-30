#include "VoxelChunkCache.h"

#include <algorithm>
#include <cmath>
#include <limits>

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
    ++m_stats.ensureCallCount;

    if (!IsConfigured() || !space.IsValid())
    {
        ++m_stats.failedBuildCount;
        return false;
    }

    const VoxelChunkIndex chunk = ToChunkIndex(index);

    if (m_hasLastChunk && chunk == m_lastChunk)
    {
        ++m_stats.cacheHitCount;
        return true;
    }

    if (m_builtChunks.find(chunk) != m_builtChunks.end())
    {
        m_hasLastChunk = true;
        m_lastChunk = chunk;
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
            space,
            m_cacheOptions.buildPadding,
            &m_buildCache
        );

    if (!result.success)
    {
        ++m_stats.failedBuildCount;
        return false;
    }

    m_builtChunks.insert(chunk);
    m_hasLastChunk = true;
    m_lastChunk = chunk;
    ++m_stats.chunkBuildCount;
    m_stats.totalCandidateTriangleCount += result.candidateTriangleCount;
    m_stats.totalRawCandidateTriangleCount +=
        result.rawCandidateTriangleCount;
    if (m_stats.chunkBuildCount == 1)
    {
        m_stats.minCandidateTriangleCount =
            result.candidateTriangleCount;
        m_stats.maxCandidateTriangleCount =
            result.candidateTriangleCount;
        m_stats.minRawCandidateTriangleCount =
            result.rawCandidateTriangleCount;
        m_stats.maxRawCandidateTriangleCount =
            result.rawCandidateTriangleCount;
    }
    else
    {
        m_stats.minCandidateTriangleCount = std::min(
            m_stats.minCandidateTriangleCount,
            result.candidateTriangleCount);
        m_stats.maxCandidateTriangleCount = std::max(
            m_stats.maxCandidateTriangleCount,
            result.candidateTriangleCount);
        m_stats.minRawCandidateTriangleCount = std::min(
            m_stats.minRawCandidateTriangleCount,
            result.rawCandidateTriangleCount);
        m_stats.maxRawCandidateTriangleCount = std::max(
            m_stats.maxRawCandidateTriangleCount,
            result.rawCandidateTriangleCount);
    }
    m_stats.totalVoxelVisitCount += result.voxelVisitCount;
    m_stats.totalOutOfBoundsVoxelCount += result.outOfBoundsVoxelCount;
    m_stats.totalDistanceCalculationCount +=
        result.distanceCalculationCount;
    m_stats.totalDistanceImprovedCount += result.distanceImprovedCount;
    m_stats.totalDistanceNotImprovedCount +=
        result.distanceNotImprovedCount;
    m_stats.totalStateWriteCount += result.stateWriteCount;
    m_stats.totalStateUnchangedWriteCount +=
        result.stateUnchangedWriteCount;
    m_stats.totalOccupiedUnchangedWriteCount +=
        result.occupiedUnchangedWriteCount;
    m_stats.totalClearanceUnchangedWriteCount +=
        result.clearanceUnchangedWriteCount;
    m_stats.totalOccupiedWriteCount += result.occupiedWriteCount;
    m_stats.totalClearanceWriteCount += result.clearanceWriteCount;
    m_stats.totalInfluenceCacheHitCount += result.influenceCacheHitCount;
    m_stats.totalInfluenceCacheMissCount += result.influenceCacheMissCount;
    m_stats.totalCandidateQueryMs += result.candidateQueryMs;
    m_stats.totalCandidateFilterMs += result.candidateFilterMs;
    m_stats.totalVoxelMarkMs += result.voxelMarkMs;
    m_stats.totalStateCountMs += result.stateCountMs;
    m_stats.totalBuildMs += result.totalBuildMs;

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

std::vector<VoxelChunkIndex> VoxelChunkCache::GetBuiltChunks() const
{
    return std::vector<VoxelChunkIndex>(
        m_builtChunks.begin(),
        m_builtChunks.end());
}

void VoxelChunkCache::Clear()
{
    m_triangles = nullptr;
    m_spatialHash = nullptr;
    m_buildOptions = VoxelMeshBuildOptions();
    m_cacheOptions = VoxelChunkCacheOptions();
    m_stats = VoxelChunkCacheStats();
    m_buildCache = VoxelMeshBuildCache();
    m_hasLastChunk = false;
    m_lastChunk = VoxelChunkIndex();
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

    box.maxP.x = std::nextafter(
        box.maxP.x,
        -std::numeric_limits<double>::infinity());
    box.maxP.y = std::nextafter(
        box.maxP.y,
        -std::numeric_limits<double>::infinity());
    box.maxP.z = std::nextafter(
        box.maxP.z,
        -std::numeric_limits<double>::infinity());

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
