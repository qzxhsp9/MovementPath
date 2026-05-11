#include "LazyVoxelStateQuery.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
std::chrono::steady_clock::time_point Now()
{
    return std::chrono::steady_clock::now();
}

double ElapsedMs(
    const std::chrono::steady_clock::time_point& start,
    const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void ExpandAABB(
    MeshAABB& box,
    double offset)
{
    if (offset <= 0.0)
    {
        return;
    }

    box.minP.x -= offset;
    box.minP.y -= offset;
    box.minP.z -= offset;
    box.maxP.x += offset;
    box.maxP.y += offset;
    box.maxP.z += offset;
}

double Dot(
    const Vec& a,
    const Vec& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec Cross(
    const Vec& a,
    const Vec& b)
{
    return Vec(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x);
}

bool SeparatesOnAxis(
    const Vec& axis,
    const Vec& v0,
    const Vec& v1,
    const Vec& v2,
    const Vec& halfSize)
{
    if (axis.SquareMagnitude() <= 1.0e-20)
    {
        return false;
    }

    const double p0 = Dot(v0, axis);
    const double p1 = Dot(v1, axis);
    const double p2 = Dot(v2, axis);
    const double triMin = std::min(p0, std::min(p1, p2));
    const double triMax = std::max(p0, std::max(p1, p2));
    const double boxRadius =
        halfSize.x * std::abs(axis.x) +
        halfSize.y * std::abs(axis.y) +
        halfSize.z * std::abs(axis.z);

    return triMin > boxRadius || triMax < -boxRadius;
}
}

bool LazyVoxelStateQuery::Configure(
    const std::vector<MeshTriangle>* triangles,
    const TriangleSpatialHash* spatialHash,
    const VoxelMeshBuildOptions& buildOptions,
    double timeBudgetMs,
    const std::function<bool()>& shouldCancel)
{
    m_triangles = triangles;
    m_spatialHash = spatialHash;
    m_buildOptions = buildOptions;
    m_timeBudgetMs = timeBudgetMs;
    m_shouldCancel = shouldCancel;
    m_timedOut = false;
    m_stats = LazyVoxelStateQueryStats();
    m_voxelCache.clear();
    m_triangleVoxelCache.clear();
    m_intersectionProcessedTriangles.clear();
    return IsConfigured();
}

void LazyVoxelStateQuery::StartBudget()
{
    m_startTime = Now();
    m_timedOut = false;
}

bool LazyVoxelStateQuery::IsConfigured() const
{
    return m_triangles != nullptr &&
        !m_triangles->empty() &&
        m_spatialHash != nullptr &&
        m_spatialHash->IsValid() &&
        m_buildOptions.voxelSize > 0.0;
}

bool LazyVoxelStateQuery::IsCancelled() const
{
    return m_shouldCancel && m_shouldCancel();
}

bool LazyVoxelStateQuery::IsBudgetExpired() const
{
    if (m_timeBudgetMs <= 0.0)
    {
        return false;
    }

    return ElapsedMs(m_startTime, Now()) >= m_timeBudgetMs;
}

bool LazyVoxelStateQuery::HasTimedOut() const
{
    return m_timedOut;
}

const LazyVoxelStateQueryStats& LazyVoxelStateQuery::GetStats() const
{
    return m_stats;
}

void LazyVoxelStateQuery::ApplyRecordToSpace(
    VoxelSpace& space,
    const VoxelIndex& index,
    const VoxelRecord& record) const
{
    space.SetCellDistanceIfSmaller(index, record.distanceToSurface);

    const VoxelState existing = space.GetCellState(index);
    if (existing != VoxelState::Start &&
        existing != VoxelState::Goal &&
        existing != VoxelState::Path)
    {
        space.SetCellState(index, record.state);
    }
}

MeshAABB LazyVoxelStateQuery::MakeVoxelBox(
    const VoxelSpace& space,
    const VoxelIndex& index)
{
    MeshAABB box;
    box.minP = space.IndexToMinCorner(index);
    box.maxP = space.IndexToMaxCorner(index);
    return box;
}

void LazyVoxelStateQuery::EnsureVoxel(
    VoxelSpace& space,
    const VoxelIndex& index)
{
    ++m_stats.ensureCallCount;

    if (!IsConfigured() ||
        m_timedOut ||
        IsCancelled() ||
        !space.IsInsideSearchBounds(index))
    {
        return;
    }

    if (IsBudgetExpired())
    {
        m_timedOut = true;
        return;
    }

    const auto voxelIt = m_voxelCache.find(index);
    if (voxelIt != m_voxelCache.end())
    {
        ++m_stats.voxelCacheHitCount;
        ApplyRecordToSpace(space, index, voxelIt->second);
        return;
    }

    const auto evaluateStart = Now();
    ++m_stats.voxelQueryCount;

    const MeshAABB voxelBox = MakeVoxelBox(space, index);
    MeshAABB queryBox = voxelBox;
    double queryExpand = m_buildOptions.clearance;
    if (m_buildOptions.conservativeClearance)
    {
        queryExpand += space.GetHalfDiagonal();
    }
    ExpandAABB(queryBox, queryExpand);

    std::vector<int> candidateTriangleIds;
    TriangleSpatialHashStats queryStats;
    const auto queryStart = Now();
    m_spatialHash->Query(queryBox, candidateTriangleIds, queryStats);
    m_stats.candidateQueryMs += ElapsedMs(queryStart, Now());
    m_stats.rawCandidateTriangleCount += queryStats.queryRawTriangleCount;
    m_stats.candidateTriangleCount += candidateTriangleIds.size();

    VoxelRecord voxelRecord;
    const Vec center = space.IndexToCenter(index);
    const double halfDiagonal = space.GetHalfDiagonal();
    const double effectiveClearance =
        m_buildOptions.clearance +
        (m_buildOptions.conservativeClearance ? halfDiagonal : 0.0);

    for (int triangleId : candidateTriangleIds)
    {
        if (IsCancelled())
        {
            return;
        }

        if (IsBudgetExpired())
        {
            m_timedOut = true;
            return;
        }

        if (triangleId < 0 ||
            static_cast<std::size_t>(triangleId) >= m_triangles->size())
        {
            continue;
        }

        EnsureTriangleIntersections(space, triangleId);
        if (m_timedOut)
        {
            return;
        }

        const auto occupiedVoxelIt = m_voxelCache.find(index);
        if (occupiedVoxelIt != m_voxelCache.end() &&
            occupiedVoxelIt->second.state == VoxelState::Occupied)
        {
            ApplyRecordToSpace(space, index, occupiedVoxelIt->second);
            m_stats.voxelEvaluateMs += ElapsedMs(evaluateStart, Now());
            return;
        }

        const TriangleVoxelKey key{ triangleId, index };
        auto pairIt = m_triangleVoxelCache.find(key);
        TriangleVoxelRecord pairRecord;
        if (pairIt != m_triangleVoxelCache.end())
        {
            pairRecord = pairIt->second;
            if (!pairRecord.intersects)
            {
                ++m_stats.triangleVoxelNoIntersectCacheHitCount;
            }
        }
        else
        {
            const MeshTriangle& triangle =
                (*m_triangles)[static_cast<std::size_t>(triangleId)];

            // Direct intersections for this triangle were already enumerated
            // by EnsureTriangleIntersections(). A missing pair record here is
            // therefore known to be non-intersecting and only needs distance.
            pairRecord.intersects = false;
            pairRecord.distanceToSurface =
                DistancePointToTriangle(center, triangle);
            ++m_stats.distanceCalculationCount;

            m_triangleVoxelCache.emplace(key, pairRecord);
        }

        if (pairRecord.intersects)
        {
            voxelRecord.state = VoxelState::Occupied;
            voxelRecord.distanceToSurface = 0.0;
            ++m_stats.occupiedWriteCount;
            m_voxelCache.emplace(index, voxelRecord);
            ApplyRecordToSpace(space, index, voxelRecord);
            m_stats.voxelEvaluateMs += ElapsedMs(evaluateStart, Now());
            return;
        }

        voxelRecord.distanceToSurface =
            std::min(voxelRecord.distanceToSurface,
                pairRecord.distanceToSurface);
    }

    if (m_buildOptions.clearance > 0.0 &&
        voxelRecord.distanceToSurface <= effectiveClearance)
    {
        voxelRecord.state = VoxelState::ClearanceBand;
        ++m_stats.clearanceWriteCount;
    }
    else
    {
        voxelRecord.state = VoxelState::Free;
        ++m_stats.freeWriteCount;
    }

    m_voxelCache.emplace(index, voxelRecord);
    ApplyRecordToSpace(space, index, voxelRecord);
    m_stats.voxelEvaluateMs += ElapsedMs(evaluateStart, Now());
}

void LazyVoxelStateQuery::EnsureTriangleIntersections(
    VoxelSpace& space,
    int triangleId)
{
    if (m_intersectionProcessedTriangles.find(triangleId) !=
        m_intersectionProcessedTriangles.end())
    {
        return;
    }

    if (triangleId < 0 ||
        static_cast<std::size_t>(triangleId) >= m_triangles->size())
    {
        return;
    }

    const MeshTriangle& triangle =
        (*m_triangles)[static_cast<std::size_t>(triangleId)];

    MeshAABB triangleBox;
    triangleBox.minP.x =
        std::min(triangle.p0.x, std::min(triangle.p1.x, triangle.p2.x));
    triangleBox.minP.y =
        std::min(triangle.p0.y, std::min(triangle.p1.y, triangle.p2.y));
    triangleBox.minP.z =
        std::min(triangle.p0.z, std::min(triangle.p1.z, triangle.p2.z));
    triangleBox.maxP.x =
        std::max(triangle.p0.x, std::max(triangle.p1.x, triangle.p2.x));
    triangleBox.maxP.y =
        std::max(triangle.p0.y, std::max(triangle.p1.y, triangle.p2.y));
    triangleBox.maxP.z =
        std::max(triangle.p0.z, std::max(triangle.p1.z, triangle.p2.z));

    VoxelIndex minIndex = space.WorldToIndex(triangleBox.minP);
    VoxelIndex maxIndex = space.WorldToIndex(triangleBox.maxP);
    if (minIndex.x > maxIndex.x) { std::swap(minIndex.x, maxIndex.x); }
    if (minIndex.y > maxIndex.y) { std::swap(minIndex.y, maxIndex.y); }
    if (minIndex.z > maxIndex.z) { std::swap(minIndex.z, maxIndex.z); }

    if (space.HasSearchBounds())
    {
        const VoxelBounds& bounds = space.GetSearchBounds();
        minIndex.x = std::max(minIndex.x, bounds.minIndex.x);
        minIndex.y = std::max(minIndex.y, bounds.minIndex.y);
        minIndex.z = std::max(minIndex.z, bounds.minIndex.z);
        maxIndex.x = std::min(maxIndex.x, bounds.maxIndex.x);
        maxIndex.y = std::min(maxIndex.y, bounds.maxIndex.y);
        maxIndex.z = std::min(maxIndex.z, bounds.maxIndex.z);
    }

    for (int x = minIndex.x; x <= maxIndex.x; ++x)
    {
        for (int y = minIndex.y; y <= maxIndex.y; ++y)
        {
            for (int z = minIndex.z; z <= maxIndex.z; ++z)
            {
                if (IsCancelled())
                {
                    return;
                }

                if (IsBudgetExpired())
                {
                    m_timedOut = true;
                    return;
                }

                const VoxelIndex index(x, y, z);
                if (!space.IsInsideSearchBounds(index))
                {
                    continue;
                }

                const auto voxelIt = m_voxelCache.find(index);
                if (voxelIt != m_voxelCache.end() &&
                    voxelIt->second.state == VoxelState::Occupied)
                {
                    continue;
                }

                const TriangleVoxelKey key{ triangleId, index };
                if (m_triangleVoxelCache.find(key) !=
                    m_triangleVoxelCache.end())
                {
                    continue;
                }

                TriangleVoxelRecord pairRecord;
                ++m_stats.triangleVoxelIntersectTestCount;
                pairRecord.intersects =
                    TriangleIntersectsAabb(triangle, MakeVoxelBox(space, index));

                if (pairRecord.intersects)
                {
                    VoxelRecord voxelRecord;
                    voxelRecord.state = VoxelState::Occupied;
                    voxelRecord.distanceToSurface = 0.0;
                    m_voxelCache[index] = voxelRecord;
                }

                m_triangleVoxelCache.emplace(key, pairRecord);
            }
        }
    }

    m_intersectionProcessedTriangles.insert(triangleId);
}

double LazyVoxelStateQuery::DistancePointToTriangle(
    const Vec& p,
    const MeshTriangle& tri)
{
    const Vec ab(tri.p0, tri.p1);
    const Vec ac(tri.p0, tri.p2);
    const Vec ap(tri.p0, p);

    const double d1 = ab.Dot(ap);
    const double d2 = ac.Dot(ap);

    if (d1 <= 0.0 && d2 <= 0.0)
    {
        return p.Distance(tri.p0);
    }

    const Vec bp(tri.p1, p);
    const double d3 = ab.Dot(bp);
    const double d4 = ac.Dot(bp);

    if (d3 >= 0.0 && d4 <= d3)
    {
        return p.Distance(tri.p1);
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        const double v = d1 / (d1 - d3);
        return p.Distance(tri.p0 + ab * v);
    }

    const Vec cp(tri.p2, p);
    const double d5 = ab.Dot(cp);
    const double d6 = ac.Dot(cp);

    if (d6 >= 0.0 && d5 <= d6)
    {
        return p.Distance(tri.p2);
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        const double w = d2 / (d2 - d6);
        return p.Distance(tri.p0 + ac * w);
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 &&
        (d4 - d3) >= 0.0 &&
        (d5 - d6) >= 0.0)
    {
        const Vec bc(tri.p1, tri.p2);
        const double w =
            (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return p.Distance(tri.p1 + bc * w);
    }

    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;
    return p.Distance(tri.p0 + ab * v + ac * w);
}

bool LazyVoxelStateQuery::TriangleIntersectsAabb(
    const MeshTriangle& triangle,
    const MeshAABB& box)
{
    const Vec center = (box.minP + box.maxP) * 0.5;
    const Vec halfSize = (box.maxP - box.minP) * 0.5;

    const Vec v0 = triangle.p0 - center;
    const Vec v1 = triangle.p1 - center;
    const Vec v2 = triangle.p2 - center;

    const Vec edges[] = {
        v1 - v0,
        v2 - v1,
        v0 - v2
    };
    const Vec axes[] = {
        Vec(1.0, 0.0, 0.0),
        Vec(0.0, 1.0, 0.0),
        Vec(0.0, 0.0, 1.0)
    };

    for (const Vec& axis : axes)
    {
        if (SeparatesOnAxis(axis, v0, v1, v2, halfSize))
        {
            return false;
        }
    }

    const Vec normal = Cross(edges[0], edges[1]);
    if (SeparatesOnAxis(normal, v0, v1, v2, halfSize))
    {
        return false;
    }

    for (const Vec& edge : edges)
    {
        for (const Vec& axis : axes)
        {
            if (SeparatesOnAxis(Cross(edge, axis), v0, v1, v2, halfSize))
            {
                return false;
            }
        }
    }

    return true;
}
