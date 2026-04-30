#include "VoxelMeshBuilder.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_set>

#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopLoc_Location.hxx>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>

#include <Poly_Triangulation.hxx>
#include <Poly_Triangle.hxx>

#include <TColgp_Array1OfPnt.hxx>
#include <Poly_Array1OfTriangle.hxx>

#include <gp_Vec.hxx>

// ============================================================
// 小工具
// ============================================================

static double Min3(double a, double b, double c)
{
    return std::min(a, std::min(b, c));
}

static double Max3(double a, double b, double c)
{
    return std::max(a, std::max(b, c));
}

static MeshAABB ComputeTriangleAABBLocal(
    const MeshTriangle& tri)
{
    MeshAABB box;

    box.minP = Vec(
        Min3(tri.p0.x, tri.p1.x, tri.p2.x),
        Min3(tri.p0.y, tri.p1.y, tri.p2.y),
        Min3(tri.p0.z, tri.p1.z, tri.p2.z)
    );

    box.maxP = Vec(
        Max3(tri.p0.x, tri.p1.x, tri.p2.x),
        Max3(tri.p0.y, tri.p1.y, tri.p2.y),
        Max3(tri.p0.z, tri.p1.z, tri.p2.z)
    );

    return box;
}

static std::chrono::steady_clock::time_point Now()
{
    return std::chrono::steady_clock::now();
}

static double ElapsedMs(
    const std::chrono::steady_clock::time_point& start,
    const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(
        end - start).count();
}

static void AddMarkStats(
    VoxelMeshBuildResult& result,
    const VoxelTriangleMarkStats& stats)
{
    result.voxelVisitCount += stats.voxelVisitCount;
    result.outOfBoundsVoxelCount += stats.outOfBoundsVoxelCount;
    result.distanceCalculationCount += stats.distanceCalculationCount;
    result.distanceImprovedCount += stats.distanceImprovedCount;
    result.distanceNotImprovedCount += stats.distanceNotImprovedCount;
    result.stateWriteCount += stats.stateWriteCount;
    result.stateUnchangedWriteCount += stats.stateUnchangedWriteCount;
    result.occupiedUnchangedWriteCount +=
        stats.occupiedUnchangedWriteCount;
    result.clearanceUnchangedWriteCount +=
        stats.clearanceUnchangedWriteCount;
    result.occupiedWriteCount += stats.occupiedWriteCount;
    result.clearanceWriteCount += stats.clearanceWriteCount;
}

static bool ComputeTrianglesAABBLocal(
    const std::vector<MeshTriangle>& triangles,
    MeshAABB& outBox)
{
    if (triangles.empty())
    {
        return false;
    }

    double xmin = std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double zmin = std::numeric_limits<double>::max();

    double xmax = -std::numeric_limits<double>::max();
    double ymax = -std::numeric_limits<double>::max();
    double zmax = -std::numeric_limits<double>::max();

    for (const MeshTriangle& tri : triangles)
    {
        MeshAABB box = ComputeTriangleAABBLocal(tri);

        xmin = std::min(xmin, box.minP.x);
        ymin = std::min(ymin, box.minP.y);
        zmin = std::min(zmin, box.minP.z);

        xmax = std::max(xmax, box.maxP.x);
        ymax = std::max(ymax, box.maxP.y);
        zmax = std::max(zmax, box.maxP.z);
    }

    outBox.minP = Vec(xmin, ymin, zmin);
    outBox.maxP = Vec(xmax, ymax, zmax);

    return true;
}

static void ExpandAABBLocal(
    MeshAABB& box,
    double offset)
{
    if (offset <= 0.0)
    {
        return;
    }

    box.minP.x = (box.minP.x - offset);
    box.minP.y = (box.minP.y - offset);
    box.minP.z = (box.minP.z - offset);

    box.maxP.x = (box.maxP.x + offset);
    box.maxP.y = (box.maxP.y + offset);
    box.maxP.z = (box.maxP.z + offset);
}

bool TriangleSpatialHash::Build(
    const std::vector<MeshTriangle>& triangles,
    const TriangleSpatialHashOptions& options)
{
    m_cells.clear();
    m_cellSize = 0.0;
    m_origin = Vec();
    m_entryCount = 0;

    if (triangles.empty() || options.cellSize <= 0.0)
    {
        return false;
    }

    MeshAABB globalBox;

    if (!ComputeTrianglesAABBLocal(triangles, globalBox))
    {
        return false;
    }

    m_origin = globalBox.minP;
    m_cellSize = options.cellSize;

    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        const MeshAABB triBox = ComputeTriangleAABBLocal(triangles[i]);

        SpatialCellIndex minCell = WorldToCell(triBox.minP);
        SpatialCellIndex maxCell = WorldToCell(triBox.maxP);

        if (minCell.x > maxCell.x) std::swap(minCell.x, maxCell.x);
        if (minCell.y > maxCell.y) std::swap(minCell.y, maxCell.y);
        if (minCell.z > maxCell.z) std::swap(minCell.z, maxCell.z);

        for (int ix = minCell.x; ix <= maxCell.x; ++ix)
        {
            for (int iy = minCell.y; iy <= maxCell.y; ++iy)
            {
                for (int iz = minCell.z; iz <= maxCell.z; ++iz)
                {
                    SpatialCellIndex cell;
                    cell.x = ix;
                    cell.y = iy;
                    cell.z = iz;

                    m_cells[cell].push_back(static_cast<int>(i));
                    ++m_entryCount;
                }
            }
        }
    }

    return true;
}

bool TriangleSpatialHash::IsValid() const
{
    return m_cellSize > 0.0;
}

void TriangleSpatialHash::Query(
    const MeshAABB& queryBox,
    std::vector<int>& outTriangleIds) const
{
    TriangleSpatialHashStats stats;
    Query(queryBox, outTriangleIds, stats);
}

void TriangleSpatialHash::Query(
    const MeshAABB& queryBox,
    std::vector<int>& outTriangleIds,
    TriangleSpatialHashStats& outStats) const
{
    outTriangleIds.clear();
    outStats = TriangleSpatialHashStats();

    if (!IsValid())
    {
        return;
    }

    SpatialCellIndex minCell = WorldToCell(queryBox.minP);
    SpatialCellIndex maxCell = WorldToCell(queryBox.maxP);

    if (minCell.x > maxCell.x) std::swap(minCell.x, maxCell.x);
    if (minCell.y > maxCell.y) std::swap(minCell.y, maxCell.y);
    if (minCell.z > maxCell.z) std::swap(minCell.z, maxCell.z);

    std::unordered_set<int> uniqueIds;

    for (int ix = minCell.x; ix <= maxCell.x; ++ix)
    {
        for (int iy = minCell.y; iy <= maxCell.y; ++iy)
        {
            for (int iz = minCell.z; iz <= maxCell.z; ++iz)
            {
                ++outStats.queryCellCount;

                SpatialCellIndex cell;
                cell.x = ix;
                cell.y = iy;
                cell.z = iz;

                const auto it = m_cells.find(cell);

                if (it == m_cells.end())
                {
                    continue;
                }

                for (int triangleId : it->second)
                {
                    ++outStats.queryRawTriangleCount;

                    if (uniqueIds.insert(triangleId).second)
                    {
                        outTriangleIds.push_back(triangleId);
                    }
                }
            }
        }
    }

    outStats.queryUniqueTriangleCount = outTriangleIds.size();
}

double TriangleSpatialHash::GetCellSize() const
{
    return m_cellSize;
}

TriangleSpatialHashStats TriangleSpatialHash::GetStats() const
{
    TriangleSpatialHashStats stats;
    stats.cellCount = m_cells.size();
    stats.entryCount = m_entryCount;
    return stats;
}

SpatialCellIndex TriangleSpatialHash::WorldToCell(const Vec& point) const
{
    SpatialCellIndex cell;

    cell.x = static_cast<int>(
        std::floor((point.x - m_origin.x) / m_cellSize)
    );
    cell.y = static_cast<int>(
        std::floor((point.y - m_origin.y) / m_cellSize)
    );
    cell.z = static_cast<int>(
        std::floor((point.z - m_origin.z) / m_cellSize)
    );

    return cell;
}

// ============================================================
// Shape 三角化
// ============================================================

bool VoxelMeshBuilder::BuildShapeTriangulation(
    const TopoDS_Shape& shape,
    double deflection,
    double angularDeflection,
    std::vector<MeshTriangle>& outTriangles)
{
    outTriangles.clear();

    if (shape.IsNull())
    {
        return false;
    }

    if (deflection <= 0.0)
    {
        deflection = 0.1;
    }

    if (angularDeflection <= 0.0)
    {
        angularDeflection = 0.5;
    }

    BRepMesh_IncrementalMesh mesher(
        shape,
        deflection,
        Standard_False,
        angularDeflection,
        Standard_True
    );

    mesher.Perform();

    if (!mesher.IsDone())
    {
        return false;
    }

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());

        TopLoc_Location loc;
        Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(face, loc);

        if (triangulation.IsNull())
        {
            continue;
        }

        const gp_Trsf trsf = loc.Transformation();

        Standard_Integer nbTriangles = triangulation->NbTriangles();
        for (int i = 1; i <= nbTriangles; ++i)
        {
            const Poly_Triangle& triangle = triangulation->Triangle(i);            
            gp_Pnt p0 = triangulation->Node(triangle.Value(1)).Transformed(trsf);
            gp_Pnt p1 = triangulation->Node(triangle.Value(2)).Transformed(trsf);
            gp_Pnt p2 = triangulation->Node(triangle.Value(3)).Transformed(trsf);
            MeshTriangle tri;
            tri.p0 = Vec(p0.X(), p0.Y(), p0.Z());
            tri.p1 = Vec(p1.X(), p1.Y(), p1.Z());
            tri.p2 = Vec(p2.X(), p2.Y(), p2.Z());
            outTriangles.push_back(tri);
        }
    }

    return !outTriangles.empty();
}

// ============================================================
// AABB
// ============================================================

MeshAABB VoxelMeshBuilder::ComputeTriangleAABB(
    const MeshTriangle& tri)
{
    return ComputeTriangleAABBLocal(tri);
}

bool VoxelMeshBuilder::ComputeTrianglesAABB(
    const std::vector<MeshTriangle>& triangles,
    MeshAABB& outBox)
{
    return ComputeTrianglesAABBLocal(triangles, outBox);
}

void VoxelMeshBuilder::ExpandAABB(
    MeshAABB& box,
    double offset)
{
    ExpandAABBLocal(box, offset);
}

bool VoxelMeshBuilder::IntersectsAABB(
    const MeshAABB& a,
    const MeshAABB& b)
{
    return a.minP.x <= b.maxP.x && a.maxP.x >= b.minP.x &&
        a.minP.y <= b.maxP.y && a.maxP.y >= b.minP.y &&
        a.minP.z <= b.maxP.z && a.maxP.z >= b.minP.z;
}

MeshAABB VoxelMeshBuilder::ComputeTriangleInfluenceAABB(
    const MeshTriangle& tri,
    const VoxelMeshBuildOptions& options,
    double halfDiag)
{
    MeshAABB influenceBox = ComputeTriangleAABB(tri);

    double effectiveClearance = options.clearance;

    if (options.conservativeClearance)
    {
        effectiveClearance += halfDiag;
    }

    ExpandAABB(influenceBox, effectiveClearance);

    return influenceBox;
}

void VoxelMeshBuilder::NormalizeIndexRange(
    VoxelIndex& minIndex,
    VoxelIndex& maxIndex)
{
    if (minIndex.x > maxIndex.x)
    {
        std::swap(minIndex.x, maxIndex.x);
    }

    if (minIndex.y > maxIndex.y)
    {
        std::swap(minIndex.y, maxIndex.y);
    }

    if (minIndex.z > maxIndex.z)
    {
        std::swap(minIndex.z, maxIndex.z);
    }
}

// ============================================================
// 点到三角形距离
// ============================================================

double VoxelMeshBuilder::DistancePointToTriangle(
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
        Vec closest = tri.p0 + ab * v;
        return p.Distance(closest);
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
        Vec closest = tri.p0 + ac * w;
        return p.Distance(closest);
    }

    const double va = d3 * d6 - d5 * d4;

    if (va <= 0.0 &&
        (d4 - d3) >= 0.0 &&
        (d5 - d6) >= 0.0)
    {
        const Vec bc(tri.p1, tri.p2);
        const double w =
            (d4 - d3) / ((d4 - d3) + (d5 - d6));

        Vec closest = tri.p1 + bc * w;
        return p.Distance(closest);
    }

    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;

    Vec offset = ab * v + ac * w;
    Vec closest = tri.p0 + offset;

    return p.Distance(closest);
}

// ============================================================
// 存储 Free 体素，可选
// ============================================================

void VoxelMeshBuilder::StoreFreeCellsInBounds(
    const VoxelBounds& bounds,
    VoxelSpace& space)
{
    if (!bounds.IsValid())
    {
        return;
    }

    for (int ix = bounds.minIndex.x; ix <= bounds.maxIndex.x; ++ix)
    {
        for (int iy = bounds.minIndex.y; iy <= bounds.maxIndex.y; ++iy)
        {
            for (int iz = bounds.minIndex.z; iz <= bounds.maxIndex.z; ++iz)
            {
                VoxelIndex index(ix, iy, iz);
                space.SetCellState(index, VoxelState::Free);
            }
        }
    }
}

// ============================================================
// 核心：单个三角形写入 VoxelSpace
// ============================================================

VoxelTriangleInfluenceRange VoxelMeshBuilder::ComputeTriangleInfluenceRange(
    const MeshTriangle& tri,
    const VoxelMeshBuildOptions& options,
    const VoxelSpace& space)
{
    const double halfDiag = space.GetHalfDiagonal();

    VoxelTriangleInfluenceRange range;
    range.influenceBox =
        ComputeTriangleInfluenceAABB(tri, options, halfDiag);
    range.minIndex = space.WorldToIndex(range.influenceBox.minP);
    range.maxIndex = space.WorldToIndex(range.influenceBox.maxP);
    NormalizeIndexRange(range.minIndex, range.maxIndex);
    return range;
}

VoxelTriangleMarkStats VoxelMeshBuilder::MarkTriangleToVoxelSpace(
    const MeshTriangle& tri,
    const VoxelMeshBuildOptions& options,
    VoxelSpace& space,
    const VoxelTriangleInfluenceRange& influenceRange,
    const VoxelBounds& markBounds)
{
    VoxelTriangleMarkStats stats;
    const double halfDiag = space.GetHalfDiagonal();

    double effectiveClearance = options.clearance;

    if (options.conservativeClearance)
    {
        effectiveClearance += halfDiag;
    }

    VoxelIndex minIndex = influenceRange.minIndex;
    VoxelIndex maxIndex = influenceRange.maxIndex;

    if (markBounds.IsValid())
    {
        minIndex.x = std::max(minIndex.x, markBounds.minIndex.x);
        minIndex.y = std::max(minIndex.y, markBounds.minIndex.y);
        minIndex.z = std::max(minIndex.z, markBounds.minIndex.z);

        maxIndex.x = std::min(maxIndex.x, markBounds.maxIndex.x);
        maxIndex.y = std::min(maxIndex.y, markBounds.maxIndex.y);
        maxIndex.z = std::min(maxIndex.z, markBounds.maxIndex.z);
    }

    if (minIndex.x > maxIndex.x ||
        minIndex.y > maxIndex.y ||
        minIndex.z > maxIndex.z)
    {
        return stats;
    }

    for (int ix = minIndex.x;
        ix <= maxIndex.x;
        ++ix)
    {
        for (int iy = minIndex.y;
            iy <= maxIndex.y;
            ++iy)
        {
            for (int iz = minIndex.z;
                iz <= maxIndex.z;
                ++iz)
            {
                ++stats.voxelVisitCount;
                VoxelIndex index(ix, iy, iz);

                if (!space.IsInsideSearchBounds(index))
                {
                    ++stats.outOfBoundsVoxelCount;
                    continue;
                }

                Vec center = space.IndexToCenter(index);

                const double d = DistancePointToTriangle(center, tri);
                ++stats.distanceCalculationCount;

                const VoxelCell* oldCell = space.FindCell(index);
                const bool distanceWillImprove =
                    oldCell == nullptr ||
                    d < oldCell->distanceToSurface;

                space.SetCellDistanceIfSmaller(index, d);

                if (distanceWillImprove)
                {
                    ++stats.distanceImprovedCount;
                }
                else
                {
                    ++stats.distanceNotImprovedCount;
                }

                // Treat a cell as occupied when the triangle can cross it.
                if (d <= halfDiag)
                {
                    const VoxelState oldState =
                        space.GetCellState(index);
                    space.SetCellState(index, VoxelState::Occupied);
                    ++stats.stateWriteCount;
                    ++stats.occupiedWriteCount;
                    if (oldState == VoxelState::Occupied)
                    {
                        ++stats.stateUnchangedWriteCount;
                        ++stats.occupiedUnchangedWriteCount;
                    }
                    continue;
                }

                // 安全距离层体素：
                // 注意 ClearanceBand 是后续 A* 的候选运动区域。
                if (options.clearance > 0.0 && d <= effectiveClearance)
                {
                    VoxelState oldState = space.GetCellState(index);

                    if (oldState != VoxelState::Occupied)
                    {
                        space.SetCellState(
                            index,
                            VoxelState::ClearanceBand
                        );
                        ++stats.stateWriteCount;
                        ++stats.clearanceWriteCount;
                        if (oldState == VoxelState::ClearanceBand)
                        {
                            ++stats.stateUnchangedWriteCount;
                            ++stats.clearanceUnchangedWriteCount;
                        }
                    }
                }
            }
        }
    }

    return stats;
}

// ============================================================
// 主入口：TopoDS_Shape -> VoxelSpace
// ============================================================

VoxelMeshBuildResult VoxelMeshBuilder::BuildVoxelSpaceFromShapeMesh(
    const TopoDS_Shape& shape,
    const VoxelMeshBuildOptions& options,
    VoxelSpace& outSpace)
{
    VoxelMeshBuildResult result;

    outSpace.Clear();

    if (shape.IsNull())
    {
        return result;
    }

    if (options.voxelSize <= 0.0)
    {
        return result;
    }

    std::vector<MeshTriangle> triangles;

    if (!BuildShapeTriangulation(
        shape,
        options.meshDeflection,
        options.angularDeflection,
        triangles))
    {
        return result;
    }

    return BuildVoxelSpaceFromTriangles(triangles, options, outSpace);
}

VoxelMeshBuildResult VoxelMeshBuilder::BuildVoxelSpaceFromTriangles(
    const std::vector<MeshTriangle>& triangles,
    const VoxelMeshBuildOptions& options,
    VoxelSpace& outSpace)
{
    VoxelMeshBuildResult result;
    const auto totalStart = Now();

    outSpace.Clear();

    if (options.voxelSize <= 0.0)
    {
        return result;
    }

    MeshAABB globalBox;

    if (!ComputeTrianglesAABB(triangles, globalBox))
    {
        return result;
    }

    const double globalExpand =
        std::max(0.0, options.bboxPadding) +
        std::max(0.0, options.clearance) +
        options.voxelSize;

    ExpandAABB(globalBox, globalExpand);

    outSpace = VoxelSpace(globalBox.minP, options.voxelSize);

    VoxelBounds bounds;
    bounds.minIndex = outSpace.WorldToIndex(globalBox.minP);
    bounds.maxIndex = outSpace.WorldToIndex(globalBox.maxP);

    NormalizeIndexRange(bounds.minIndex, bounds.maxIndex);

    outSpace.SetSearchBounds(bounds);

    if (options.storeFreeCells)
    {
        StoreFreeCellsInBounds(bounds, outSpace);
    }

    const auto markStart = Now();
    for (const MeshTriangle& tri : triangles)
    {
        const VoxelTriangleInfluenceRange range =
            ComputeTriangleInfluenceRange(tri, options, outSpace);
        AddMarkStats(
            result,
            MarkTriangleToVoxelSpace(
                tri,
                options,
                outSpace,
                range,
                bounds));
    }
    result.voxelMarkMs = ElapsedMs(markStart, Now());

    std::size_t occupiedCount = 0;
    std::size_t clearanceBandCount = 0;

    for (const auto& kv : outSpace.Cells())
    {
        if (kv.second.state == VoxelState::Occupied)
        {
            ++occupiedCount;
        }
        else if (kv.second.state == VoxelState::ClearanceBand)
        {
            ++clearanceBandCount;
        }
    }

    result.success = true;
    result.triangleCount = triangles.size();
    result.candidateTriangleCount = triangles.size();
    result.rawCandidateTriangleCount = triangles.size();
    result.occupiedVoxelCount = occupiedCount;
    result.clearanceBandVoxelCount = clearanceBandCount;
    result.bounds = bounds;

    return result;
}

VoxelMeshBuildResult VoxelMeshBuilder::BuildVoxelSpaceFromShapeMeshInBox(
    const TopoDS_Shape& shape,
    const MeshAABB& buildBox,
    const VoxelMeshBuildOptions& options,
    VoxelSpace& outSpace)
{
    VoxelMeshBuildResult result;
    const auto totalStart = Now();

    outSpace.Clear();

    if (shape.IsNull())
    {
        return result;
    }

    if (options.voxelSize <= 0.0)
    {
        return result;
    }

    if (buildBox.minP.x > buildBox.maxP.x ||
        buildBox.minP.y > buildBox.maxP.y ||
        buildBox.minP.z > buildBox.maxP.z)
    {
        return result;
    }

    std::vector<MeshTriangle> triangles;

    if (!BuildShapeTriangulation(
        shape,
        options.meshDeflection,
        options.angularDeflection,
        triangles))
    {
        return result;
    }

    return BuildVoxelSpaceFromTrianglesInBox(
        triangles,
        nullptr,
        buildBox,
        options,
        outSpace
    );
}

VoxelMeshBuildResult VoxelMeshBuilder::BuildVoxelSpaceFromTrianglesInBox(
    const std::vector<MeshTriangle>& triangles,
    const TriangleSpatialHash* spatialHash,
    const MeshAABB& buildBox,
    const VoxelMeshBuildOptions& options,
    VoxelSpace& outSpace)
{
    VoxelMeshBuildResult result;
    const auto totalStart = Now();

    outSpace.Clear();

    if (options.voxelSize <= 0.0)
    {
        return result;
    }

    if (triangles.empty())
    {
        return result;
    }

    if (buildBox.minP.x > buildBox.maxP.x ||
        buildBox.minP.y > buildBox.maxP.y ||
        buildBox.minP.z > buildBox.maxP.z)
    {
        return result;
    }

    outSpace = VoxelSpace(buildBox.minP, options.voxelSize);

    VoxelBounds bounds;
    bounds.minIndex = outSpace.WorldToIndex(buildBox.minP);
    bounds.maxIndex = outSpace.WorldToIndex(buildBox.maxP);

    NormalizeIndexRange(bounds.minIndex, bounds.maxIndex);

    outSpace.SetSearchBounds(bounds);

    if (options.storeFreeCells)
    {
        StoreFreeCellsInBounds(bounds, outSpace);
    }

    const double halfDiag = outSpace.GetHalfDiagonal();
    std::size_t candidateTriangleCount = 0;
    std::vector<int> candidateTriangleIds;

    MeshAABB queryBox = buildBox;
    double queryExpand = options.clearance;

    if (options.conservativeClearance)
    {
        queryExpand += halfDiag;
    }

    ExpandAABB(queryBox, queryExpand);

    const auto candidateQueryStart = Now();
    if (spatialHash != nullptr && spatialHash->IsValid())
    {
        TriangleSpatialHashStats queryStats;
        spatialHash->Query(queryBox, candidateTriangleIds, queryStats);
        result.hashQueryCellCount = queryStats.queryCellCount;
        result.hashRawTriangleCount = queryStats.queryRawTriangleCount;
    }
    else
    {
        candidateTriangleIds.reserve(triangles.size());

        for (std::size_t i = 0; i < triangles.size(); ++i)
        {
            candidateTriangleIds.push_back(static_cast<int>(i));
        }
    }
    result.candidateQueryMs = ElapsedMs(candidateQueryStart, Now());

    std::vector<int> filteredTriangleIds;
    std::vector<VoxelTriangleInfluenceRange> filteredInfluenceRanges;
    filteredTriangleIds.reserve(candidateTriangleIds.size());
    filteredInfluenceRanges.reserve(candidateTriangleIds.size());

    const auto candidateFilterStart = Now();
    for (int triangleId : candidateTriangleIds)
    {
        if (triangleId < 0 ||
            static_cast<std::size_t>(triangleId) >= triangles.size())
        {
            continue;
        }

        const MeshTriangle& tri = triangles[triangleId];

        const VoxelTriangleInfluenceRange influenceRange =
            ComputeTriangleInfluenceRange(tri, options, outSpace);

        if (!IntersectsAABB(influenceRange.influenceBox, buildBox))
        {
            continue;
        }

        ++candidateTriangleCount;
        filteredTriangleIds.push_back(triangleId);
        filteredInfluenceRanges.push_back(influenceRange);
    }
    result.candidateFilterMs = ElapsedMs(candidateFilterStart, Now());

    const auto markStart = Now();
    for (std::size_t i = 0; i < filteredTriangleIds.size(); ++i)
    {
        const int triangleId = filteredTriangleIds[i];
        AddMarkStats(
            result,
            MarkTriangleToVoxelSpace(
                triangles[static_cast<std::size_t>(triangleId)],
                options,
                outSpace,
                filteredInfluenceRanges[i],
                bounds));
    }
    result.voxelMarkMs = ElapsedMs(markStart, Now());

    std::size_t occupiedCount = 0;
    std::size_t clearanceBandCount = 0;

    const auto stateCountStart = Now();
    for (const auto& kv : outSpace.Cells())
    {
        if (kv.second.state == VoxelState::Occupied)
        {
            ++occupiedCount;
        }
        else if (kv.second.state == VoxelState::ClearanceBand)
        {
            ++clearanceBandCount;
        }
    }
    result.stateCountMs = ElapsedMs(stateCountStart, Now());

    result.success = true;
    result.triangleCount = triangles.size();
    result.candidateTriangleCount = candidateTriangleCount;
    result.rawCandidateTriangleCount = candidateTriangleIds.size();
    result.occupiedVoxelCount = occupiedCount;
    result.clearanceBandVoxelCount = clearanceBandCount;
    result.bounds = bounds;
    result.totalBuildMs = ElapsedMs(totalStart, Now());

    return result;
}

VoxelMeshBuildResult VoxelMeshBuilder::AppendVoxelSpaceFromTrianglesInBox(
    const std::vector<MeshTriangle>& triangles,
    const TriangleSpatialHash* spatialHash,
    const MeshAABB& buildBox,
    const VoxelMeshBuildOptions& options,
    VoxelSpace& outSpace,
    double extraQueryPadding,
    VoxelMeshBuildCache* buildCache)
{
    VoxelMeshBuildResult result;
    const auto totalStart = Now();

    if (options.voxelSize <= 0.0 || !outSpace.IsValid())
    {
        return result;
    }

    if (triangles.empty())
    {
        return result;
    }

    if (buildBox.minP.x > buildBox.maxP.x ||
        buildBox.minP.y > buildBox.maxP.y ||
        buildBox.minP.z > buildBox.maxP.z)
    {
        return result;
    }

    VoxelBounds appendBounds;
    appendBounds.minIndex = outSpace.WorldToIndex(buildBox.minP);
    appendBounds.maxIndex = outSpace.WorldToIndex(buildBox.maxP);

    NormalizeIndexRange(appendBounds.minIndex, appendBounds.maxIndex);

    const bool hadOriginalBounds = outSpace.HasSearchBounds();
    const VoxelBounds originalBounds = outSpace.GetSearchBounds();
    VoxelBounds combinedBounds = appendBounds;

    if (hadOriginalBounds && originalBounds.IsValid())
    {
        combinedBounds = originalBounds;
        ExpandBoundsToInclude(combinedBounds, appendBounds.minIndex);
        ExpandBoundsToInclude(combinedBounds, appendBounds.maxIndex);
    }

    outSpace.SetSearchBounds(appendBounds);

    if (options.storeFreeCells)
    {
        StoreFreeCellsInBounds(appendBounds, outSpace);
    }

    const double halfDiag = outSpace.GetHalfDiagonal();
    std::size_t candidateTriangleCount = 0;
    std::vector<int> candidateTriangleIds;

    MeshAABB queryBox = buildBox;
    double queryExpand = options.clearance;

    if (options.conservativeClearance)
    {
        queryExpand += halfDiag;
    }

    queryExpand += std::max(0.0, extraQueryPadding);

    ExpandAABB(queryBox, queryExpand);

    const auto candidateQueryStart = Now();
    if (spatialHash != nullptr && spatialHash->IsValid())
    {
        TriangleSpatialHashStats queryStats;
        spatialHash->Query(queryBox, candidateTriangleIds, queryStats);
        result.hashQueryCellCount = queryStats.queryCellCount;
        result.hashRawTriangleCount = queryStats.queryRawTriangleCount;
    }
    else
    {
        candidateTriangleIds.reserve(triangles.size());

        for (std::size_t i = 0; i < triangles.size(); ++i)
        {
            candidateTriangleIds.push_back(static_cast<int>(i));
        }
    }
    result.candidateQueryMs = ElapsedMs(candidateQueryStart, Now());

    std::vector<int> filteredTriangleIds;
    std::vector<VoxelTriangleInfluenceRange> filteredInfluenceRanges;
    filteredTriangleIds.reserve(candidateTriangleIds.size());
    filteredInfluenceRanges.reserve(candidateTriangleIds.size());

    const auto candidateFilterStart = Now();
    for (int triangleId : candidateTriangleIds)
    {
        if (triangleId < 0 ||
            static_cast<std::size_t>(triangleId) >= triangles.size())
        {
            continue;
        }

        VoxelTriangleInfluenceRange influenceRange;

        if (buildCache != nullptr)
        {
            const auto cacheIt =
                buildCache->triangleInfluence.find(triangleId);

            if (cacheIt != buildCache->triangleInfluence.end())
            {
                influenceRange = cacheIt->second;
                ++result.influenceCacheHitCount;
            }
            else
            {
                influenceRange = ComputeTriangleInfluenceRange(
                    triangles[static_cast<std::size_t>(triangleId)],
                    options,
                    outSpace);
                buildCache->triangleInfluence.emplace(
                    triangleId,
                    influenceRange);
                ++result.influenceCacheMissCount;
            }
        }
        else
        {
            influenceRange = ComputeTriangleInfluenceRange(
                triangles[static_cast<std::size_t>(triangleId)],
                options,
                outSpace);
            ++result.influenceCacheMissCount;
        }

        if (!IntersectsAABB(influenceRange.influenceBox, buildBox))
        {
            continue;
        }

        ++candidateTriangleCount;
        filteredTriangleIds.push_back(triangleId);
        filteredInfluenceRanges.push_back(influenceRange);
    }
    result.candidateFilterMs = ElapsedMs(candidateFilterStart, Now());

    const auto markStart = Now();
    for (std::size_t i = 0; i < filteredTriangleIds.size(); ++i)
    {
        const int triangleId = filteredTriangleIds[i];
        AddMarkStats(
            result,
            MarkTriangleToVoxelSpace(
                triangles[static_cast<std::size_t>(triangleId)],
                options,
                outSpace,
                filteredInfluenceRanges[i],
                appendBounds));
    }
    result.voxelMarkMs = ElapsedMs(markStart, Now());

    outSpace.SetSearchBounds(combinedBounds);

    result.success = true;
    result.triangleCount = triangles.size();
    result.candidateTriangleCount = candidateTriangleCount;
    result.rawCandidateTriangleCount = candidateTriangleIds.size();
    result.bounds = appendBounds;
    result.totalBuildMs = ElapsedMs(totalStart, Now());

    return result;
}
