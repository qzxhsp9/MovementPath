#include "VoxelMeshBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

bool VoxelMeshBuilder::ComputeTrianglesAABB(
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
        MeshAABB box = ComputeTriangleAABB(tri);

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

void VoxelMeshBuilder::ExpandAABB(
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

void VoxelMeshBuilder::MarkTriangleToVoxelSpace(
    const MeshTriangle& tri,
    const VoxelMeshBuildOptions& options,
    VoxelSpace& space)
{
    MeshAABB triBox = ComputeTriangleAABB(tri);

    const double halfDiag = space.GetHalfDiagonal();

    double effectiveClearance = options.clearance;

    if (options.conservativeClearance)
    {
        effectiveClearance += halfDiag;
    }

    MeshAABB expandedBox = triBox;

    // 为了找出所有可能受该三角形影响的体素，需要按有效安全距离扩展三角形 AABB。
    ExpandAABB(expandedBox, effectiveClearance);

    VoxelIndex minIndex = space.WorldToIndex(expandedBox.minP);
    VoxelIndex maxIndex = space.WorldToIndex(expandedBox.maxP);

    NormalizeIndexRange(minIndex, maxIndex);

    for (int ix = minIndex.x; ix <= maxIndex.x; ++ix)
    {
        for (int iy = minIndex.y; iy <= maxIndex.y; ++iy)
        {
            for (int iz = minIndex.z; iz <= maxIndex.z; ++iz)
            {
                VoxelIndex index(ix, iy, iz);

                Vec center = space.IndexToCenter(index);

                const double d = DistancePointToTriangle(center, tri);

                space.SetCellDistanceIfSmaller(index, d);

                // 与表面相交的体素：
                // 体素中心到三角形距离 <= 半体素对角线，认为三角形可能穿过该体素。
                if (d <= halfDiag)
                {
                    space.SetCellState(index, VoxelState::Occupied);
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
                    }
                }
            }
        }
    }
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

    for (const MeshTriangle& tri : triangles)
    {
        MarkTriangleToVoxelSpace(tri, options, outSpace);
    }

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
    result.occupiedVoxelCount = occupiedCount;
    result.clearanceBandVoxelCount = clearanceBandCount;
    result.bounds = bounds;

    return result;
}