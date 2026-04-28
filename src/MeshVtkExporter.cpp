#include "MeshVtkExporter.h"

#include <fstream>
#include <iomanip>

bool MeshVtkExporter::ExportTrianglesToVtk(
    const std::vector<MeshTriangle>& triangles,
    const std::string& filePath)
{
    if (triangles.empty())
    {
        return false;
    }

    std::ofstream ofs(filePath.c_str(), std::ios::out);
    if (!ofs.is_open())
    {
        return false;
    }

    const std::size_t triangleCount = triangles.size();
    const std::size_t pointCount = triangleCount * 3;

    ofs << "# vtk DataFile Version 3.0\n";
    ofs << "OCCT shape triangulation mesh\n";
    ofs << "ASCII\n";
    ofs << "DATASET POLYDATA\n";

    ofs << std::setprecision(15);

    // ============================================================
    // POINTS
    // 每个三角形独立写 3 个点。
    // 优点：简单稳定。
    // 缺点：共享顶点不会复用，文件稍大。
    // ============================================================

    ofs << "POINTS " << pointCount << " double\n";

    for (const MeshTriangle& tri : triangles)
    {
        ofs << tri.p0.x << " "
            << tri.p0.y << " "
            << tri.p0.z << "\n";

        ofs << tri.p1.x << " "
            << tri.p1.y << " "
            << tri.p1.z << "\n";

        ofs << tri.p2.x << " "
            << tri.p2.y << " "
            << tri.p2.z << "\n";
    }

    // ============================================================
    // POLYGONS
    // 每个三角形格式：
    // 3 p0 p1 p2
    //
    // POLYGONS 后第二个数字是总整数数：
    // triangleCount * 4
    // ============================================================

    ofs << "POLYGONS "
        << triangleCount << " "
        << triangleCount * 4 << "\n";

    for (std::size_t i = 0; i < triangleCount; ++i)
    {
        const std::size_t base = i * 3;

        ofs << "3 "
            << base + 0 << " "
            << base + 1 << " "
            << base + 2 << "\n";
    }

    // ============================================================
    // CELL_DATA
    // 输出 triangle_id，方便 ParaView 检查
    // ============================================================

    ofs << "CELL_DATA " << triangleCount << "\n";

    ofs << "SCALARS triangle_id int 1\n";
    ofs << "LOOKUP_TABLE default\n";

    for (std::size_t i = 0; i < triangleCount; ++i)
    {
        ofs << i << "\n";
    }

    ofs.close();

    return true;
}

bool MeshVtkExporter::ExportShapeMeshToVtk(
    const TopoDS_Shape& shape,
    const std::string& filePath,
    double deflection,
    double angularDeflection)
{
    if (shape.IsNull())
    {
        return false;
    }

    std::vector<MeshTriangle> triangles;

    bool ok = VoxelMeshBuilder::BuildShapeTriangulation(
        shape,
        deflection,
        angularDeflection,
        triangles
    );

    if (!ok || triangles.empty())
    {
        return false;
    }

    return ExportTrianglesToVtk(triangles, filePath);
}