#pragma once

#include "VoxelMeshBuilder.h"

#include <string>
#include <vector>

class MeshVtkExporter
{
public:
    // 将三角网格导出为 VTK PolyData
    static bool ExportTrianglesToVtk(
        const std::vector<MeshTriangle>& triangles,
        const std::string& filePath);

    // 直接从 TopoDS_Shape 三角化并导出
    static bool ExportShapeMeshToVtk(
        const TopoDS_Shape& shape,
        const std::string& filePath,
        double deflection = 0.1,
        double angularDeflection = 0.5);
};