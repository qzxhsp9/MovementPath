#pragma once

#include "VoxelSpace.h"

#include <vector>
#include <cstddef>

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

// ============================================================
// 三角形
// ============================================================

struct MeshTriangle
{
    gp_Pnt p0;
    gp_Pnt p1;
    gp_Pnt p2;
};

// ============================================================
// AABB
// ============================================================

struct MeshAABB
{
    gp_Pnt minP;
    gp_Pnt maxP;
};

// ============================================================
// 构建选项
// ============================================================

struct VoxelMeshBuildOptions
{
    // 体素尺寸
    double voxelSize = 1.0;

    // 三角化弦高误差
    double meshDeflection = 0.1;

    // 三角化角度误差，单位弧度
    double angularDeflection = 0.5;

    // 安全距离层厚度
    double clearance = 0.0;

    // VoxelSpace 包围盒外扩距离
    double bboxPadding = 0.0;

    // 是否把体素半对角线计入安全距离。
    // true 更保守。
    bool conservativeClearance = true;

    // 是否显式存储 Free 体素。
    // 稀疏体素建议 false。
    bool storeFreeCells = false;
};

// ============================================================
// 构建结果
// ============================================================

struct VoxelMeshBuildResult
{
    bool success = false;

    std::size_t triangleCount = 0;
    std::size_t occupiedVoxelCount = 0;
    std::size_t clearanceBandVoxelCount = 0;

    VoxelBounds bounds;
};

// ============================================================
// VoxelMeshBuilder
// ============================================================

class VoxelMeshBuilder
{
public:
    static bool BuildShapeTriangulation(
        const TopoDS_Shape& shape,
        double deflection,
        double angularDeflection,
        std::vector<MeshTriangle>& outTriangles);

    static VoxelMeshBuildResult BuildVoxelSpaceFromShapeMesh(
        const TopoDS_Shape& shape,
        const VoxelMeshBuildOptions& options,
        VoxelSpace& outSpace);

private:
    static MeshAABB ComputeTriangleAABB(
        const MeshTriangle& tri);

    static bool ComputeTrianglesAABB(
        const std::vector<MeshTriangle>& triangles,
        MeshAABB& outBox);

    static void ExpandAABB(
        MeshAABB& box,
        double offset);

    static double DistancePointToTriangle(
        const Vec& p,
        const MeshTriangle& tri);

    static void NormalizeIndexRange(
        VoxelIndex& minIndex,
        VoxelIndex& maxIndex);

    static void MarkTriangleToVoxelSpace(
        const MeshTriangle& tri,
        const VoxelMeshBuildOptions& options,
        VoxelSpace& space);

    static void StoreFreeCellsInBounds(
        const VoxelBounds& bounds,
        VoxelSpace& space);
};