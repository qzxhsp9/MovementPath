#pragma once

#include "VoxelSpace.h"

#include <vector>
#include <cstddef>
#include <unordered_map>

#include <TopoDS_Shape.hxx>

// ============================================================
// 三角形
// ============================================================

struct MeshTriangle
{
    Vec p0;
    Vec p1;
    Vec p2;
};

// ============================================================
// AABB
// ============================================================

struct MeshAABB
{
    Vec minP;
    Vec maxP;
};

struct SpatialCellIndex
{
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const SpatialCellIndex& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct SpatialCellIndexHash
{
    std::size_t operator()(const SpatialCellIndex& index) const
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

struct TriangleSpatialHashOptions
{
    double cellSize = 10.0;
};

struct TriangleSpatialHashStats
{
    std::size_t cellCount = 0;
    std::size_t entryCount = 0;
    std::size_t queryCellCount = 0;
    std::size_t queryRawTriangleCount = 0;
    std::size_t queryUniqueTriangleCount = 0;
};

class TriangleSpatialHash
{
public:
    bool Build(
        const std::vector<MeshTriangle>& triangles,
        const TriangleSpatialHashOptions& options);

    bool IsValid() const;

    void Query(
        const MeshAABB& queryBox,
        std::vector<int>& outTriangleIds) const;

    void Query(
        const MeshAABB& queryBox,
        std::vector<int>& outTriangleIds,
        TriangleSpatialHashStats& outStats) const;

    double GetCellSize() const;

    TriangleSpatialHashStats GetStats() const;

private:
    SpatialCellIndex WorldToCell(const Vec& point) const;

private:
    Vec m_origin;
    double m_cellSize = 0.0;

    std::unordered_map<
        SpatialCellIndex,
        std::vector<int>,
        SpatialCellIndexHash> m_cells;

    std::size_t m_entryCount = 0;
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

    // Timing breakdown used by lazy chunk benchmarks. Full builds mostly
    // report voxelMarkMs/stateCountMs; local and chunk builds also report
    // candidate query/filter costs.
    double candidateQueryMs = 0.0;
    double candidateFilterMs = 0.0;
    double voxelMarkMs = 0.0;
    double stateCountMs = 0.0;
    double totalBuildMs = 0.0;

    std::size_t triangleCount = 0;
    std::size_t candidateTriangleCount = 0;
    std::size_t rawCandidateTriangleCount = 0;
    std::size_t hashQueryCellCount = 0;
    std::size_t hashRawTriangleCount = 0;
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

    static VoxelMeshBuildResult BuildVoxelSpaceFromTriangles(
        const std::vector<MeshTriangle>& triangles,
        const VoxelMeshBuildOptions& options,
        VoxelSpace& outSpace);

    static VoxelMeshBuildResult BuildVoxelSpaceFromShapeMeshInBox(
        const TopoDS_Shape& shape,
        const MeshAABB& buildBox,
        const VoxelMeshBuildOptions& options,
        VoxelSpace& outSpace);

    static VoxelMeshBuildResult BuildVoxelSpaceFromTrianglesInBox(
        const std::vector<MeshTriangle>& triangles,
        const TriangleSpatialHash* spatialHash,
        const MeshAABB& buildBox,
        const VoxelMeshBuildOptions& options,
        VoxelSpace& outSpace);

    static VoxelMeshBuildResult AppendVoxelSpaceFromTrianglesInBox(
        const std::vector<MeshTriangle>& triangles,
        const TriangleSpatialHash* spatialHash,
        const MeshAABB& buildBox,
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

    static bool IntersectsAABB(
        const MeshAABB& a,
        const MeshAABB& b);

    static MeshAABB ComputeTriangleInfluenceAABB(
        const MeshTriangle& tri,
        const VoxelMeshBuildOptions& options,
        double halfDiag);

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
