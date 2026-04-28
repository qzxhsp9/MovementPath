#include "VoxelSpace.h"
#include "VoxelMeshBuilder.h"
#include "VoxelAStar.h"
#include "VoxelVtkExporter.h"
#include "VoxelPathOptimizer.h"
#include "MeshVtkExporter.h"

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepTools.hxx>

#include <iostream>

static void DebugVoxelStateAround(
    const VoxelSpace& space,
    const VoxelIndex& seed,
    int radius)
{
    int freeCount = 0;
    int occupiedCount = 0;
    int clearanceBandCount = 0;
    int startCount = 0;
    int goalCount = 0;
    int pathCount = 0;
    int outOfBoundsCount = 0;

    for (int dx = -radius; dx <= radius; ++dx)
    {
        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dz = -radius; dz <= radius; ++dz)
            {
                VoxelIndex index(
                    seed.x + dx,
                    seed.y + dy,
                    seed.z + dz
                );

                if (!space.IsInsideSearchBounds(index))
                {
                    ++outOfBoundsCount;
                    continue;
                }

                VoxelState state = space.GetCellState(index);

                switch (state)
                {
                case VoxelState::Free:
                    ++freeCount;
                    break;
                case VoxelState::Occupied:
                    ++occupiedCount;
                    break;
                case VoxelState::ClearanceBand:
                    ++clearanceBandCount;
                    break;
                case VoxelState::Start:
                    ++startCount;
                    break;
                case VoxelState::Goal:
                    ++goalCount;
                    break;
                case VoxelState::Path:
                    ++pathCount;
                    break;
                default:
                    break;
                }
            }
        }
    }

    std::cout << "Debug around index: "
        << seed.x << ", "
        << seed.y << ", "
        << seed.z << std::endl;

    std::cout << "radius: " << radius << std::endl;
    std::cout << "Free: " << freeCount << std::endl;
    std::cout << "Occupied: " << occupiedCount << std::endl;
    std::cout << "ClearanceBand: " << clearanceBandCount << std::endl;
    std::cout << "Start: " << startCount << std::endl;
    std::cout << "Goal: " << goalCount << std::endl;
    std::cout << "Path: " << pathCount << std::endl;
    std::cout << "OutOfBounds: " << outOfBoundsCount << std::endl;
}

void TestVoxelAStar(
    const TopoDS_Shape& shape,
    const gp_Pnt& startPoint,
    const gp_Vec& startDir,
    const gp_Pnt& goalPoint,
    const gp_Vec& goalDir)
{
    VoxelMeshBuildOptions buildOptions;

    buildOptions.voxelSize = 1.0;
    buildOptions.meshDeflection = 0.25;
    buildOptions.angularDeflection = 0.3;

    bool ok = MeshVtkExporter::ExportShapeMeshToVtk(
        shape,
        "D:/shape_mesh.vtk",
        buildOptions.meshDeflection,
        buildOptions.angularDeflection
    );

    if (!ok)
    {
        std::cout << "Export shape mesh failed." << std::endl;
    }
    else
    {
        std::cout << "Export shape mesh success." << std::endl;
    }

    // 路径要在该安全距离层中搜索
    buildOptions.clearance = 3.0;

    buildOptions.bboxPadding = 10.0;
    buildOptions.conservativeClearance = true;
    buildOptions.storeFreeCells = false;

    VoxelSpace voxelSpace;

    VoxelMeshBuildResult buildResult =
        VoxelMeshBuilder::BuildVoxelSpaceFromShapeMesh(
            shape,
            buildOptions,
            voxelSpace
        );

    if (!buildResult.success)
    {
        std::cout << "Build voxel space failed." << std::endl;
        return;
    }

    // 确保搜索范围包含起点和终点
    Vec startPoint3D = {startPoint.X(), startPoint.Y(), startPoint.Z()};
    Vec goalPoint3D = {goalPoint.X(), goalPoint.Y(), goalPoint.Z()};
    VoxelIndex startIndex = voxelSpace.WorldToIndex(startPoint3D);
    VoxelIndex goalIndex = voxelSpace.WorldToIndex(goalPoint3D);

    DebugVoxelStateAround(voxelSpace, startIndex, 10);
    DebugVoxelStateAround(voxelSpace, goalIndex, 10);

    VoxelBounds bounds = voxelSpace.GetSearchBounds();

    ExpandBoundsToInclude(bounds, startIndex);
    ExpandBoundsToInclude(bounds, goalIndex);
    ExpandBoundsByVoxelRadius(bounds, 10);

    voxelSpace.SetSearchBounds(bounds);

    VoxelAStarOptions astarOptions;

    astarOptions.searchMode = VoxelAStarSearchMode::ClearanceBand;
    astarOptions.neighborType = VoxelNeighborType::Face6;
    astarOptions.heuristicWeight = 1.0;
    astarOptions.turnPenalty = voxelSpace.GetVoxelSize() * 0.1;

    // 起点终点如果不在 ClearanceBand 中，自动吸附到最近 ClearanceBand 体素
    astarOptions.snapStartGoalToWalkable = true;
    astarOptions.snapMaxRadius = 20;

    astarOptions.useStartSnapDirection = true;
    astarOptions.useGoalSnapDirection = true;

    astarOptions.startSnapDirection = { startDir.X(), startDir.Y(), startDir.Z() };
    astarOptions.goalSnapDirection = { goalDir.X(), goalDir.Y(), goalDir.Z() };

    astarOptions.maxVisitedCount = 0;
    astarOptions.markPathToVoxelSpace = true;

    VoxelAStarResult astarResult =
        VoxelAStar::Search(
            voxelSpace,
            startPoint3D,
            goalPoint3D,
            astarOptions
        );

    if (!astarResult.success)
    {
        std::cout << "A* failed." << std::endl;
        std::cout << "Visited count: "
            << astarResult.visitedCount << std::endl;

        std::cout << "input start: "
            << astarResult.inputStartIndex.x << ", "
            << astarResult.inputStartIndex.y << ", "
            << astarResult.inputStartIndex.z << std::endl;

        std::cout << "snapped start: "
            << astarResult.startIndex.x << ", "
            << astarResult.startIndex.y << ", "
            << astarResult.startIndex.z << std::endl;

        std::cout << "input goal: "
            << astarResult.inputGoalIndex.x << ", "
            << astarResult.inputGoalIndex.y << ", "
            << astarResult.inputGoalIndex.z << std::endl;

        std::cout << "snapped goal: "
            << astarResult.goalIndex.x << ", "
            << astarResult.goalIndex.y << ", "
            << astarResult.goalIndex.z << std::endl;

        VoxelVtkExporter::ExportVoxelSpaceToVtk(
            voxelSpace,
            "D:/astar_failed.vtk",
            {
                VoxelState::Occupied,
                VoxelState::ClearanceBand,
                VoxelState::Start,
                VoxelState::Goal
            }
        );

        return;
    }

    std::cout << "A* success." << std::endl;
    std::cout << "Visited count: "
        << astarResult.visitedCount << std::endl;
    std::cout << "Path voxel count: "
        << astarResult.voxelPath.size() << std::endl;
    std::cout << "Total cost: "
        << astarResult.totalCost << std::endl;

    VoxelVtkExporter::ExportVoxelSpaceToVtk(
        voxelSpace,
        "D:/astar_path.vtk",
        {
            VoxelState::Start,
            VoxelState::Goal,
            VoxelState::Path
        }
    );

    VoxelPathOptimizeOptions optOptions;

    optOptions.searchMode = astarOptions.searchMode;
    optOptions.removeCollinear = true;
    optOptions.enableLineOfSightShortcut = true;

    // 路径点不多时可以设大一点。
    // 工程中建议 100~300，避免 O(n^2) 太重。
    optOptions.maxShortcutLookAhead = 200;

    VoxelPathOptimizeResult optResult =
        VoxelPathOptimizer::Optimize(
            voxelSpace,
            astarResult.voxelPath,
            optOptions
        );

    std::cout << "Path optimize result:" << std::endl;
    std::cout << "Input count: "
        << optResult.inputCount << std::endl;
    std::cout << "After collinear: "
        << optResult.afterCollinearCount << std::endl;
    std::cout << "Output count: "
        << optResult.outputCount << std::endl;
    std::cout << "Line check count: "
        << optResult.lineCheckCount << std::endl;

    VoxelVtkExporter::MarkPathToVoxelSpace(
        voxelSpace,
        optResult.voxelPath
    );

    VoxelVtkExporter::ExportVoxelSpaceToVtk(
        voxelSpace,
        "D:/optimized_path_voxels.vtk",
        {
            VoxelState::Occupied,
            VoxelState::ClearanceBand,
            VoxelState::Start,
            VoxelState::Goal,
            VoxelState::Path
        }
    );

    VoxelVtkExporter::ExportPathPolylineToVtk(
        voxelSpace,
        optResult.voxelPath,
        "D:/optimized_path_polyline.vtk"
    );
}

int main()
{
    TopoDS_Shape shape;
    
    BRep_Builder builder;
    BRepTools::Read(shape, "d:/shape.brep", builder);

    const double R = 50.0;
    TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), R).Shape();

    // TestBuildVoxelSpace(sphere);

    TestVoxelAStar(sphere, gp_Pnt(0, 0, -50), gp_Vec(0, 0, -1), gp_Pnt(0, 0, 50), gp_Vec(0, 0, 1));
    return 0;
}