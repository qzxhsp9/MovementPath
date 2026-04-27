#include "VoxelSpace.h"
#include "VoxelMeshBuilder.h"
#include "VoxelAStar.h"
#include "VoxelVtkExporter.h"

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepTools.hxx>

#include <iostream>

void TestVoxelAStar(
    const TopoDS_Shape& shape,
    const gp_Pnt& startPoint,
    const gp_Pnt& goalPoint)
{
    VoxelMeshBuildOptions buildOptions;

    buildOptions.voxelSize = 1.0;
    buildOptions.meshDeflection = 0.25;
    buildOptions.angularDeflection = 0.5;

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
    VoxelIndex startIndex = voxelSpace.WorldToIndex(startPoint);
    VoxelIndex goalIndex = voxelSpace.WorldToIndex(goalPoint);

    VoxelBounds bounds = voxelSpace.GetSearchBounds();

    ExpandBoundsToInclude(bounds, startIndex);
    ExpandBoundsToInclude(bounds, goalIndex);
    ExpandBoundsByVoxelRadius(bounds, 10);

    voxelSpace.SetSearchBounds(bounds);

    VoxelAStarOptions astarOptions;

    astarOptions.searchMode = VoxelAStarSearchMode::ClearanceBand;
    astarOptions.neighborType = VoxelNeighborType::Face6;
    astarOptions.heuristicWeight = 1.0;
    astarOptions.turnPenalty = voxelSpace.GetVoxelSize() * 0.3;

    // 起点终点如果不在 ClearanceBand 中，自动吸附到最近 ClearanceBand 体素
    astarOptions.snapStartGoalToWalkable = true;
    astarOptions.snapMaxRadius = 30;

    astarOptions.maxVisitedCount = 0;
    astarOptions.markPathToVoxelSpace = true;

    VoxelAStarResult astarResult =
        VoxelAStar::Search(
            voxelSpace,
            startPoint,
            goalPoint,
            astarOptions
        );

    if (!astarResult.success)
    {
        std::cout << "A* failed." << std::endl;
        std::cout << "Visited count: "
            << astarResult.visitedCount << std::endl;

        VoxelVtkExporter::ExportVoxelSpaceToVtk(
            voxelSpace,
            "D:/data/workspace/astar_failed.vtk",
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
        "D:/data/workspace/astar_path.vtk",
        {
            VoxelState::Occupied,
            VoxelState::ClearanceBand,
            VoxelState::Start,
            VoxelState::Goal,
            VoxelState::Path
        }
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

    TestVoxelAStar(sphere, gp_Pnt(0, 0, -50), gp_Pnt(R, 0, 50));
    return 0;
}