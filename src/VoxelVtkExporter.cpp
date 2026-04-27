#include "VoxelVtkExporter.h"

#include <fstream>
#include <iomanip>
#include <vector>

#include <gp_Pnt.hxx>

bool VoxelVtkExporter::ShouldExportState(
    VoxelState state,
    const std::set<VoxelState>& statesToExport)
{
    return statesToExport.find(state) != statesToExport.end();
}

int VoxelVtkExporter::StateToInt(VoxelState state)
{
    switch (state)
    {
    case VoxelState::Free:
        return 0;
    case VoxelState::Occupied:
        return 1;
    case VoxelState::ClearanceBand:
        return 2;
    case VoxelState::Start:
        return 3;
    case VoxelState::Goal:
        return 4;
    case VoxelState::Path:
        return 5;
    default:
        return -1;
    }
}

bool VoxelVtkExporter::ExportVoxelSpaceToVtk(
    const VoxelSpace& space,
    const std::string& filePath,
    const std::set<VoxelState>& statesToExport)
{
    if (!space.IsValid())
    {
        return false;
    }

    struct ExportCell
    {
        VoxelIndex index;
        VoxelState state = VoxelState::Free;
        double distanceToSurface = 0.0;
    };

    std::vector<ExportCell> exportCells;
    exportCells.reserve(space.Cells().size());

    for (const auto& kv : space.Cells())
    {
        const VoxelCell& cell = kv.second;

        if (!ShouldExportState(cell.state, statesToExport))
        {
            continue;
        }

        ExportCell exportCell;
        exportCell.index = cell.index;
        exportCell.state = cell.state;
        exportCell.distanceToSurface = cell.distanceToSurface;

        exportCells.push_back(exportCell);
    }

    std::ofstream ofs(filePath.c_str(), std::ios::out);
    if (!ofs.is_open())
    {
        return false;
    }

    const std::size_t cellCount = exportCells.size();
    const std::size_t pointCount = cellCount * 8;

    ofs << "# vtk DataFile Version 3.0\n";
    ofs << "VoxelSpace exported from OCCT voxel builder\n";
    ofs << "ASCII\n";
    ofs << "DATASET UNSTRUCTURED_GRID\n";

    ofs << std::setprecision(15);

    ofs << "POINTS " << pointCount << " double\n";

    for (const ExportCell& cell : exportCells)
    {
        const gp_Pnt minP = space.IndexToMinCorner(cell.index);
        const gp_Pnt maxP = space.IndexToMaxCorner(cell.index);

        const double x0 = minP.X();
        const double y0 = minP.Y();
        const double z0 = minP.Z();

        const double x1 = maxP.X();
        const double y1 = maxP.Y();
        const double z1 = maxP.Z();

        // VTK_HEXAHEDRON 点序：
        // bottom: 0,1,2,3
        // top   : 4,5,6,7

        ofs << x0 << " " << y0 << " " << z0 << "\n"; // 0
        ofs << x1 << " " << y0 << " " << z0 << "\n"; // 1
        ofs << x1 << " " << y1 << " " << z0 << "\n"; // 2
        ofs << x0 << " " << y1 << " " << z0 << "\n"; // 3

        ofs << x0 << " " << y0 << " " << z1 << "\n"; // 4
        ofs << x1 << " " << y0 << " " << z1 << "\n"; // 5
        ofs << x1 << " " << y1 << " " << z1 << "\n"; // 6
        ofs << x0 << " " << y1 << " " << z1 << "\n"; // 7
    }

    ofs << "CELLS " << cellCount << " " << cellCount * 9 << "\n";

    for (std::size_t i = 0; i < cellCount; ++i)
    {
        const std::size_t base = i * 8;

        ofs << "8 "
            << base + 0 << " "
            << base + 1 << " "
            << base + 2 << " "
            << base + 3 << " "
            << base + 4 << " "
            << base + 5 << " "
            << base + 6 << " "
            << base + 7 << "\n";
    }

    ofs << "CELL_TYPES " << cellCount << "\n";

    for (std::size_t i = 0; i < cellCount; ++i)
    {
        // VTK_HEXAHEDRON = 12
        ofs << "12\n";
    }

    ofs << "CELL_DATA " << cellCount << "\n";

    ofs << "SCALARS voxel_state int 1\n";
    ofs << "LOOKUP_TABLE default\n";

    for (const ExportCell& cell : exportCells)
    {
        ofs << StateToInt(cell.state) << "\n";
    }

    ofs << "SCALARS distance_to_surface double 1\n";
    ofs << "LOOKUP_TABLE default\n";

    for (const ExportCell& cell : exportCells)
    {
        ofs << cell.distanceToSurface << "\n";
    }

    ofs << "SCALARS voxel_x int 1\n";
    ofs << "LOOKUP_TABLE default\n";

    for (const ExportCell& cell : exportCells)
    {
        ofs << cell.index.x << "\n";
    }

    ofs << "SCALARS voxel_y int 1\n";
    ofs << "LOOKUP_TABLE default\n";

    for (const ExportCell& cell : exportCells)
    {
        ofs << cell.index.y << "\n";
    }

    ofs << "SCALARS voxel_z int 1\n";
    ofs << "LOOKUP_TABLE default\n";

    for (const ExportCell& cell : exportCells)
    {
        ofs << cell.index.z << "\n";
    }

    ofs.close();

    return true;
}