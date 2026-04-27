#pragma once

#include "VoxelSpace.h"

#include <string>
#include <set>

class VoxelVtkExporter
{
public:
    // 导出指定状态的体素为六面体 VTK。
    // 默认导出 Occupied + ClearanceBand。
    static bool ExportVoxelSpaceToVtk(
        const VoxelSpace& space,
        const std::string& filePath,
        const std::set<VoxelState>& statesToExport =
        {
            VoxelState::Occupied,
            VoxelState::ClearanceBand
        });

private:
    static bool ShouldExportState(
        VoxelState state,
        const std::set<VoxelState>& statesToExport);

    static int StateToInt(VoxelState state);
};