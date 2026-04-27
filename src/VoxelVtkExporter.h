#pragma once

#include "VoxelSpace.h"

#include <string>
#include <set>
#include <vector>

class VoxelVtkExporter
{
public:
    // 导出指定状态的体素为六面体 VTK。
    static bool ExportVoxelSpaceToVtk(
        const VoxelSpace& space,
        const std::string& filePath,
        const std::set<VoxelState>& statesToExport =
        {
            VoxelState::Occupied,
            VoxelState::ClearanceBand
        });

    // 将路径关键体素标记到 VoxelSpace 中，用于后续导出体素 Path。
    static void MarkPathToVoxelSpace(
        VoxelSpace& space,
        const std::vector<VoxelIndex>& path);

    // 导出路径折线为 VTK PolyData。
    // 这个导出的是连续折线，不是六面体体素。
    static bool ExportPathPolylineToVtk(
        const VoxelSpace& space,
        const std::vector<VoxelIndex>& path,
        const std::string& filePath);

    // 直接导出 Vec 路径折线。
    static bool ExportPathPolylineToVtk(
        const std::vector<Vec>& points,
        const std::string& filePath);

private:
    static bool ShouldExportState(
        VoxelState state,
        const std::set<VoxelState>& statesToExport);

    static int StateToInt(VoxelState state);
};