set(VOXEL_PATH_PLANNER_HEADERS
    MeshVtkExporter.h
    VoxelChunkCache.h
    VoxelAStar.h
    VoxelMeshBuilder.h
    VoxelPathOptimizer.h
    VoxelPathPlanner.h
    VoxelSpace.h
    VoxelVtkExporter.h
    VoxelWalkability.h
)

set(VOXEL_PATH_PLANNER_SOURCES
    MeshVtkExporter.cpp
    VoxelChunkCache.cpp
    VoxelAStar.cpp
    VoxelMeshBuilder.cpp
    VoxelPathOptimizer.cpp
    VoxelPathPlanner.cpp
    VoxelVtkExporter.cpp
)

add_library(VoxelPathPlanner
    ${VOXEL_PATH_PLANNER_HEADERS}
    ${VOXEL_PATH_PLANNER_SOURCES}
)

target_include_directories(VoxelPathPlanner
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(VoxelPathPlanner
    PUBLIC
        ${OCCT_LIBS}
        ${FREETYPE_LIBS}
)
