set(VOXEL_PATH_PLANNER_HEADERS
    VoxelChunkCache.h
    VoxelAStar.h
    VoxelMeshBuilder.h
    VoxelPathOptimizer.h
    VoxelPathPlanner.h
    VoxelSpace.h
    VoxelWalkability.h
)

set(VOXEL_PATH_PLANNER_SOURCES
    VoxelChunkCache.cpp
    VoxelAStar.cpp
    VoxelMeshBuilder.cpp
    VoxelPathOptimizer.cpp
    VoxelPathPlanner.cpp
)

add_library(VoxelPathPlanner
    ${VOXEL_PATH_PLANNER_HEADERS}
    ${VOXEL_PATH_PLANNER_SOURCES}
)

target_include_directories(VoxelPathPlanner
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/../../IO/VtkExport
)

target_link_libraries(VoxelPathPlanner
    PUBLIC
        ${OCCT_LIBS}
        ${FREETYPE_LIBS}
    PRIVATE
        MovementPathVtkExport
)
