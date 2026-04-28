set(MOVEMENT_PATH_CORE_HEADERS
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

set(MOVEMENT_PATH_CORE_SOURCES
    MeshVtkExporter.cpp
    VoxelChunkCache.cpp
    VoxelAStar.cpp
    VoxelMeshBuilder.cpp
    VoxelPathOptimizer.cpp
    VoxelPathPlanner.cpp
    VoxelVtkExporter.cpp
)

add_library(MovementPathCore
    ${MOVEMENT_PATH_CORE_HEADERS}
    ${MOVEMENT_PATH_CORE_SOURCES}
)

target_include_directories(MovementPathCore
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(MovementPathCore
    PUBLIC
        ${OCCT_LIBS}
        ${FREETYPE_LIBS}
)
