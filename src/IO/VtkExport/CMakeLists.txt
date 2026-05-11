set(MOVEMENT_PATH_VTK_EXPORT_HEADERS
    MeshVtkExporter.h
    VoxelVtkExporter.h
)

set(MOVEMENT_PATH_VTK_EXPORT_SOURCES
    MeshVtkExporter.cpp
    VoxelVtkExporter.cpp
)

add_library(MovementPathVtkExport
    ${MOVEMENT_PATH_VTK_EXPORT_HEADERS}
    ${MOVEMENT_PATH_VTK_EXPORT_SOURCES}
)

target_include_directories(MovementPathVtkExport
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../../Algorithms/VoxelPathPlanner
)
