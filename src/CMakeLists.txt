project (MovementPath)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/occt_setup_install.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/occt_3rdpart_setup_install.cmake)

add_subdirectory(
    ${CMAKE_CURRENT_LIST_DIR}/VoxelPathPlanner
    ${CMAKE_CURRENT_BINARY_DIR}/VoxelPathPlanner)

add_subdirectory(
    ${CMAKE_CURRENT_LIST_DIR}/GeometryQueryPathPlanner
    ${CMAKE_CURRENT_BINARY_DIR}/GeometryQueryPathPlanner)

add_executable (${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE VoxelPathPlanner)

add_executable(
    MovementPathBenchmark
    ${CMAKE_SOURCE_DIR}/benchmarks/VoxelPlannerBenchmark.cpp)

target_link_libraries(MovementPathBenchmark PRIVATE VoxelPathPlanner)

include(CTest)

if(BUILD_TESTING)
    function(add_movement_path_test target source)
        add_executable(${target} ${source})
        target_link_libraries(${target} PRIVATE VoxelPathPlanner)
        add_test(NAME ${target} COMMAND ${target})
    endfunction()

    function(add_geometry_query_path_planner_test target source)
        add_executable(${target} ${source})
        target_link_libraries(${target} PRIVATE GeometryQueryPathPlanner)
        add_test(NAME ${target} COMMAND ${target})
    endfunction()

    add_movement_path_test(
        TriangleSpatialHashTests
        ${CMAKE_SOURCE_DIR}/tests/TriangleSpatialHashTests.cpp)
    add_movement_path_test(
        VoxelChunkCacheTests
        ${CMAKE_SOURCE_DIR}/tests/VoxelChunkCacheTests.cpp)
    add_movement_path_test(
        VoxelPathPlannerTests
        ${CMAKE_SOURCE_DIR}/tests/VoxelPathPlannerTests.cpp)
    add_geometry_query_path_planner_test(
        GeometryQueryPathPlannerTests
        ${CMAKE_SOURCE_DIR}/tests/GeometryQueryPathPlannerTests.cpp)
endif()

file(COPY ${OCCT_DLLS} DESTINATION "${CMAKE_BINARY_DIR}/")
file(COPY ${OCCT_PDBS} DESTINATION "${CMAKE_BINARY_DIR}/")
file(COPY ${FREETYPE_DLLS} DESTINATION "${CMAKE_BINARY_DIR}/")

# TODO: 如有需要，请添加测试并安装目标。
