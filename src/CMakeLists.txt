project (MovementPath)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/occt_setup_install.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/occt_3rdpart_setup_install.cmake)

add_subdirectory(
    ${CMAKE_CURRENT_LIST_DIR}/MovementPathCore
    ${CMAKE_CURRENT_BINARY_DIR}/MovementPathCore)

add_executable (${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE MovementPathCore)

include(CTest)

if(BUILD_TESTING)
    add_executable(MovementPathSmokeTests
        ${CMAKE_SOURCE_DIR}/tests/VoxelPathPlannerSmokeTests.cpp)

    target_link_libraries(MovementPathSmokeTests PRIVATE MovementPathCore)

    add_test(
        NAME MovementPathSmokeTests
        COMMAND MovementPathSmokeTests)
endif()

file(COPY ${OCCT_DLLS} DESTINATION "${CMAKE_BINARY_DIR}/")
file(COPY ${OCCT_PDBS} DESTINATION "${CMAKE_BINARY_DIR}/")
file(COPY ${FREETYPE_DLLS} DESTINATION "${CMAKE_BINARY_DIR}/")

# TODO: 如有需要，请添加测试并安装目标。
