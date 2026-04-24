project (MovementPath)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(BEFORE ${CMAKE_CURRENT_LIST_DIR})

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/occt_setup_install.cmake)

file(GLOB_RECURSE SOURCECODE ${CMAKE_CURRENT_LIST_DIR}/*.h
    ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

add_executable (${PROJECT_NAME} ${SOURCECODE})

target_link_libraries(${PROJECT_NAME} PRIVATE 
${OCCT_LIBS}
) # add library

file(COPY ${OCCT_DLLS} DESTINATION "${CMAKE_BINARY_DIR}/")
file(COPY ${OCCT_PDBS} DESTINATION "${CMAKE_BINARY_DIR}/")


# TODO: 如有需要，请添加测试并安装目标。
