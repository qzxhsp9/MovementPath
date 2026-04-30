set(GEOMETRY_QUERY_PATH_PLANNER_HEADERS
    GeometryPrimitives.h
    GeometryQueryPathPlanner.h
)

set(GEOMETRY_QUERY_PATH_PLANNER_SOURCES
    GeometryQueryPathPlanner.cpp
)

add_library(GeometryQueryPathPlanner
    ${GEOMETRY_QUERY_PATH_PLANNER_HEADERS}
    ${GEOMETRY_QUERY_PATH_PLANNER_SOURCES}
)

target_include_directories(GeometryQueryPathPlanner
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
)
