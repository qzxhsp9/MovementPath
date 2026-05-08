set(GEOMETRY_QUERY_PATH_PLANNER_HEADERS
    GeometryPrimitives.h
    GeometryPathSmoother.h
    GeometryQueries.h
    GeometryQueryContext.h
    GeometryQueryPathPlanner.h
    GeometrySearchGraph.h
    TriangleMesh.h
    TriangleAabbTree.h
)

set(GEOMETRY_QUERY_PATH_PLANNER_SOURCES
    GeometryQueries.cpp
    GeometryPathSmoother.cpp
    GeometryQueryContext.cpp
    GeometryQueryPathPlanner.cpp
    TriangleMesh.cpp
    TriangleAabbTree.cpp
)

add_library(GeometryQueryPathPlanner
    ${GEOMETRY_QUERY_PATH_PLANNER_HEADERS}
    ${GEOMETRY_QUERY_PATH_PLANNER_SOURCES}
)

target_include_directories(GeometryQueryPathPlanner
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
)
