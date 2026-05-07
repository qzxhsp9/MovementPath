set(_ppw_qt_user_defaults "${CMAKE_CURRENT_LIST_DIR}/QtWorkbenchDefaults.cmake")
if(EXISTS "${_ppw_qt_user_defaults}")
    include("${_ppw_qt_user_defaults}")
endif()
if(NOT MOVEMENTPATH_QT_WORKBENCH_DEFAULT_KIT)
    set(MOVEMENTPATH_QT_WORKBENCH_DEFAULT_KIT "C:/Qt/6.11.0/msvc2022_64")
endif()

set(PATH_PLANNING_WORKBENCH_QT_ROOT
    ""
    CACHE PATH
    "Optional Qt kit root for PathPlanningWorkbench (MSVC kit with lib/cmake/Qt6). See QtWorkbenchDefaults.cmake.example.")

if(NOT PATH_PLANNING_WORKBENCH_QT_ROOT AND
    EXISTS "${MOVEMENTPATH_QT_WORKBENCH_DEFAULT_KIT}/lib/cmake/Qt6/Qt6Config.cmake")
    set(PATH_PLANNING_WORKBENCH_QT_ROOT
        "${MOVEMENTPATH_QT_WORKBENCH_DEFAULT_KIT}"
        CACHE PATH
        "Optional Qt kit root for PathPlanningWorkbench (MSVC kit with lib/cmake/Qt6). See QtWorkbenchDefaults.cmake.example."
        FORCE)
endif()
if(PATH_PLANNING_WORKBENCH_QT_ROOT)
    list(PREPEND CMAKE_PREFIX_PATH "${PATH_PLANNING_WORKBENCH_QT_ROOT}")
endif()

find_package(Qt6 COMPONENTS Widgets Concurrent QUIET)

if(Qt6_FOUND)
    set(PATH_PLANNING_WORKBENCH_QT_TARGET Qt6::Widgets)
    set(PATH_PLANNING_WORKBENCH_QT_CONCURRENT_TARGET Qt6::Concurrent)
else()
    find_package(Qt5 COMPONENTS Widgets Concurrent QUIET)
    if(Qt5_FOUND)
        set(PATH_PLANNING_WORKBENCH_QT_TARGET Qt5::Widgets)
        set(PATH_PLANNING_WORKBENCH_QT_CONCURRENT_TARGET Qt5::Concurrent)
    endif()
endif()

if(NOT PATH_PLANNING_WORKBENCH_QT_TARGET OR
    NOT PATH_PLANNING_WORKBENCH_QT_CONCURRENT_TARGET)
    message(STATUS
        "PathPlanningWorkbench skipped: Qt Widgets/Concurrent was not found. "
        "Set PATH_PLANNING_WORKBENCH_QT_ROOT or CMAKE_PREFIX_PATH, or copy "
        "QtWorkbenchDefaults.cmake.example to QtWorkbenchDefaults.cmake and set MOVEMENTPATH_QT_WORKBENCH_DEFAULT_KIT.")
    return()
endif()

message(STATUS
    "PathPlanningWorkbench enabled with Qt root: "
    "${PATH_PLANNING_WORKBENCH_QT_ROOT}")

set(CMAKE_AUTOMOC ON)

add_executable(PathPlanningWorkbench
    main.cpp
    ModelLoader.h
    ModelLoader.cpp
    OcctViewWidget.h
    OcctViewWidget.cpp
    WorkbenchMainWindow.h
    WorkbenchMainWindow.cpp
)

target_link_libraries(PathPlanningWorkbench
    PRIVATE
        VoxelPathPlanner
        GeometryQueryPathPlanner
        ${PATH_PLANNING_WORKBENCH_QT_TARGET}
        ${PATH_PLANNING_WORKBENCH_QT_CONCURRENT_TARGET}
)

set(_ppw_repo_data_dir "${CMAKE_SOURCE_DIR}/data")
file(TO_CMAKE_PATH "${_ppw_repo_data_dir}" _ppw_repo_data_dir)
target_compile_definitions(PathPlanningWorkbench PRIVATE
    "MOVEMENTPATH_WORKBENCH_DATA_DIR=\"${_ppw_repo_data_dir}\"")

if(WIN32 AND PATH_PLANNING_WORKBENCH_QT_ROOT)
    find_program(PATH_PLANNING_WORKBENCH_WINDEPLOYQT
        NAMES windeployqt
        HINTS "${PATH_PLANNING_WORKBENCH_QT_ROOT}/bin"
        NO_DEFAULT_PATH)

    if(PATH_PLANNING_WORKBENCH_WINDEPLOYQT)
        add_custom_command(
            TARGET PathPlanningWorkbench
            POST_BUILD
            COMMAND "${PATH_PLANNING_WORKBENCH_WINDEPLOYQT}"
                --no-translations
                --no-compiler-runtime
                "$<TARGET_FILE:PathPlanningWorkbench>"
            VERBATIM)
    else()
        add_custom_command(
            TARGET PathPlanningWorkbench
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${PATH_PLANNING_WORKBENCH_QT_ROOT}/bin"
                "$<TARGET_FILE_DIR:PathPlanningWorkbench>"
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:PathPlanningWorkbench>/platforms"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${PATH_PLANNING_WORKBENCH_QT_ROOT}/plugins/platforms"
                "$<TARGET_FILE_DIR:PathPlanningWorkbench>/platforms"
            VERBATIM)
    endif()
endif()

if(WIN32 AND OCCT_DLLS)
    add_custom_command(
        TARGET PathPlanningWorkbench
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${OCCT_DLLS}
            "$<TARGET_FILE_DIR:PathPlanningWorkbench>"
        VERBATIM)
endif()

if(WIN32 AND FREETYPE_DLLS)
    add_custom_command(
        TARGET PathPlanningWorkbench
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${FREETYPE_DLLS}
            "$<TARGET_FILE_DIR:PathPlanningWorkbench>"
        VERBATIM)
endif()
