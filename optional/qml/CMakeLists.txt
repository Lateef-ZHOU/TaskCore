find_package(Qt6 6.11.2 REQUIRED COMPONENTS Quick Qml GuiPrivate ShaderTools)
set(CMAKE_AUTOMOC ON)
include(FetchContent)
if(NOT TARGET Lidar::UICore)
    FetchContent_Declare(lidar_uicore
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/UICore.git
        GIT_TAG bc9f330c8442169c4dd0fc10b5b7fbb5e7eeecb8)
    FetchContent_MakeAvailable(lidar_uicore)
endif()
if(NOT TARGET Lidar::ChartCore)
    FetchContent_Declare(lidar_chartcore
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/ChartCore.git
        GIT_TAG 223182769dcb51b6c591892edbe1655cf12219cd)
    FetchContent_MakeAvailable(lidar_chartcore)
endif()
set(import_paths)
foreach(module Lidar::UICore Lidar::ChartCore)
    get_target_property(output ${module} QT_QML_MODULE_OUTPUT_DIRECTORY)
    get_filename_component(import_root "${output}/../.." ABSOLUTE)
    list(APPEND import_paths "${import_root}")
endforeach()
file(GLOB views CONFIGURE_DEPENDS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.qml")
qt_add_qml_module(LidarTaskUI STATIC URI Lidar.TaskUI VERSION 1.0
    CLASS_NAME LidarTaskUIPlugin RESOURCE_PREFIX /qt/qml
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Lidar/TaskUI"
    DEPENDENCIES Lidar.UI Lidar.Charts
    IMPORT_PATH ${import_paths}
    QML_FILES ${views}
    SOURCES TaskQmlTypes.h BeamCompassItem.h)
set_target_properties(LidarTaskUI PROPERTIES AUTOMOC ON)
add_library(Lidar::TaskUI ALIAS LidarTaskUI)
target_include_directories(LidarTaskUI PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(LidarTaskUI PUBLIC Lidar::TaskSupport Lidar::ChartCore Lidar::UICore)
target_compile_definitions(LidarTaskUI PUBLIC QT_NO_KEYWORDS)
if(MSVC)
    target_compile_options(LidarTaskUI PRIVATE /W4)
    # Qt generated QML code can trigger C4702 inside Qt headers during optimization.
    get_target_property(module_sources LidarTaskUI SOURCES)
    foreach(source IN LISTS module_sources)
        if(source MATCHES "/[.]rcc/qmlcache/")
            set_source_files_properties("${source}" PROPERTIES COMPILE_OPTIONS "/wd4702")
        endif()
    endforeach()
endif()
