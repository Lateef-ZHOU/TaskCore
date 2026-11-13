include_guard(GLOBAL)
include(FetchContent)
if(NOT TARGET Lidar::Algorithm)
    FetchContent_Declare(lidar_algorithm
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/Algorithm.git
        GIT_TAG 5507bdf3299cb91a069466516b8c8eea1c617079)
    FetchContent_MakeAvailable(lidar_algorithm)
endif()
