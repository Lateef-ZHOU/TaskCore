include_guard(GLOBAL)
include(FetchContent)
if(NOT TARGET Lidar::Algorithm)
    FetchContent_Declare(lidar_algorithm
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/Algorithm.git
        GIT_TAG 49408fc195df20066fb84e251ef1f5b6ae22944d)
    FetchContent_MakeAvailable(lidar_algorithm)
endif()
