include_guard(GLOBAL)
include(FetchContent)
# 消费方已提供目标时直接复用；否则拉取经过验证的固定提交，不启用 Handle 可选模块。
if(NOT TARGET Lidar::Algorithm)
    FetchContent_Declare(lidar_algorithm
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/Algorithm.git
        GIT_TAG 00244abaf8fdabb6ad6bf2d2b875af4e60522a4f)
    FetchContent_MakeAvailable(lidar_algorithm)
endif()
if(NOT TARGET Lidar::DeviceCore)
    FetchContent_Declare(lidar_devicecore
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/DeviceCore.git
        GIT_TAG eb3876e9b10b69d3161b34d6c26d17631d77545d)
    FetchContent_MakeAvailable(lidar_devicecore)
endif()
