include_guard(GLOBAL)
include(FetchContent)
if(NOT TARGET Lidar::HandleCore)
    FetchContent_Declare(lidar_handle_core
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/HandleCore.git
        GIT_TAG 379401c85f0da11adde9a2f6fde8fe767fbae707)
    FetchContent_MakeAvailable(lidar_handle_core)
endif()
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
        GIT_TAG 5c63ec5c501d83a15daa65a41dca438ee4ff330f)
    FetchContent_MakeAvailable(lidar_devicecore)
endif()
