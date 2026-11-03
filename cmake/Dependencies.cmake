include_guard(GLOBAL)
include(FetchContent)
if(NOT TARGET Lidar::HandleCore)
    FetchContent_Declare(lidar_handle_core
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/HandleCore.git
        GIT_TAG 5afbfc6b2ee5b9c8a4e1138e8c2ab7a2d02b0e56)
    FetchContent_MakeAvailable(lidar_handle_core)
endif()
# 消费方已提供目标时直接复用；否则拉取经过验证的固定提交，不启用 Handle 可选模块。
if(NOT TARGET Lidar::Algorithm)
    FetchContent_Declare(lidar_algorithm
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/Algorithm.git
        GIT_TAG 3161796b7c5fa9231f0c16f094e281f3535336b4)
    FetchContent_MakeAvailable(lidar_algorithm)
endif()
if(NOT TARGET Lidar::DeviceCore)
    FetchContent_Declare(lidar_devicecore
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/DeviceCore.git
        GIT_TAG 2b59a967a847c9b13844ec8ac5d298ad858b95b5)
    FetchContent_MakeAvailable(lidar_devicecore)
endif()
