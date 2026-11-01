include_guard(GLOBAL)
include(FetchContent)
if(NOT TARGET Lidar::HandleCore)
    FetchContent_Declare(lidar_handle_core
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/HandleCore.git
        GIT_TAG 113523f3a40ad2dc7c417d01326688b5bd79a51b)
    FetchContent_MakeAvailable(lidar_handle_core)
endif()
# 消费方已提供目标时直接复用；否则拉取经过验证的固定提交，不启用 Handle 可选模块。
if(NOT TARGET Lidar::Algorithm)
    FetchContent_Declare(lidar_algorithm
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/Algorithm.git
        GIT_TAG 224c784117b775177a52d69fe0d61858b39ee867)
    FetchContent_MakeAvailable(lidar_algorithm)
endif()
if(NOT TARGET Lidar::DeviceCore)
    FetchContent_Declare(lidar_devicecore
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/DeviceCore.git
        GIT_TAG 9f2ee2799cd24a809a5e16031418b24374d1961e)
    FetchContent_MakeAvailable(lidar_devicecore)
endif()
