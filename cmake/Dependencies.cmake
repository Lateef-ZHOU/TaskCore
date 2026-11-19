include_guard(GLOBAL)
include(FetchContent)
if(NOT TARGET Lidar::HandleCore)
    FetchContent_Declare(lidar_handle_core
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/HandleCore.git
        GIT_TAG 73d5bec896aeba4b445163df279fc15a3157ca4b)
    FetchContent_MakeAvailable(lidar_handle_core)
endif()
# Algorithm 和 Boost 已由先加载的 models 入口准备；本文件只补设备运行层。
if(NOT TARGET Lidar::DeviceCore)
    FetchContent_Declare(lidar_devicecore
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/DeviceCore.git
        GIT_TAG b25247814fd33445ac2463c832cb752fe5627bc3)
    FetchContent_MakeAvailable(lidar_devicecore)
endif()
