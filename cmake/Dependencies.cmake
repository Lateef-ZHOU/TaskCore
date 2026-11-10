include_guard(GLOBAL)
include(FetchContent)
if(NOT TARGET Lidar::HandleCore)
    FetchContent_Declare(lidar_handle_core
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/HandleCore.git
        GIT_TAG 4b2479bcd1911298f28ff12f0e78f9e4774acb8c)
    FetchContent_MakeAvailable(lidar_handle_core)
endif()
# 消费方已提供目标时直接复用；否则拉取经过验证的固定提交，不启用 Handle 可选模块。
if(NOT TARGET Lidar::Algorithm)
    FetchContent_Declare(lidar_algorithm
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/Algorithm.git
        GIT_TAG 733d0e72058783247ccf8c5a194afaa1765b9de5)
    FetchContent_MakeAvailable(lidar_algorithm)
endif()
# 采集卡族的四束环用 Boost.CircularBuffer；先复用消费方目标或已装包，再取固定哈希的官方归档。
if(NOT TARGET Boost::circular_buffer)
    find_package(Boost 1.88 QUIET CONFIG COMPONENTS circular_buffer)
    if(NOT TARGET Boost::circular_buffer)
        set(BOOST_INCLUDE_LIBRARIES ${BOOST_INCLUDE_LIBRARIES} circular_buffer)
        FetchContent_Declare(boost
            URL https://github.com/boostorg/boost/releases/download/boost-1.88.0/boost-1.88.0-cmake.tar.xz
            URL_HASH SHA256=f48b48390380cfb94a629872346e3a81370dc498896f16019ade727ab72eb1ec
            EXCLUDE_FROM_ALL)
        FetchContent_MakeAvailable(boost)
    endif()
endif()
if(NOT TARGET Lidar::DeviceCore)
    FetchContent_Declare(lidar_devicecore
        GIT_REPOSITORY https://github.com/Lateef-ZHOU/DeviceCore.git
        GIT_TAG bdcc0b576bcfe21a569cc2fcbebf1c8f3173fb88)
    FetchContent_MakeAvailable(lidar_devicecore)
endif()
