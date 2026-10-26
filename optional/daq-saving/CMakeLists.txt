add_library(LidarDAQSaving INTERFACE)
add_library(Lidar::DAQSaving ALIAS LidarDAQSaving)
target_include_directories(LidarDAQSaving INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(LidarDAQSaving INTERFACE Lidar::Algorithm Lidar::DeviceCore)
