#include "WindField.h"
#include <optional>
#include <chrono>
#include <type_traits>
#include <QCoreApplication>
#include "DeviceCoro.h"
#include "DAQDevice.h"
#include "GMBDevice.h"
#include "WindProfile.h"
#include "LidarForward.h"
static_assert(std::is_same_v<RxTime, std::chrono::system_clock::time_point>);
static_assert(std::is_same_v<decltype(WindField::Frame::at), const RxTime>);
static_assert(!std::is_copy_constructible_v<DeviceLock>);
int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    const auto now = std::chrono::system_clock::now();
    WindField::Frame frame{now, 100.0, 25.0, std::vector<std::optional<LinAlg::Vec3>>{LinAlg::Vec3{1.0, 2.0, 3.0}}};
    if (!frame.hasValidGrid() || frame.winds.positionAt(0) != 100.0 || frame.at != now) return 1;
    const WorldModel world(35.0, 60.0);
    const WorldPoseHistory history{{now, world}};
    if (history.front().first != now) return 2;
    return 0;
}
