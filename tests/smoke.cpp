#include <optional>
#include <chrono>
#include <type_traits>
#include <QCoreApplication>
#include "DeviceCoro.h"
#include "DAQDevice.h"
#include "GMBDevice.h"
#include "WindProfile.h"
#include "LidarForward.h"
struct ProbeSpec {
    static constexpr const char *name = "test";
    static constexpr int rangeGateCount = 1, frequencyBinCount = 1;
    static constexpr double rangeStartM = 100, rangeStepM = 25, frequencyStartMHz = 0,
        frequencyStepMHz = 1, wavelengthM = 1.55e-6, zeroWindMHz = 80;
};
using Frame = VerticalWindProfileUnit<ProbeSpec>;
static_assert(std::is_same_v<RxTime, std::chrono::system_clock::time_point>);
static_assert(std::is_same_v<decltype(Frame::at), const RxTime>);
static_assert(!std::is_copy_constructible_v<DeviceLock>);
int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    const auto now = std::chrono::system_clock::now();
    Frame frame{now, 90.0};
    frame[0] = LinAlg::Vec3{1,2,3};
    if (!WindProfile::hasValidGrid(frame) || frame.heightStartM() != 100.0 || frame.at != now) return 1;
    const WorldModel world(35.0, 60.0);
    const WorldPoseHistory history{{now, world}};
    if (history.front().first != now) return 2;
    return 0;
}
