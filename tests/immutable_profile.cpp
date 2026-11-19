#include "DAQFamily/WindProfile.h"
#include "WallClockHistory.h"
#include <type_traits>
#include <utility>

inline constexpr DAQParams immutableParams{
    .wavelengthM = 1.55e-6, .zeroWindMHz = 80,
    .rangeStartM = 30, .rangeStepM = 15,
    .frequencyStartMHz = 79, .frequencyStepMHz = 1,
    .rangeGateCount = 3, .frequencyBinCount = 2,
};
using Frame = VerticalWindProfileUnit<immutableParams>;
static_assert(std::is_nothrow_constructible_v<Frame, std::chrono::system_clock::time_point, double>);
static_assert(ImmutableWallClockFrame<Frame>);
static_assert(!std::is_copy_constructible_v<Spectrum<immutableParams>> && !std::is_move_constructible_v<Spectrum<immutableParams>>);
static_assert(!std::is_copy_assignable_v<Spectrum<immutableParams>>);
static_assert(!std::is_copy_constructible_v<RadialWind<immutableParams>> && !std::is_move_constructible_v<RadialWind<immutableParams>>);
static_assert(!std::is_copy_assignable_v<RadialWind<immutableParams>>);
static_assert(!std::is_copy_constructible_v<Frame> && !std::is_move_constructible_v<Frame>);
static_assert(!std::is_copy_assignable_v<Frame> && !std::is_move_assignable_v<Frame>);
int main() { return 0; }
