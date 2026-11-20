#include "DAQFamily/WindProfile.h"
#include "WallClockHistory.h"
#include <type_traits>
#include <utility>

using ImmutableRange = LinearMapping<30.0, 15.0, 3>;
using ImmutableFrequency = LinearMapping<79.0, 1.0, 2>;
using ImmutableSpectrum = Grid2D<ImmutableRange, ImmutableFrequency>;
using Frame = WindProfileFrame<ImmutableRange>;
static_assert(std::is_nothrow_constructible_v<Frame, std::chrono::system_clock::time_point, double>);
static_assert(ImmutableWallClockFrame<Frame>);
static_assert(!std::is_copy_constructible_v<ImmutableSpectrum> && !std::is_move_constructible_v<ImmutableSpectrum>);
static_assert(!std::is_copy_assignable_v<ImmutableSpectrum>);
static_assert(!std::is_copy_constructible_v<Array1D<ImmutableRange>> && !std::is_move_constructible_v<Array1D<ImmutableRange>>);
static_assert(!std::is_copy_assignable_v<Array1D<ImmutableRange>>);
static_assert(!std::is_copy_constructible_v<Frame> && !std::is_move_constructible_v<Frame>);
static_assert(!std::is_copy_assignable_v<Frame> && !std::is_move_assignable_v<Frame>);
int main() { return 0; }
