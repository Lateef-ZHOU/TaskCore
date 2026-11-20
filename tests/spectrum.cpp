#include "DAQFamily/DAQChannelSpec.h"
#include "DAQFamily/RadialWind.h"

#include <array>
#include <cmath>
#include <numbers>
#include <limits>
#include <span>

inline constexpr DAQParams<UniformRangeMapping<30.0, 60.0, 2>> testParams{
    .wavelengthM = 1.55e-6, .zeroWindMHz = 18,
    .frequencyStartMHz = 10, .frequencyStepMHz = 2,
    .frequencyBinCount = 17,
};

// 帧类型的基本布局和行为。
static_assert(sizeof(Spectrum<testParams>) == sizeof(std::array<double, 34>));
static_assert(sizeof(RadialWind<testParams>) == sizeof(std::array<double, 2>));
static_assert(sizeof(CNR<testParams>) == sizeof(std::array<double, 2>));
static_assert(Spectrum<testParams>{}[33] == 0);

template<class T> concept CanBorrowRow = requires(T &&s) {
    { std::forward<T>(s).row(0) } -> std::same_as<std::span<const double>>;
};
static_assert(CanBorrowRow<Spectrum<testParams>&> && CanBorrowRow<const Spectrum<testParams>&>);
static_assert(!CanBorrowRow<Spectrum<testParams>> && !CanBorrowRow<const Spectrum<testParams>>);
static_assert(noexcept(std::declval<const Spectrum<testParams>&>().row(0)));

static_assert([] {
    Spectrum<testParams> spectrum;
    return spectrum.row(0).size() == testParams.frequencyBinCount
        && spectrum.row(1).size() == testParams.frequencyBinCount
        && spectrum.row(2).empty()
        && spectrum.row(std::numeric_limits<std::size_t>::max()).empty();
}());

// 径向风帧。
static_assert(std::same_as<decltype(std::declval<RadialWind<testParams>&>()[0]), double&>);
static_assert(!std::is_convertible_v<RadialWind<testParams>*, std::array<double, 2>*>);
static_assert(StationaryObject<Spectrum<testParams>> && StationaryObject<RadialWind<testParams>>
    && StationaryObject<CNR<testParams>>);
static_assert(!std::same_as<RadialWind<testParams>, CNR<testParams>>);

// params 静态成员。
static_assert(Spectrum<testParams>::params.wavelengthM == testParams.wavelengthM);
static_assert(RadialWind<testParams>::params.rangeGateCount == testParams.rangeGateCount);

// 不同参数产生不同类型。
inline constexpr DAQParams<UniformRangeMapping<30.0, 60.0, 2>> shifted{.wavelengthM = 1.6e-6, .zeroWindMHz = 16,
    .frequencyStartMHz = 10, .frequencyStepMHz = 2, .frequencyBinCount = 17};
static_assert(!std::same_as<Spectrum<testParams>, Spectrum<shifted>>);

// 径向风计算入口。
template<class T>
concept CanCalculate = requires(const T &value) { RadialWindCalculation::Calc(value); };
static_assert(CanCalculate<Spectrum<testParams>>);

// 径向风帧的基本读写。
int main()
{
    RadialWind<testParams> wind;
    if (wind[0] != 0 || wind[1] != 0) return 1;
    wind[0] = -3.5; wind[1] = 8.0;
    if (wind[0] != -3.5 || wind[1] != 8.0) return 2;
    return 0;
}
