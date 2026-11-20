#include "DAQFamily/DAQChannelSpec.h"
#include "DAQFamily/RadialWind.h"

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>

using TestRange = UniformMapping<30.0, 15.0, 2>;
using TestFrequency = DopplerMapping<UniformMapping<10.0, 2.0, 17>, 1.55e-6, 18.0>;
using TestSpectrum = Spectrum<TestRange, TestFrequency>;

// 帧类型的基本布局和行为。
static_assert(sizeof(TestSpectrum) == sizeof(std::array<double, 34>));
static_assert(sizeof(RadialWind<TestRange>) == sizeof(std::array<double, 2>));
static_assert(sizeof(CNR<TestRange>) == sizeof(std::array<double, 2>));
static_assert(TestSpectrum{}[33] == 0);

template<class T> concept CanBorrowRow = requires(T &&s) {
    { std::forward<T>(s).row(0) } -> std::same_as<std::span<const double>>;
};
static_assert(CanBorrowRow<TestSpectrum&> && CanBorrowRow<const TestSpectrum&>);
static_assert(!CanBorrowRow<TestSpectrum> && !CanBorrowRow<const TestSpectrum>);
static_assert(noexcept(std::declval<const TestSpectrum&>().row(0)));

static_assert([] {
    TestSpectrum spectrum;
    return spectrum.row(0).size() == static_cast<std::size_t>(TestFrequency::count)
        && spectrum.row(1).size() == static_cast<std::size_t>(TestFrequency::count)
        && spectrum.row(2).empty()
        && spectrum.row(std::numeric_limits<std::size_t>::max()).empty();
}());

// 径向风帧。
static_assert(std::same_as<decltype(std::declval<RadialWind<TestRange>&>()[0]), double&>);
static_assert(!std::is_convertible_v<RadialWind<TestRange>*, std::array<double, 2>*>);
static_assert(StationaryObject<TestSpectrum> && StationaryObject<RadialWind<TestRange>>
    && StationaryObject<CNR<TestRange>>);
static_assert(!std::same_as<RadialWind<TestRange>, CNR<TestRange>>);

// 映射的静态成员。
static_assert(TestFrequency::wavelengthM == 1.55e-6);
static_assert(TestFrequency::zeroWindMHz == 18.0);
static_assert(TestRange::count == 2);

// 不同的标定即不同类型。
using ShiftedFrequency = DopplerMapping<UniformMapping<10.0, 2.0, 17>, 1.6e-6, 16.0>;
static_assert(!std::same_as<TestSpectrum, Spectrum<TestRange, ShiftedFrequency>>);

// 径向风计算入口。
template<class T>
concept CanCalculate = requires(const T &value) { RadialWindCalculation::Calc(value); };
static_assert(CanCalculate<TestSpectrum>);

// 径向风帧的基本读写。
int main()
{
    RadialWind<TestRange> wind;
    if (wind[0] != 0 || wind[1] != 0) return 1;
    wind[0] = -3.5; wind[1] = 8.0;
    if (wind[0] != -3.5 || wind[1] != 8.0) return 2;
    return 0;
}
