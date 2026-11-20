#include "DAQFamily/DAQFrames.h"
#include "DAQFamily/RadialWind.h"

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>

using TestRange = LinearMapping<30.0, 15.0, 2>;
using TestFrequency = LinearMapping<10.0, 2.0, 17>;
using TestWind = LinearMapping<0.0, 1.0, 17>;
using TestSpectrum = Grid2D<TestRange, TestFrequency>;

// 帧类型的基本布局和行为。
static_assert(sizeof(TestSpectrum) == sizeof(std::array<double, 34>));
static_assert(sizeof(Array1D<TestRange>) == sizeof(std::array<double, 2>));
static_assert(sizeof(Array1D<TestRange>) == sizeof(std::array<double, 2>));
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
static_assert(std::same_as<decltype(std::declval<Array1D<TestRange>&>()[0]), double&>);
static_assert(!std::is_convertible_v<Array1D<TestRange>*, std::array<double, 2>*>);
static_assert(StationaryObject<TestSpectrum> && StationaryObject<Array1D<TestRange>>
    && StationaryObject<Array1D<TestRange>>);
// 径向风与 CNR 合并为同一容器，不再靠类型区分。

// 映射的静态成员。
static_assert(TestFrequency::count == 17);
static_assert(TestWind::count == TestFrequency::count);
static_assert(TestRange::count == 2);

// 非均匀轴允许递减：映射本身只要求严格单调。
constexpr std::array<double, 3> descendingEdges() { return {75.0, 15.0, 0.0}; }
using DescendingRange = NonLinearMapping<descendingEdges>;
static_assert(DescendingRange::count == 2 && DescendingRange::stop == 0.0
    && sizeof(Grid2D<DescendingRange, TestFrequency>) == sizeof(std::array<double, 34>));

// 非单调表映射（风速等值映射）：不做单调校验，逐点取值。
constexpr std::array<double, 3> windTable() { return {5.0, 1.0, 3.0}; }
using WindTable = TableMapping<windTable>;
static_assert(WindTable::count == 2 && WindTable::at(0) == 5.0 && WindTable::at(2) == 3.0);

// 不同的标定即不同类型。
using ShiftedFrequency = LinearMapping<10.0, 1.0, 17>;
static_assert(!std::same_as<TestSpectrum, Grid2D<TestRange, ShiftedFrequency>>);

// 径向风计算入口。
template<class T>
concept CanCalculate = requires(const T &value, TestWind wind) { RadialWindCalculation::Calc(value, wind); };
static_assert(CanCalculate<TestSpectrum>);

// 径向风帧的基本读写。
int main()
{
    Array1D<TestRange> wind;
    if (wind[0] != 0 || wind[1] != 0) return 1;
    wind[0] = -3.5; wind[1] = 8.0;
    if (wind[0] != -3.5 || wind[1] != 8.0) return 2;
    return 0;
}
