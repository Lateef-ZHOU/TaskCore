#include "DAQFamily/RadialWind.h"
#include "DAQFamily/DAQChannelSpec.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <limits>
#include <span>
#include <vector>
#include <utility>

struct ChannelSpec
{
    static constexpr const char *name = "test";
    static constexpr int rangeGateCount = 2, frequencyBinCount = 17;
    static constexpr double rangeStartM = 30, rangeStepM = 15;
    static constexpr double frequencyStartMHz = 10, frequencyStepMHz = 2;
    static constexpr double wavelengthM = 1.55e-6, zeroWindMHz = 18;
};
struct FinalSpec final : ChannelSpec {};
struct NoDefaultSpec : ChannelSpec { NoDefaultSpec() = delete; };
struct PrivateDefaultSpec : ChannelSpec { private: PrivateDefaultSpec() = default; };
static_assert(!DAQChannelSpec<FinalSpec> && !DAQChannelSpec<NoDefaultSpec>
    && !DAQChannelSpec<PrivateDefaultSpec>);
template<class S> concept CanUseFrames = requires {
    typename Spectrum<S>;
    typename RadialWind<S>;
    typename CNR<S>;
    typename VerticalWindProfileUnit<S>;
};
static_assert(CanUseFrames<ChannelSpec>);
static_assert(!CanUseFrames<FinalSpec> && !CanUseFrames<NoDefaultSpec>);
struct WithState : ChannelSpec { int state = 0; };
struct RuntimeWavelength : ChannelSpec { static inline double wavelengthM = 1.55e-6; };
struct RuntimeRangeGateCount : ChannelSpec { static inline int rangeGateCount = 2; };
// const 不等于常量表达式：即使函数可被优化器折叠，非 constexpr 调用也不能作为规格。
double runtimeReal() { return 1.0; }
int runtimeCount() { return 2; }
const char *runtimeName() { return "runtime"; }
struct RuntimeConstName : ChannelSpec { static inline const char *const name = runtimeName(); };
struct RuntimeConstWavelength : ChannelSpec { static inline const double wavelengthM = runtimeReal(); };
struct RuntimeConstZeroWind : ChannelSpec { static inline const double zeroWindMHz = runtimeReal(); };
struct RuntimeConstRangeStart : ChannelSpec { static inline const double rangeStartM = runtimeReal(); };
struct RuntimeConstRangeStep : ChannelSpec { static inline const double rangeStepM = runtimeReal(); };
struct RuntimeConstFrequencyStart : ChannelSpec { static inline const double frequencyStartMHz = runtimeReal(); };
struct RuntimeConstFrequencyStep : ChannelSpec { static inline const double frequencyStepMHz = runtimeReal(); };
struct RuntimeConstRangeCount : ChannelSpec { static inline const int rangeGateCount = runtimeCount(); };
struct RuntimeConstFrequencyCount : ChannelSpec { static inline const int frequencyBinCount = runtimeCount(); };
static_assert(!DAQChannelSpec<RuntimeConstName>);
static_assert(!DAQChannelSpec<RuntimeConstWavelength> && !DAQChannelSpec<RuntimeConstZeroWind>);
static_assert(!DAQChannelSpec<RuntimeConstRangeStart> && !DAQChannelSpec<RuntimeConstRangeStep>);
static_assert(!DAQChannelSpec<RuntimeConstFrequencyStart> && !DAQChannelSpec<RuntimeConstFrequencyStep>);
static_assert(!DAQChannelSpec<RuntimeConstRangeCount> && !DAQChannelSpec<RuntimeConstFrequencyCount>);
struct NoRangeGates : ChannelSpec { static constexpr int rangeGateCount = 0; };
struct NegativeRangeGateCount : ChannelSpec { static constexpr int rangeGateCount = -2; };
struct LargeSize : ChannelSpec { static constexpr int rangeGateCount = 2147483647, frequencyBinCount = 2147483647; };
static_assert(DAQChannelSpec<ChannelSpec>);
static_assert(!DAQChannelSpec<WithState> && !DAQChannelSpec<RuntimeWavelength>);
static_assert(!DAQChannelSpec<RuntimeRangeGateCount> && !DAQChannelSpec<NoRangeGates>);
static_assert(!DAQChannelSpec<NegativeRangeGateCount> && !DAQChannelSpec<LargeSize>);
// 四个轴参数在实例化时拒绝 NaN/无穷；步长另外拒绝零及负数。
inline constexpr std::array invalidAxes{
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN(), 0.0, -0.0, -1.0};
template<int I> struct RangeStart : ChannelSpec { static constexpr double rangeStartM = invalidAxes[I]; };
template<int I> struct RangeStep : ChannelSpec { static constexpr double rangeStepM = invalidAxes[I]; };
template<int I> struct FrequencyStart : ChannelSpec { static constexpr double frequencyStartMHz = invalidAxes[I]; };
template<int I> struct FrequencyStep : ChannelSpec { static constexpr double frequencyStepMHz = invalidAxes[I]; };
static_assert(!DAQChannelSpec<RangeStart<0>> && !DAQChannelSpec<RangeStart<1>> && !DAQChannelSpec<RangeStart<2>>);
static_assert(!DAQChannelSpec<FrequencyStart<0>> && !DAQChannelSpec<FrequencyStart<1>> && !DAQChannelSpec<FrequencyStart<2>>);
static_assert(!DAQChannelSpec<RangeStep<0>> && !DAQChannelSpec<RangeStep<1>> && !DAQChannelSpec<RangeStep<2>>
    && !DAQChannelSpec<RangeStep<3>> && !DAQChannelSpec<RangeStep<4>> && !DAQChannelSpec<RangeStep<5>>);
static_assert(!DAQChannelSpec<FrequencyStep<0>> && !DAQChannelSpec<FrequencyStep<1>> && !DAQChannelSpec<FrequencyStep<2>>
    && !DAQChannelSpec<FrequencyStep<3>> && !DAQChannelSpec<FrequencyStep<4>> && !DAQChannelSpec<FrequencyStep<5>>);
static_assert(DAQChannelSpec<RangeStart<3>> && DAQChannelSpec<RangeStart<5>>);
static_assert(DAQChannelSpec<FrequencyStart<3>> && DAQChannelSpec<FrequencyStart<5>>);
struct FiniteBounds : ChannelSpec
{
    static constexpr double rangeStartM = std::numeric_limits<double>::lowest();
    static constexpr double frequencyStartMHz = std::numeric_limits<double>::max();
    static constexpr double rangeStepM = std::numeric_limits<double>::denorm_min();
    static constexpr double frequencyStepMHz = std::numeric_limits<double>::max();
};
static_assert(DAQChannelSpec<FiniteBounds>);
template<class T> concept HasAxisValidation = requires(const T &s) { s.hasValidAxes(); };
static_assert(!HasAxisValidation<Spectrum<ChannelSpec>>);
static_assert(std::is_base_of_v<std::array<double, 34>, Spectrum<ChannelSpec>>);
static_assert(!std::is_convertible_v<Spectrum<ChannelSpec>*, std::array<double, 34>*>);
static_assert(std::same_as<decltype(std::declval<Spectrum<ChannelSpec>&>()[0]), double&>);
static_assert(std::same_as<decltype(std::declval<const Spectrum<ChannelSpec>&>()[0]), const double&>);
template<class T> concept HasValues = requires(T s) { s.values; };
template<class T> concept HasAxisAliases = requires { T::rows; } || requires { T::columns; }
    || requires { T::rowStart; } || requires { T::rowStep; }
    || requires { T::columnStart; } || requires { T::columnStep; };
static_assert(!HasValues<Spectrum<ChannelSpec>> && !HasAxisAliases<Spectrum<ChannelSpec>>);
static_assert(std::is_const_v<decltype(Spectrum<ChannelSpec>::rangeGateCount)>);
static_assert(std::is_const_v<decltype(Spectrum<ChannelSpec>::wavelengthM)>);
static_assert(sizeof(Spectrum<ChannelSpec>) == sizeof(std::array<double, 34>));
static_assert(Spectrum<ChannelSpec>{}[33] == 0);
template<class T> concept HasCompletion = requires(const T &s) { s.complete(); };
static_assert(!HasCompletion<Spectrum<ChannelSpec>>);
template<class T> concept CanBorrowRow = requires(T &&s) {
    { std::forward<T>(s).row(0) } -> std::same_as<std::span<const double>>;
};
static_assert(CanBorrowRow<Spectrum<ChannelSpec>&> && CanBorrowRow<const Spectrum<ChannelSpec>&>);
static_assert(!CanBorrowRow<Spectrum<ChannelSpec>> && !CanBorrowRow<const Spectrum<ChannelSpec>>);
static_assert([] {
    const Spectrum<ChannelSpec> spectrum;
    return spectrum.row(0).size() == ChannelSpec::frequencyBinCount
        && spectrum.row(1).size() == ChannelSpec::frequencyBinCount
        && spectrum.row(2).empty()
        && spectrum.row(std::numeric_limits<std::size_t>::max()).empty();
}());
static_assert(noexcept(std::declval<const Spectrum<ChannelSpec>&>().row(0)));

// 径向风只存每门一个值，轴和尺寸来自同一份静态规格。
static_assert(std::derived_from<RadialWind<ChannelSpec>, ChannelSpec>);
static_assert(std::is_base_of_v<std::array<double, 2>, RadialWind<ChannelSpec>>);
static_assert(!std::is_convertible_v<RadialWind<ChannelSpec>*, std::array<double, 2>*>);
static_assert(sizeof(RadialWind<ChannelSpec>) == sizeof(std::array<double, 2>));
static_assert(std::same_as<decltype(std::declval<RadialWind<ChannelSpec>&>()[0]), double&>);
static_assert(std::same_as<decltype(std::declval<const RadialWind<ChannelSpec>&>()[0]), const double&>);
static_assert(!HasCompletion<RadialWind<ChannelSpec>> && !HasAxisAliases<RadialWind<ChannelSpec>>);
static_assert(!HasValues<RadialWind<ChannelSpec>> && !HasAxisValidation<RadialWind<ChannelSpec>>);
template<class S> concept CanMakeRadialWind = requires { typename RadialWind<S>;
    typename CNR<S>; };
static_assert(!CanMakeRadialWind<NoRangeGates> && !CanMakeRadialWind<RuntimeRangeGateCount>);
static_assert(!CanMakeRadialWind<RangeStep<3>>);
static_assert([] {
    RadialWind<ChannelSpec> wind;
    if (wind[0] != 0 || wind[1] != 0) return false;
    wind[0] = -3.5;
    wind[1] = 8.0;
    return wind[0] == -3.5 && wind[1] == 8.0;
}());

// 公开计算入口只接受静态 Spectrum，不再按结构接收任意对象。
struct UnrelatedSpectrum
{
    static constexpr int rows = 2, columns = 17;
    static constexpr double rowStart = 30, rowStep = 15, columnStart = 10, columnStep = 2;
    std::array<double, 34> values{};
    bool complete() const { return true; }
    bool hasValidAxes() const { return true; }
    std::span<const double> row(std::size_t index) const
    {
        return {values.data() + index * columns, columns};
    }
};
template<class T>
concept CanCalculate = requires(const T &value) { RadialWindCalculation::Calc(value); };
static_assert(CanCalculate<Spectrum<ChannelSpec>>);
static_assert(!CanCalculate<UnrelatedSpectrum>);
static_assert(!CanCalculate<std::array<double, 34>> && !CanCalculate<std::span<const double>>);
template<class T>
concept CanOverrideCalibration = requires(const T &value) { RadialWindCalculation::Calc(value, 0.8, 16.0); };
static_assert(!CanOverrideCalibration<Spectrum<ChannelSpec>>);

static_assert(StationaryObject<Spectrum<ChannelSpec>> && StationaryObject<RadialWind<ChannelSpec>>
    && StationaryObject<CNR<ChannelSpec>>);
static_assert(!std::same_as<RadialWind<ChannelSpec>, CNR<ChannelSpec>>);
struct NarrowSpec : ChannelSpec { static constexpr int frequencyBinCount = 1; };
struct ShiftedSpec : ChannelSpec { static constexpr double wavelengthM = 1.6e-6, zeroWindMHz = 16; };

int main()
{
    Spectrum<ChannelSpec> fixed{};
    Spectrum<ShiftedSpec> shiftedSpectrum{};
    // 不同门用不同峰位及幅度，验证行列布局没有颠倒。
    for (int gate = 0; gate < fixed.rangeGateCount; ++gate)
        for (int bin = 0; bin < fixed.frequencyBinCount; ++bin)
        {
            const double center = gate == 0 ? 6.0 : 10.0;
            fixed[gate * fixed.frequencyBinCount + bin] = shiftedSpectrum[gate * fixed.frequencyBinCount + bin] = 3 + (gate + 1) * 100
                * std::exp(-(bin - center) * (bin - center) / 2.0);
        }
    const auto baseline = RadialWindCalculation::Calc(fixed);
    const auto shifted = RadialWindCalculation::Calc(shiftedSpectrum);
    if (fixed.row(1).data() != fixed.row(0).data() + ChannelSpec::frequencyBinCount) return 9;
    if (std::abs((*shifted.ws)[0] - 4.8) > 0.01 || std::abs((*shifted.ws)[1] - 11.2) > 0.01) return 8;
    for (int gate = 0; gate < fixed.rangeGateCount; ++gate)
    {
        const double expectedWind = (gate == 0 ? 4.0 : 12.0) * 0.775;
        const double expectedCnr = 10 * std::log10(std::sqrt(2 * std::numbers::pi)
            * (gate + 1) * 100 / (3 * fixed.frequencyBinCount));
        if (std::abs((*baseline.ws)[gate] - expectedWind) > 0.01
            || std::abs((*baseline.cnr)[gate] - expectedCnr) > 0.01) return 3;
    }
    // 频点不足的既有失败分支保留；轴和数组长度已由类型约束。
    const Spectrum<NarrowSpec> narrow;
    if (RadialWindCalculation::Calc(narrow).ws || RadialWindCalculation::Calc(narrow).cnr) return 6;
    std::puts("Spectrum: typed axes, stationary frames and Gaussian numeric baseline passed");
}
