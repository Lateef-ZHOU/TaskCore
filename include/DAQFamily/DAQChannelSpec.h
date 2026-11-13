#pragma once
#include "ObjectPolicy.h"

#include <array>
#include <concepts>
#include <cstddef>
#include <span>
#include <type_traits>
#include <limits>
#include <chrono>
#include <cmath>
#include <numbers>
#include <optional>
#include "Matrix.h"

// 一路采集通道的型号规格，不带帧数据或实例状态；可默认构造且可被继承。
// 所有参数须为 const，且可在编译期读取。
// 当前固定波长及零风速参考沿用型号现值；以后动态配置在计算边界调整，不在此复制运行状态。
// 两条轴的起点必须有限，步长必须有限且为正；不替硬件验证波长或零风速参考的物理含义。
template<class Spec>
concept DAQChannelSpec = std::is_empty_v<Spec> && std::default_initializable<Spec>
    && (!std::is_final_v<Spec>) && requires
{
    { Spec::name } -> std::convertible_to<const char *>;
    { Spec::wavelengthM } -> std::convertible_to<double>;
    { Spec::zeroWindMHz } -> std::convertible_to<double>;
    { Spec::rangeStartM } -> std::convertible_to<double>;
    { Spec::rangeStepM } -> std::convertible_to<double>;
    { Spec::frequencyStartMHz } -> std::convertible_to<double>;
    { Spec::frequencyStepMHz } -> std::convertible_to<double>;
    typename std::integral_constant<int, Spec::rangeGateCount>;
    typename std::integral_constant<int, Spec::frequencyBinCount>;
    requires std::is_const_v<decltype(Spec::name)>;
    requires std::is_const_v<decltype(Spec::wavelengthM)>;
    requires std::is_const_v<decltype(Spec::zeroWindMHz)>;
    requires std::is_const_v<decltype(Spec::rangeStartM)>;
    requires std::is_const_v<decltype(Spec::rangeStepM)>;
    requires std::is_const_v<decltype(Spec::frequencyStartMHz)>;
    requires std::is_const_v<decltype(Spec::frequencyStepMHz)>;
    requires std::is_const_v<decltype(Spec::rangeGateCount)>;
    requires std::is_const_v<decltype(Spec::frequencyBinCount)>;
    // 这些类型要求表达式能常量求值；允许 false，不据此施加正值或频带限制。
    typename std::bool_constant<(Spec::name != nullptr)>;
    typename std::bool_constant<(Spec::wavelengthM > 0)>;
    typename std::bool_constant<(Spec::zeroWindMHz > 0)>;
    // C++20 用有限上下界比较，避免依赖 C++23 才保证 constexpr 的 std::isfinite。
    { std::bool_constant<(Spec::rangeStartM >= std::numeric_limits<double>::lowest()
        && Spec::rangeStartM <= std::numeric_limits<double>::max())>{} } -> std::same_as<std::true_type>;
    { std::bool_constant<(Spec::frequencyStartMHz >= std::numeric_limits<double>::lowest()
        && Spec::frequencyStartMHz <= std::numeric_limits<double>::max())>{} } -> std::same_as<std::true_type>;
    { std::bool_constant<(Spec::rangeStepM > 0.0
        && Spec::rangeStepM <= std::numeric_limits<double>::max())>{} } -> std::same_as<std::true_type>;
    { std::bool_constant<(Spec::frequencyStepMHz > 0.0
        && Spec::frequencyStepMHz <= std::numeric_limits<double>::max())>{} } -> std::same_as<std::true_type>;
    requires (Spec::rangeGateCount > 0 && Spec::frequencyBinCount > 0);
    requires (static_cast<std::size_t>(Spec::rangeGateCount) <= std::numeric_limits<std::size_t>::max()
        / sizeof(double) / static_cast<std::size_t>(Spec::frequencyBinCount));
};

// 新整帧的时间必须在类型上不可变；通用 WallClockElement 的时间提取契约不变。
template<class Frame>
concept ImmutableWallClockFrame = requires {
    requires std::same_as<decltype(Frame::at), const std::chrono::system_clock::time_point>;
};

// 一帧功率谱：静态规格不占实例空间，唯一数据存储是固定长度 double 数组。
// 固定数组禁止复制与移动。按距离门逐行、频点逐列排列。凑帧进度属于 Collector，不由数组容量判断。
// 构造即有完整的零初始化数组；大规格应由上层在堆上的收集实例中原地构造。
template<DAQChannelSpec Spec>
class Spectrum : public Spec,
                 private std::array<double, static_cast<std::size_t>(Spec::rangeGateCount) * Spec::frequencyBinCount>
{
public:
    constexpr Spectrum() noexcept(std::is_nothrow_default_constructible_v<Spec>)
        : std::array<double, static_cast<std::size_t>(Spec::rangeGateCount) * Spec::frequencyBinCount>{} {}

    DISABLE_COPY_MOVE(Spectrum)

    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount) * Spec::frequencyBinCount>::operator[];
    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount) * Spec::frequencyBinCount>::size;
    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount) * Spec::frequencyBinCount>::begin;
    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount) * Spec::frequencyBinCount>::end;

    // 仅从左值借用，视图不保活原谱；越界返回空，先判范围再计算行地址。
    [[nodiscard]] constexpr std::span<const double> row(const std::size_t index) const & noexcept
    {
        if (index >= static_cast<std::size_t>(Spec::rangeGateCount))
            return {};
        return {this->data() + index * static_cast<std::size_t>(Spec::frequencyBinCount),
                static_cast<std::size_t>(Spec::frequencyBinCount)};
    }
    auto row(std::size_t) const && = delete;
};

// 一帧径向风速：每个距离门一个 double（m/s），斜距轴直接取通道规格。
// 固定数组禁止复制与移动；零初始化；是否已得到测量结果由持有方判断，不在数组内维护完成状态。
template<DAQChannelSpec Spec>
class RadialWind : public Spec,
                   private std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>
{
public:
    constexpr RadialWind() noexcept(std::is_nothrow_default_constructible_v<Spec>)
        : std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>{} {}

    DISABLE_COPY_MOVE(RadialWind)

    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>::operator[];
    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>::size;
    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>::begin;
    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>::end;
};

// 一帧信噪比：每个距离门一个 double（dB），斜距轴直接取通道规格。
// 固定数组禁止复制与移动；零初始化；是否已得到测量结果由持有方判断，不在数组内维护完成状态。
template<DAQChannelSpec Spec>
class CNR : public Spec,
                   private std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>
{
public:
    constexpr CNR() noexcept(std::is_nothrow_default_constructible_v<Spec>)
        : std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>{} {}

    DISABLE_COPY_MOVE(CNR)

    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>::operator[];
    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>::size;
    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>::begin;
    using std::array<double, static_cast<std::size_t>(Spec::rangeGateCount)>::end;
};

// 通道反演结果：每个距离门一个 ENU 风矢量（m/s）；nullopt 为缺测，零向量为静风。
// 时间和高度角构造后固定；直接填入本帧数组，完成后交 shared_ptr<const Frame>。
// 禁止整帧复制和移动；历史仅替换共享指针。公开数组借用须在帧存活期间使用。
// 只保存本轮平均高度角；高度 = 规格斜距 × sin(高度角)，不重复保存高度起点和层距。
template<DAQChannelSpec Spec>
struct VerticalWindProfileUnit : public Spec,
    public std::array<std::optional<LinAlg::Vec3>, Spec::rangeGateCount>
{
    const std::chrono::system_clock::time_point at;
    const double elevationDeg;

    constexpr VerticalWindProfileUnit(std::chrono::system_clock::time_point time, double elevation)
        noexcept(std::is_nothrow_default_constructible_v<Spec>)
        : std::array<std::optional<LinAlg::Vec3>, Spec::rangeGateCount>{}, at(time), elevationDeg(elevation) {}
    DISABLE_COPY_MOVE(VerticalWindProfileUnit)

    [[nodiscard]] constexpr std::span<const std::optional<LinAlg::Vec3>, Spec::rangeGateCount> values() const & noexcept
    { return std::span<const std::optional<LinAlg::Vec3>, Spec::rangeGateCount>(this->data(), this->size()); }
    auto values() const && = delete;

    [[nodiscard]] double heightStartM() const noexcept
    { return Spec::rangeStartM * std::sin(elevationDeg * std::numbers::pi / 180.0); }
    [[nodiscard]] double heightStepM() const noexcept
    { return Spec::rangeStepM * std::sin(elevationDeg * std::numbers::pi / 180.0); }
};
