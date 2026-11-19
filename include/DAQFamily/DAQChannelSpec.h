#pragma once
#include "Matrix.h"
#include "ObjectPolicy.h"

#include <array>
#include <chrono>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <span>

// 一路采集通道的数值规格：非类型模板参数，编译期定值。
// 两条轴的起点须有限，步长须有限且为正；标定值由型号提供。
// 不在此复制运行状态；动态配置在计算边界调整。
struct DAQParams
{
    double wavelengthM = 0.0;
    double zeroWindMHz = 0.0;
    double rangeStartM = 0.0;
    double rangeStepM = 0.0;
    double frequencyStartMHz = 0.0;
    double frequencyStepMHz = 0.0;
    int rangeGateCount = 0;
    int frequencyBinCount = 0;
};

template<DAQParams P>
concept ValidDAQGrid =
    P.rangeStartM >= std::numeric_limits<double>::lowest() && P.rangeStartM <= std::numeric_limits<double>::max()
    && P.frequencyStartMHz >= std::numeric_limits<double>::lowest() && P.frequencyStartMHz <= std::numeric_limits<double>::max()
    && P.rangeStepM > 0.0 && P.rangeStepM <= std::numeric_limits<double>::max()
    && P.frequencyStepMHz > 0.0 && P.frequencyStepMHz <= std::numeric_limits<double>::max()
    && P.rangeGateCount > 0 && P.frequencyBinCount > 0
    && static_cast<std::size_t>(P.rangeGateCount) <= std::numeric_limits<std::size_t>::max()
        / sizeof(double) / static_cast<std::size_t>(P.frequencyBinCount);

// 新整帧的时间必须在类型上不可变；通用 WallClockElement 的时间提取契约不变。
template<class Frame>
concept ImmutableWallClockFrame = requires {
    requires std::same_as<decltype(Frame::at), const std::chrono::system_clock::time_point>;
};

// 一帧功率谱：规格不占实例空间，唯一数据存储是固定长度 double 数组。
// 固定数组禁止复制与移动。按距离门逐行、频点逐列排列。
template<DAQParams P> requires ValidDAQGrid<P>
class Spectrum : private std::array<double, static_cast<std::size_t>(P.rangeGateCount) * static_cast<std::size_t>(P.frequencyBinCount)>
{
    using Array = std::array<double, static_cast<std::size_t>(P.rangeGateCount) * static_cast<std::size_t>(P.frequencyBinCount)>;
public:
    constexpr Spectrum() noexcept : Array{} {}
    DISABLE_COPY_MOVE(Spectrum)

    using Array::operator[];
    using Array::size;
    using Array::begin;
    using Array::end;

    // 仅从左值借用，视图不保活原谱；越界返回空。
    [[nodiscard]] constexpr std::span<const double> row(const std::size_t index) const & noexcept
    {
        if (index >= static_cast<std::size_t>(P.rangeGateCount))
            return {};
        return {this->data() + index * static_cast<std::size_t>(P.frequencyBinCount),
                static_cast<std::size_t>(P.frequencyBinCount)};
    }
    auto row(std::size_t) const && = delete;

    static constexpr DAQParams params = P;
};

// 一帧径向风速：每个距离门一个 double（m/s）。
template<DAQParams P> requires ValidDAQGrid<P>
class RadialWind : private std::array<double, static_cast<std::size_t>(P.rangeGateCount)>
{
public:
    constexpr RadialWind() noexcept : std::array<double, static_cast<std::size_t>(P.rangeGateCount)>{} {}
    DISABLE_COPY_MOVE(RadialWind)

    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::operator[];
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::size;
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::begin;
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::end;

    static constexpr DAQParams params = P;
};

// 一帧信噪比：每个距离门一个 double（dB）。
template<DAQParams P> requires ValidDAQGrid<P>
class CNR : private std::array<double, static_cast<std::size_t>(P.rangeGateCount)>
{
public:
    constexpr CNR() noexcept : std::array<double, static_cast<std::size_t>(P.rangeGateCount)>{} {}
    DISABLE_COPY_MOVE(CNR)

    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::operator[];
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::size;
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::begin;
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::end;

    static constexpr DAQParams params = P;
};

// 通道反演结果：每个距离门一个 ENU 风矢量（m/s）；nullopt 为缺测，零向量为静风。
template<DAQParams P> requires ValidDAQGrid<P>
struct VerticalWindProfileUnit : public std::array<std::optional<LinAlg::Vec3>, P.rangeGateCount>
{
    const std::chrono::system_clock::time_point at;
    const double elevationDeg;

    constexpr VerticalWindProfileUnit(std::chrono::system_clock::time_point time, double elevation) noexcept
        : std::array<std::optional<LinAlg::Vec3>, P.rangeGateCount>{}, at(time), elevationDeg(elevation) {}
    DISABLE_COPY_MOVE(VerticalWindProfileUnit)

    [[nodiscard]] constexpr std::span<const std::optional<LinAlg::Vec3>, P.rangeGateCount> values() const & noexcept
    { return std::span<const std::optional<LinAlg::Vec3>, P.rangeGateCount>(this->data(), this->size()); }
    auto values() const && = delete;

    [[nodiscard]] double heightStartM() const noexcept
    { return P.rangeStartM * std::sin(elevationDeg * std::numbers::pi / 180.0); }
    [[nodiscard]] double heightStepM() const noexcept
    { return P.rangeStepM * std::sin(elevationDeg * std::numbers::pi / 180.0); }

    static constexpr DAQParams params = P;
};
