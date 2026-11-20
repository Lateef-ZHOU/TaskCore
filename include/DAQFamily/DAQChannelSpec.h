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

namespace DAQDetail {
[[nodiscard]] constexpr bool finite(double value) noexcept
{ return value >= std::numeric_limits<double>::lowest() && value <= std::numeric_limits<double>::max(); }

// 分段距离参数的结构检查：形状为 {起点, 门数, 门宽, …}，门数须为正整数。
// 起点、门宽与展开后的边界是否可用由 ValidDAQGrid 判定，不在这里拒绝。
template<auto Segments>
[[nodiscard]] consteval bool segmentedShape() noexcept
{
    if (Segments.size() < 3 || Segments.size() % 2 == 0) return false;
    for (std::size_t i = 1; i < Segments.size(); i += 2)
    {
        const double count = Segments[i];
        if (!finite(count) || count < 1.0 || count > static_cast<double>(std::numeric_limits<int>::max()))
            return false;
        if (count != static_cast<double>(static_cast<int>(count))) return false; // 门数须是整数
    }
    return true;
}

template<auto Segments>
[[nodiscard]] consteval int segmentedGateCount() noexcept
{
    int gates = 0;
    for (std::size_t i = 1; i < Segments.size(); i += 2)
        gates += static_cast<int>(Segments[i]);
    return gates;
}

// 按段展开成逐门边界：段内等宽，段间可不等；末项是覆盖终点。
template<auto Segments>
[[nodiscard]] consteval auto segmentedEdges() noexcept
{
    std::array<double, static_cast<std::size_t>(segmentedGateCount<Segments>()) + 1> edges{};
    double position = Segments[0];
    std::size_t gate = 0;
    for (std::size_t i = 1; i < Segments.size(); i += 2)
        for (int k = 0, count = static_cast<int>(Segments[i]); k < count; ++k)
        {
            edges[gate++] = position;
            position += Segments[i + 1];
        }
    edges[gate] = position;
    return edges;
}
} // namespace DAQDetail

struct UniformRangeMapping
{
    double start;
    double step;

    [[nodiscard]] constexpr double operator[](std::size_t gate) const noexcept
    { return start + step * static_cast<double>(gate); }
};

// 非均匀距离门的分段参数：{首门起点, 门数₁, 门宽₁, 门数₂, 门宽₂, …}，按值作为模板实参交入
// （std::array<double, N>）。段内等宽、段间可不等；门数须是正整数。类型即值：同一组分段在不同位置
// 声明得到同一类型，帧与规格实例均不持有副本；展开后的边界表按类型保留一个静态副本，末项是覆盖终点。
template<auto Segments>
struct NonUniformRangeMapping
{
    static_assert(DAQDetail::segmentedShape<Segments>(),
                  "分段距离参数须为 {起点, 门数, 门宽, …}，门数须为正整数");
    static constexpr int gateCount = DAQDetail::segmentedGateCount<Segments>();
    static constexpr std::array<double, static_cast<std::size_t>(gateCount) + 1> kEdges =
        DAQDetail::segmentedEdges<Segments>();
    static constexpr std::span<const double> edges{kEdges.data(), kEdges.size()};

    [[nodiscard]] constexpr double operator[](std::size_t gate) const noexcept { return edges[gate]; }
};

// 均匀通道的数值规格。起点和步长独立给定，兼容现有均匀通道的构造。
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
    static constexpr bool uniformRange = true;

    [[nodiscard]] constexpr double rangeAt(std::size_t gate) const noexcept
    { return UniformRangeMapping{rangeStartM, rangeStepM}[gate]; }
    [[nodiscard]] constexpr double rangeStopM() const noexcept { return rangeAt(rangeGateCount); }
};

template<auto Segments>
struct NonUniformDAQParams
{
    double wavelengthM = 0.0;
    double zeroWindMHz = 0.0;
    double frequencyStartMHz = 0.0;
    double frequencyStepMHz = 0.0;
    int frequencyBinCount = 0;
    static constexpr NonUniformRangeMapping<Segments> range{};
    static constexpr int rangeGateCount = NonUniformRangeMapping<Segments>::gateCount;
    static constexpr bool uniformRange = false;

    [[nodiscard]] static constexpr double rangeAt(std::size_t gate) noexcept { return range[gate]; }
    [[nodiscard]] static constexpr double rangeStopM() noexcept { return range[rangeGateCount]; }
};

namespace DAQDetail {
template<auto P>
[[nodiscard]] consteval bool validRange()
{
    if constexpr (P.uniformRange)
        return finite(P.rangeStartM) && finite(P.rangeStepM) && P.rangeStepM > 0 && finite(P.rangeStopM());
    else
    {
        auto before = P.rangeAt(0);
        if (!finite(before)) return false;
        for (std::size_t i = 1; i <= static_cast<std::size_t>(P.rangeGateCount); ++i)
        {
            const auto current = P.rangeAt(i);
            if (!finite(current) || current <= before) return false;
            before = current;
        }
        return true;
    }
}
}

template<auto P>
concept ValidDAQGrid = P.rangeGateCount > 0 && P.frequencyBinCount > 0
    && (DAQDetail::validRange<P>())
    && DAQDetail::finite(P.frequencyStartMHz)
    && P.frequencyStepMHz > 0.0 && P.frequencyStepMHz <= std::numeric_limits<double>::max()
    && static_cast<std::size_t>(P.rangeGateCount) <= std::numeric_limits<std::size_t>::max()
        / sizeof(double) / static_cast<std::size_t>(P.frequencyBinCount);

// 新整帧的时间必须在类型上不可变；通用 WallClockElement 的时间提取契约不变。
template<class Frame>
concept ImmutableWallClockFrame = requires {
    requires std::same_as<decltype(Frame::at), const std::chrono::system_clock::time_point>;
};

// 一帧功率谱：规格不占实例空间，唯一数据存储是固定长度 double 数组。
// 固定数组禁止复制与移动。按距离门逐行、频点逐列排列。
template<auto P> requires ValidDAQGrid<P>
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

    static constexpr const auto &params = P;
};

// 一帧径向风速：每个距离门一个 double（m/s）。
template<auto P> requires ValidDAQGrid<P>
class RadialWind : private std::array<double, static_cast<std::size_t>(P.rangeGateCount)>
{
public:
    constexpr RadialWind() noexcept : std::array<double, static_cast<std::size_t>(P.rangeGateCount)>{} {}
    DISABLE_COPY_MOVE(RadialWind)

    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::operator[];
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::size;
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::begin;
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::end;

    static constexpr const auto &params = P;
};

// 一帧信噪比：每个距离门一个 double（dB）。
template<auto P> requires ValidDAQGrid<P>
class CNR : private std::array<double, static_cast<std::size_t>(P.rangeGateCount)>
{
public:
    constexpr CNR() noexcept : std::array<double, static_cast<std::size_t>(P.rangeGateCount)>{} {}
    DISABLE_COPY_MOVE(CNR)

    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::operator[];
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::size;
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::begin;
    using std::array<double, static_cast<std::size_t>(P.rangeGateCount)>::end;

    static constexpr const auto &params = P;
};

// 通道反演结果：每个距离门一个 ENU 风矢量（m/s）；nullopt 为缺测，零向量为静风。
template<auto P> requires ValidDAQGrid<P>
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

    [[nodiscard]] double heightScale() const noexcept
    { return std::sin(elevationDeg * std::numbers::pi / 180.0); }
    [[nodiscard]] double heightStartM() const noexcept { return P.rangeAt(0) * heightScale(); }
    [[nodiscard]] double heightStepM() const noexcept requires (P.uniformRange)
    { return P.rangeStepM * heightScale(); }
    [[nodiscard]] double heightAt(std::size_t gate) const noexcept
    {
        const auto scale = heightScale();
        if constexpr (P.uniformRange)
            return P.rangeStartM * scale + (P.rangeStepM * scale) * static_cast<double>(gate);
        else
            return P.rangeAt(gate) * scale;
    }

    static constexpr const auto &params = P;
};
