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
} // namespace DAQDetail

// 均匀映射：起点、步长、门数是静态模板值。步长是权威值——用覆盖终点反推步长要做除法，
// 既有舍入也说不清末门边界归谁；覆盖终点由乘加导出，是末门的右边界（不含在门内）。
template<double Start, double Step, int GateCount>
struct UniformMapping
{
    static_assert(GateCount > 0, "门数须为正");
    static_assert(DAQDetail::finite(Start) && DAQDetail::finite(Step) && Step > 0.0,
                  "距离起点须有限、步长须为正");
    static constexpr bool uniform = true;
    static constexpr double start = Start;
    static constexpr double step = Step;
    static constexpr int gateCount = GateCount;
    static constexpr double stop = Start + Step * static_cast<double>(GateCount);

    [[nodiscard]] static constexpr double at(std::size_t gate) noexcept
    { return Start + Step * static_cast<double>(gate); }
};

// 非均匀映射：逐门起点加末门右边界。表由模板实参的 constexpr 函数在编译期生成，
// 映射自己静态持有 std::array<double, N>；逐门坐标不进类型名，大表也不会撑爆 QML 元类型签名。
template<auto Edges>
struct NonUniformMapping
{
    static constexpr bool uniform = false;
    static constexpr auto edges = Edges();
    static_assert(edges.size() > 1, "非均匀映射至少需要首门起点和覆盖终点");
    static constexpr int gateCount = static_cast<int>(edges.size()) - 1;
    static constexpr double stop = edges.back();

    [[nodiscard]] static constexpr double at(std::size_t gate) noexcept { return edges[gate]; }
};

// 映射的公共接口：门数、覆盖终点、逐门起点，以及映射种类 uniform；DAQParams 按距离门轴使用它。
// 均匀性由映射类型自己声明（与 gateCount/stop 同为静态常量）；非均匀映射还须带逐门边界 edges。
template<class T>
concept RangeMapping = requires(std::size_t gate) {
    { T::uniform } -> std::convertible_to<bool>;
    { T::gateCount } -> std::convertible_to<int>;
    { T::stop } -> std::convertible_to<double>;
    { T::at(gate) } -> std::same_as<double>;
} && (T::uniform || requires { T::edges; });

// 通道数值规格：距离门映射由类型给出，波长、频率与频点数仍是值。
// 帧类按规格值确定数组尺寸，因此同一映射类型加同一组数值就是同一规格。
template<RangeMapping Range>
struct DAQParams
{
    double wavelengthM = 0.0;
    double zeroWindMHz = 0.0;
    double frequencyStartMHz = 0.0;
    double frequencyStepMHz = 0.0;
    int frequencyBinCount = 0;

    static constexpr Range range{};
    static constexpr int rangeGateCount = Range::gateCount;
    static constexpr bool uniformRange = Range::uniform;

    [[nodiscard]] static constexpr double rangeAt(std::size_t gate) noexcept { return Range::at(gate); }
    [[nodiscard]] static constexpr double rangeStopM() noexcept { return Range::stop; }
};

namespace DAQDetail {
template<auto P>
[[nodiscard]] consteval bool validRange()
{
    if constexpr (P.uniformRange)
        return finite(P.range.start) && P.range.step > 0.0 && finite(P.range.stop);
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
    { return P.range.step * heightScale(); }
    [[nodiscard]] double heightAt(std::size_t gate) const noexcept
    {
        const auto scale = heightScale();
        if constexpr (P.uniformRange)
            return P.range.start * scale + (P.range.step * scale) * static_cast<double>(gate);
        else
            return P.rangeAt(gate) * scale;
    }

    static constexpr const auto &params = P;
};
