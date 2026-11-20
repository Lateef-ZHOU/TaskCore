#pragma once
#include "Matrix.h"
#include "ObjectPolicy.h"

#include <algorithm>
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

// 一条轴：均匀轴查起点/步长/终点，非均匀轴逐点验严格递增。
template<class T>
[[nodiscard]] consteval bool validAxis() noexcept
{
    if constexpr (T::uniform)
        return finite(T::start) && T::step > 0.0 && finite(T::stop);
    else
    {
        auto before = T::at(0);
        if (!finite(before)) return false;
        for (std::size_t i = 1; i <= static_cast<std::size_t>(T::count); ++i)
        {
            const auto current = T::at(i);
            if (!finite(current) || current <= before) return false;
            before = current;
        }
        return true;
    }
}
} // namespace DAQDetail

// 映射的公共接口：坐标轴种类 uniform、点数 count、覆盖终点 stop、逐点坐标 at；
// 非均匀映射还须带逐点边界 edges。均匀性由映射类型自己声明，不另存标志。
template<class T>
concept Mapping = requires(std::size_t index) {
    { T::uniform } -> std::convertible_to<bool>;
    { T::count } -> std::convertible_to<int>;
    { T::stop } -> std::convertible_to<double>;
    { T::at(index) } -> std::same_as<double>;
} && (T::uniform || requires { T::edges; });

// 均匀映射：起点、步长、点数是静态模板值。步长是权威值——用覆盖终点反推步长要做除法，
// 既有舍入也说不清末点边界归谁；覆盖终点由乘加导出，是末点的右边界（不含在点内）。
template<double Start, double Step, int Count>
struct UniformMapping
{
    static_assert(Count > 0, "点数须为正");
    static_assert(DAQDetail::finite(Start) && DAQDetail::finite(Step) && Step > 0.0,
                  "轴起点须有限、步长须为正");
    static constexpr bool uniform = true;
    static constexpr double start = Start;
    static constexpr double step = Step;
    static constexpr int count = Count;
    static constexpr double stop = Start + Step * static_cast<double>(Count);

    [[nodiscard]] static constexpr double at(std::size_t index) noexcept
    { return Start + Step * static_cast<double>(index); }
};

// 非均匀映射：逐点坐标由模板实参的 constexpr 函数在编译期生成，映射自己静态持有
// std::array<double, N>（N = 点数 + 1，末项是覆盖终点）；坐标不进类型名，大表也不会撑爆 QML 元类型签名。
template<auto Edges>
struct NonUniformMapping
{
    static constexpr bool uniform = false;
    static constexpr auto edges = Edges();
    static_assert(edges.size() > 1, "非均匀映射至少需要首点坐标和覆盖终点");
    static constexpr int count = static_cast<int>(edges.size()) - 1;
    static constexpr double stop = edges.back();

    [[nodiscard]] static constexpr double at(std::size_t index) noexcept { return edges[index]; }
};

// 频率映射：在频率轴之上并入多普勒标定，同一类型给出 MHz 与 m/s 两个视图。
// 频谱只带「距离映射 + 频率映射」两个参数，标定不再单独存放。
template<Mapping Frequency, double WavelengthM, double ZeroWindMHz>
struct DopplerMapping : Frequency
{
    static_assert(DAQDetail::finite(WavelengthM) && WavelengthM > 0.0, "波长须为有限正数");
    static constexpr double wavelengthM = WavelengthM;
    static constexpr double zeroWindMHz = ZeroWindMHz;

    // 拟合峰位置（可为分数 bin）→ 径向风速（m/s）。均匀轴退化为线性式，非均匀轴在相邻点间插值。
    [[nodiscard]] static double velocityAt(const double bin) noexcept
    {
        const double perMHz = WavelengthM / 2.0 * 1e6;
        if constexpr (Frequency::uniform)
            return (Frequency::start + Frequency::step * bin - ZeroWindMHz) * perMHz;
        else
        {
            const auto last = static_cast<std::size_t>(Frequency::count);
            const double clamped = std::clamp(bin, 0.0, static_cast<double>(last));
            const auto low = static_cast<std::size_t>(clamped);
            const auto high = std::min(low + 1, last);
            const double t = clamped - static_cast<double>(low);
            const double lowVelocity = (Frequency::at(low) - ZeroWindMHz) * perMHz;
            const double highVelocity = (Frequency::at(high) - ZeroWindMHz) * perMHz;
            return lowVelocity + (highVelocity - lowVelocity) * t;
        }
    }
};

// 频率映射必须能换算风速：带标定常数与速度视图。
template<class T>
concept Doppler = Mapping<T> && requires(double bin) {
    { T::velocityAt(bin) } -> std::same_as<double>;
    { T::zeroWindMHz } -> std::convertible_to<double>;
    { T::wavelengthM } -> std::convertible_to<double>;
};

// 一条轴可作帧维度：映射有效、点数正、坐标严格递增。
template<class Range>
concept ValidAxis = Mapping<Range> && Range::count > 0 && DAQDetail::validAxis<Range>();

// 功率谱两维：距离轴与频率轴（带多普勒标定）。
template<class Range, class Frequency>
concept ValidGrid = ValidAxis<Range> && ValidAxis<Frequency> && Doppler<Frequency>
    && static_cast<std::size_t>(Range::count) <= std::numeric_limits<std::size_t>::max()
        / sizeof(double) / static_cast<std::size_t>(Frequency::count);

// 新整帧的时间必须在类型上不可变；通用 WallClockElement 的时间提取契约不变。
template<class Frame>
concept ImmutableWallClockFrame = requires {
    requires std::same_as<decltype(Frame::at), const std::chrono::system_clock::time_point>;
};

// 一帧功率谱：距离点逐行、频点逐列；规格不占实例空间，唯一数据存储是固定长度 double 数组。
template<class Range, class Frequency> requires ValidGrid<Range, Frequency>
class Spectrum : private std::array<double, static_cast<std::size_t>(Range::count) * static_cast<std::size_t>(Frequency::count)>
{
    using Array = std::array<double, static_cast<std::size_t>(Range::count) * static_cast<std::size_t>(Frequency::count)>;
public:
    using RangeType = Range;
    using FrequencyType = Frequency;
    constexpr Spectrum() noexcept : Array{} {}
    DISABLE_COPY_MOVE(Spectrum)

    using Array::operator[];
    using Array::size;
    using Array::begin;
    using Array::end;

    // 仅从左值借用，视图不保活原谱；越界返回空。
    [[nodiscard]] constexpr std::span<const double> row(const std::size_t index) const & noexcept
    {
        if (index >= static_cast<std::size_t>(Range::count))
            return {};
        return {this->data() + index * static_cast<std::size_t>(Frequency::count),
                static_cast<std::size_t>(Frequency::count)};
    }
    auto row(std::size_t) const && = delete;

    static constexpr Range range{};
    static constexpr Frequency frequency{};
};

// 一帧径向风速：每个距离点一个 double（m/s）。
template<class Range> requires ValidAxis<Range>
class RadialWind : private std::array<double, static_cast<std::size_t>(Range::count)>
{
public:
    using RangeType = Range;
    constexpr RadialWind() noexcept : std::array<double, static_cast<std::size_t>(Range::count)>{} {}
    DISABLE_COPY_MOVE(RadialWind)

    using std::array<double, static_cast<std::size_t>(Range::count)>::operator[];
    using std::array<double, static_cast<std::size_t>(Range::count)>::size;
    using std::array<double, static_cast<std::size_t>(Range::count)>::begin;
    using std::array<double, static_cast<std::size_t>(Range::count)>::end;

    static constexpr Range range{};
};

// 一帧信噪比：每个距离点一个 double（dB）。
template<class Range> requires ValidAxis<Range>
class CNR : private std::array<double, static_cast<std::size_t>(Range::count)>
{
public:
    using RangeType = Range;
    constexpr CNR() noexcept : std::array<double, static_cast<std::size_t>(Range::count)>{} {}
    DISABLE_COPY_MOVE(CNR)

    using std::array<double, static_cast<std::size_t>(Range::count)>::operator[];
    using std::array<double, static_cast<std::size_t>(Range::count)>::size;
    using std::array<double, static_cast<std::size_t>(Range::count)>::begin;
    using std::array<double, static_cast<std::size_t>(Range::count)>::end;

    static constexpr Range range{};
};

// 通道反演结果：每个距离点一个 ENU 风矢量（m/s）；nullopt 为缺测，零向量为静风。
// 高度 = 距离点坐标 × sin(仰角)，仰角随帧（运行时）保存。
template<class Range> requires ValidAxis<Range>
struct VerticalWindProfileUnit : public std::array<std::optional<LinAlg::Vec3>, static_cast<std::size_t>(Range::count)>
{
    using RangeType = Range;
    const std::chrono::system_clock::time_point at;
    const double elevationDeg;

    constexpr VerticalWindProfileUnit(std::chrono::system_clock::time_point time, double elevation) noexcept
        : std::array<std::optional<LinAlg::Vec3>, static_cast<std::size_t>(Range::count)>{}, at(time), elevationDeg(elevation) {}
    DISABLE_COPY_MOVE(VerticalWindProfileUnit)

    [[nodiscard]] constexpr std::span<const std::optional<LinAlg::Vec3>, static_cast<std::size_t>(Range::count)> values() const & noexcept
    { return std::span<const std::optional<LinAlg::Vec3>, static_cast<std::size_t>(Range::count)>(this->data(), this->size()); }
    auto values() const && = delete;

    [[nodiscard]] double heightScale() const noexcept
    { return std::sin(elevationDeg * std::numbers::pi / 180.0); }
    [[nodiscard]] double heightStartM() const noexcept { return Range::at(0) * heightScale(); }
    [[nodiscard]] double heightStepM() const noexcept requires (Range::uniform)
    { return Range::step * heightScale(); }
    [[nodiscard]] double heightAt(std::size_t gate) const noexcept
    {
        const auto scale = heightScale();
        if constexpr (Range::uniform)
            return Range::start * scale + (Range::step * scale) * static_cast<double>(gate);
        else
            return Range::at(gate) * scale;
    }

    static constexpr Range range{};
};
