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
#include <type_traits>

namespace DAQDetail {
[[nodiscard]] constexpr bool finite(double value) noexcept
{ return value >= std::numeric_limits<double>::lowest() && value <= std::numeric_limits<double>::max(); }

// 非均匀轴：逐点有限且严格递增。有效性由映射自己在类型上保证，帧不再重复校验。
template<std::size_t N>
[[nodiscard]] constexpr bool strictlyIncreasing(const std::array<double, N> &values) noexcept
{
    if (!finite(values[0])) return false;
    for (std::size_t i = 1; i < N; ++i)
        if (!finite(values[i]) || !(values[i] > values[i - 1])) return false;
    return true;
}
} // namespace DAQDetail

// 映射只有两种模板：均匀与非均匀，二者都继承 MappingTag；派生映射（例如测试里的缺测风速）
// 也一并符合。识别不依赖前置声明：标签基类先定义，概念放在类型定义之后。
struct MappingTag {};

// 均匀映射：起点、步长、点数是静态模板值。步长是权威值——用覆盖终点反推步长要做除法，
// 既有舍入也说不清末点边界归谁；覆盖终点由乘加导出，是末点的右边界（不含在点内）。
template<double Start, double Step, int Count>
struct UniformMapping : MappingTag
{
    static_assert(Count > 0, "点数须为正");
    static_assert(DAQDetail::finite(Start) && DAQDetail::finite(Step) && Step > 0.0,
                  "轴起点须有限、步长须为正");
    static constexpr bool uniform = true;
    static constexpr double start = Start;
    static constexpr double step = Step;
    static constexpr int count = Count;
    static constexpr double stop = Start + Step * static_cast<double>(Count);
    static_assert(DAQDetail::finite(stop), "轴的覆盖终点须有限");

    [[nodiscard]] static constexpr double at(std::size_t index) noexcept
    { return Start + Step * static_cast<double>(index); }
};

// 非均匀映射：逐点坐标由模板实参的 constexpr 函数在编译期生成，映射自己静态持有
// std::array<double, N>（N = 点数 + 1，末项是覆盖终点）；坐标不进类型名，大表也不会撑爆 QML 元类型签名。
template<auto Edges>
struct NonUniformMapping : MappingTag
{
    static constexpr bool uniform = false;
    static constexpr auto edges = Edges();
    static_assert(edges.size() > 1, "非均匀映射至少需要首点坐标和覆盖终点");
    static_assert(DAQDetail::strictlyIncreasing(edges), "非均匀映射的坐标须有限且严格递增");
    static constexpr int count = static_cast<int>(edges.size()) - 1;
    static constexpr double stop = edges.back();

    [[nodiscard]] static constexpr double at(std::size_t index) noexcept { return edges[index]; }
};

// 两种映射模板及其派生类型；使用点只写这个概念名。
template<class T>
concept MappingLike = std::derived_from<std::remove_cvref_t<T>, MappingTag>;

// 新整帧的时间必须在类型上不可变；通用 WallClockElement 的时间提取契约不变。
template<class Frame>
concept ImmutableWallClockFrame = requires {
    requires std::same_as<decltype(Frame::at), const std::chrono::system_clock::time_point>;
};

// 一帧功率谱：距离点逐行、频点逐列；规格不占实例空间，唯一数据存储是固定长度 double 数组。
template<MappingLike Range, MappingLike Frequency>
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
template<MappingLike Range>
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
template<MappingLike Range>
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
template<MappingLike Range>
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
