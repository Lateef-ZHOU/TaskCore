#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <numbers>
#include <utility>

#include "GaussFit.h"
#include "DataArrays.h"

// ============================================================
// RadialWindCalculation — 功率谱 → 逐距离点的径向风速 + CNR
//
// 一帧谱是距离点数 × 频点数的行主序矩阵，每一行是该距离点的一条频谱。
// 拟合峰位 x0 是频点坐标；风速由调用方交入的「下标 → 风速」映射给出（均匀或非均匀皆可），
// 族基不持有波长、零风速参考等型号标定知识；分数下标在相邻两点间线性插值。
// 信号功率 = sqrt(2π)·|A·w|；噪声 = |d|·频点数；CNR = 10·log10(signal/noise)。
//
// 功率谱、径向风和 CNR 全程保留轴规格；Qt 登记由具体型号边界负责。
// ============================================================
namespace RadialWindCalculation
{

// 风速映射在下标处的取值；分数下标（拟合峰位）在相邻两点间线性插值。
template<MappingLike Wind>
[[nodiscard]] double windAtBin(const double bin) noexcept
{
    if (!std::isfinite(bin)) return std::numeric_limits<double>::quiet_NaN();
    const auto last = static_cast<std::size_t>(Wind::count);
    const double clamped = std::clamp(bin, 0.0, static_cast<double>(last));
    const auto low = static_cast<std::size_t>(clamped);
    const auto high = std::min(low + 1, last);
    const double fraction = clamped - static_cast<double>(low);
    const double lower = Wind::at(low);
    const double upper = Wind::at(high);
    return lower + (upper - lower) * fraction;
}

// 两个独立量纲的静态产物；只读份额保留原通道的距离轴。
template<class Range>
struct Result
{
    std::shared_ptr<const Array1D<Range>> ws;
    std::shared_ptr<const Array1D<Range>> cnr;
};

template<MappingLike Range, MappingLike Frequency, MappingLike Wind>
[[nodiscard]] Result<Range> Calc(const Grid2D<Range, Frequency> &spectrum, Wind)
{
    static_assert(Wind::count == Frequency::count, "风速映射的点数须与频率轴一致");
    if constexpr (Frequency::count < 2) return {};
    auto ws = std::make_shared<Array1D<Range>>();
    auto cnr = std::make_shared<Array1D<Range>>();
#pragma omp parallel for
    for (int row = 0; row < Range::count; ++row)
    {
        const GaussFit::Params p = GaussFit(spectrum.row(row)).params;
        const double signal = std::sqrt(2.0 * std::numbers::pi) * std::abs(p.A * p.w);
        const double noise = std::abs(p.d) * Frequency::count;
        (*ws)[row] = windAtBin<Wind>(p.x0);
        (*cnr)[row] = noise > 1e-30 ? 10.0 * std::log10(signal / noise) : 0.0;
    }
    return {std::move(ws), std::move(cnr)};
}
} // namespace RadialWindCalculation
