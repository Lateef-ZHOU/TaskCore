#pragma once
#include <cmath>
#include <memory>
#include <numbers>
#include <utility>

#include "GaussFit.h"
#include "DAQFamily/DAQChannelSpec.h"

// ============================================================
// RadialWindCalculation — 功率谱 → 逐距离点的径向风速 + CNR
//
// 一帧谱是距离点数 × 频点数的行主序矩阵，每一行是该距离点的一条频谱。
// 拟合峰位 x0 是频点坐标；频率映射的 velocityAt(x0) 直接给出径向风速（m/s），
// 标定（波长、零风速参考）已并进频率映射。
// 信号功率 = sqrt(2π)·|A·w|；噪声 = |d|·频点数；CNR = 10·log10(signal/noise)。
//
// 功率谱、径向风和 CNR 全程保留轴规格；Qt 登记由具体型号边界负责。
// ============================================================
namespace RadialWindCalculation
{
// 相干多普勒测风：径向风速 v = λ·f_D / 2，每 MHz 频移对应 λ/2 × 10⁶ m/s（λ 以米计；1550 nm → 0.775）。
[[nodiscard]] constexpr double metersPerSecondPerMHz(const double wavelengthM) noexcept
{
    return wavelengthM / 2.0 * 1e6;
}

// 两个独立量纲的静态产物；只读份额保留原通道的距离轴。
template<class Range>
struct Result
{
    std::shared_ptr<const RadialWind<Range>> ws;
    std::shared_ptr<const CNR<Range>> cnr;
};

template<class Range, class Frequency>
[[nodiscard]] Result<Range> Calc(const Spectrum<Range, Frequency> &spectrum)
{
    if constexpr (Frequency::count < 2) return {};
    auto ws = std::make_shared<RadialWind<Range>>();
    auto cnr = std::make_shared<CNR<Range>>();
#pragma omp parallel for
    for (int row = 0; row < Range::count; ++row)
    {
        const GaussFit::Params p = GaussFit(spectrum.row(row)).params;
        const double signal = std::sqrt(2.0 * std::numbers::pi) * std::abs(p.A * p.w);
        const double noise = std::abs(p.d) * Frequency::count;
        (*ws)[row] = Frequency::velocityAt(p.x0);
        (*cnr)[row] = noise > 1e-30 ? 10.0 * std::log10(signal / noise) : 0.0;
    }
    return {std::move(ws), std::move(cnr)};
}
} // namespace RadialWindCalculation
