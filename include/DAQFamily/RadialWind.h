#pragma once
#include <cmath>
#include <memory>
#include <numbers>
#include <utility>

#include "GaussFit.h"
#include "DAQFamily/DAQChannelSpec.h"

// ============================================================
// RadialWindCalculation — 功率谱 → 逐距离门的径向风速 + CNR
//
// 一帧谱是距离门数 × 频点数的行主序矩阵，每一行是该距离门的一条频谱。
// 拟合峰位 x0 是频点坐标；多普勒频移（MHz）为
// frequencyStartMHz + x0·frequencyStepMHz - zeroWindMHz，再按 λ/2 换算成径向风速。
// 信号功率 = sqrt(2π)·|A·w|；噪声 = |d|·频点数；CNR = 10·log10(signal/noise)。
//
// 功率谱、径向风和 CNR 全程保留 Spec；Qt 登记由具体型号边界负责。
// ============================================================
namespace RadialWindCalculation
{
// 相干多普勒测风：径向风速 v = λ·f_D / 2，每 MHz 频移对应 λ/2 × 10⁶ m/s（λ 以米计；1550 nm → 0.775）。
[[nodiscard]] constexpr double metersPerSecondPerMHz(const double wavelengthM) noexcept
{
    return wavelengthM / 2.0 * 1e6;
}

// 两个独立量纲的静态产物；只读份额保留原通道规格。
template<DAQChannelSpec Spec>
struct Result
{
    std::shared_ptr<const RadialWind<Spec>> ws;
    std::shared_ptr<const CNR<Spec>> cnr;
};

// 每行借用原谱；波长与零风速参考统一读取 Spec，计算结果与保存的规格一致。
// 频点不足两点仍拒绝拟合；其他尺寸和轴由 Spec 保证。
template<DAQChannelSpec Spec>
[[nodiscard]] Result<Spec> Calc(const Spectrum<Spec> &spectrum)
{
    if constexpr (Spec::frequencyBinCount < 2) return {};
    const double velocityPerMHz = metersPerSecondPerMHz(Spec::wavelengthM);
    const double slope = Spec::frequencyStepMHz * velocityPerMHz;
    const double intercept = (Spec::frequencyStartMHz - Spec::zeroWindMHz) * velocityPerMHz;
    auto ws = std::make_shared<RadialWind<Spec>>();
    auto cnr = std::make_shared<CNR<Spec>>();
#pragma omp parallel for
    for (int row = 0; row < Spec::rangeGateCount; ++row)
    {
        const GaussFit::Params p = GaussFit(spectrum.row(row)).params; // 借用谱行，不复制数据
        const double signal = std::sqrt(2.0 * std::numbers::pi) * std::abs(p.A * p.w);
        const double noise = std::abs(p.d) * Spec::frequencyBinCount;
        (*ws)[row] = p.x0 * slope + intercept;
        (*cnr)[row] = noise > 1e-30 ? 10.0 * std::log10(signal / noise) : 0.0;
    }
    return {std::move(ws), std::move(cnr)};
}
} // namespace RadialWindCalculation
