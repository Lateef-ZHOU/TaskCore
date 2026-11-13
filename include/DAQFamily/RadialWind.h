#pragma once
#include "ObjectPolicy.h"
#include <cmath>
#include <memory>
#include <numbers>
#include <utility>

#include "GaussFit.h"
#include "DAQFamily/DAQChannelSpec.h"

// ============================================================
// RadialWindCalculation — 功率谱 → 逐距离门的径向风速 + CNR
//
// 一帧谱是 rows × cols 的行主序矩阵（**row = 距离门、col = 频率 bin**）：每一行是该门上的
// 一条频谱，高斯拟合出峰位 x0（bin）即多普勒频移，线性换算成径向风速；峰的能量与基线之比
// 即 CNR。
//   信号功率 = sqrt(2π)·|A·w|；噪声 = |d|·cols；CNR = 10·log10(signal/noise)。
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
    constexpr int rangeGateCount = Spec::rangeGateCount, frequencyBinCount = Spec::frequencyBinCount;
    constexpr double frequencyStartMHz = Spec::frequencyStartMHz, frequencyStepMHz = Spec::frequencyStepMHz;
    const double velocityPerMHz = metersPerSecondPerMHz(Spec::wavelengthM);
    const double slope = frequencyStepMHz * velocityPerMHz;
    const double intercept = (frequencyStartMHz - Spec::zeroWindMHz) * velocityPerMHz;
    auto ws = std::make_shared<RadialWind<Spec>>();
    auto cnr = std::make_shared<CNR<Spec>>();
#pragma omp parallel for
    for (int row = 0; row < rangeGateCount; ++row)
    {
        const GaussFit::Params p = GaussFit(spectrum.row(row)).params; // 借用谱行，不复制数据
        const double signal = std::sqrt(2.0 * std::numbers::pi) * std::abs(p.A * p.w);
        const double noise = std::abs(p.d) * frequencyBinCount;
        (*ws)[row] = p.x0 * slope + intercept;
        (*cnr)[row] = noise > 1e-30 ? 10.0 * std::log10(signal / noise) : 0.0;
    }
    return {std::move(ws), std::move(cnr)};
}
} // namespace RadialWindCalculation
