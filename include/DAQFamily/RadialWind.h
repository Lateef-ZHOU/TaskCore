#pragma once
#include "ObjectPolicy.h"
#include <cmath>
#include <numbers>
#include <vector>

#include "GaussFit.h"
#include "LinearArray.h"
#include "DAQFamily/DAQChannelSpec.h"

// ============================================================
// RadialWindCalculation — 功率谱 → 逐距离门的径向风速 + CNR
//
// 一帧谱是 rows × cols 的行主序矩阵（**row = 距离门、col = 频率 bin**）：每一行是该门上的
// 一条频谱，高斯拟合出峰位 x0（bin）即多普勒频移，线性换算成径向风速；峰的能量与基线之比
// 即 CNR。
//   信号功率 = sqrt(2π)·|A·w|；噪声 = |d|·cols；CNR = 10·log10(signal/noise)。
//
// 静态谱使用 Spectrum<Spec>；现有二维传输谱保留明确类型的入口，Qt 登记由调用边界负责。
// 径向风产物暂保留 LinearArray，后续单独整理。
// ============================================================
namespace RadialWindCalculation
{
// 相干多普勒测风：径向风速 v = λ·f_D / 2，每 MHz 频移对应 λ/2 × 10⁶ m/s（λ 以米计；1550 nm → 0.775）。
[[nodiscard]] constexpr double metersPerSecondPerMHz(const double wavelengthM) noexcept
{
    return wavelengthM / 2.0 * 1e6;
}

// 一帧的两列产物：轴取功率谱的行轴（斜距）。
struct Result
{
    LinearSeriesPtr ws = std::make_shared<const LinearArray<double>>();  ///< 每门径向风速（m/s）
    LinearSeriesPtr cnr = std::make_shared<const LinearArray<double>>(); ///< 每门 CNR（dB）
};

// spectrum 行 = 距离门、列 = 频点（列轴单位 MHz）。峰位按列轴换成频率，减零风速参考后乘 λ/2 系数得风速。
// 静态谱在概念中约束轴；动态谱入口拒绝不完整网格或无效轴，频点不足两点时均返回空数组。
// 两个明确类型的公开入口共用计算实现，不开放任意容器读取契约。
namespace detail
{
template<class S>
[[nodiscard]] Result
calculate(const S &spectrum, const int rangeGateCount, const int frequencyBinCount,
          const double rangeStartM, const double rangeStepM,
          const double frequencyStartMHz, const double frequencyStepMHz,
          const double metersPerSecondPerMHz, const double zeroWindMHz)
{
    if (frequencyBinCount < 2)
        return {};
    const double slope = frequencyStepMHz * metersPerSecondPerMHz;
    const double intercept = (frequencyStartMHz - zeroWindMHz) * metersPerSecondPerMHz;
    auto ws = std::make_shared<LinearArray<double>>(rangeStartM, rangeStepM, std::vector<double>(rangeGateCount));
    auto cnr = std::make_shared<LinearArray<double>>(rangeStartM, rangeStepM, std::vector<double>(rangeGateCount));
    double *const wsOut = ws->data(); // 循环前取写指针，omp 体内只按下标写
    double *const cnrOut = cnr->data();
#pragma omp parallel for
    for (int row = 0; row < rangeGateCount; ++row)
    {
        const GaussFit::Params p = GaussFit(spectrum.row(row)).params; // 借用谱行，不复制数据
        const double signal = std::sqrt(2.0 * std::numbers::pi) * std::abs(p.A * p.w);
        const double noise = std::abs(p.d) * frequencyBinCount;
        wsOut[row] = p.x0 * slope + intercept;
        cnrOut[row] = noise > 1e-30 ? 10.0 * std::log10(signal / noise) : 0.0;
    }
    return {std::move(ws), std::move(cnr)};
}
} // namespace detail

// 静态功率谱默认使用通道规格的波长及零风速参考；调用方仍可显式给出换算参数。
template<DAQChannelSpec Spec>
[[nodiscard]] Result Calc(const Spectrum<Spec> &spectrum,
                          const double metersPerSecondPerMHz = RadialWindCalculation::metersPerSecondPerMHz(Spec::wavelengthM),
                          const double zeroWindMHz = Spec::zeroWindMHz)
{
    return detail::calculate(spectrum, Spec::rangeGateCount, Spec::frequencyBinCount,
                             Spec::rangeStartM, Spec::rangeStepM, Spec::frequencyStartMHz, Spec::frequencyStepMHz,
                             metersPerSecondPerMHz, zeroWindMHz);
}

// 现有采集、绘图和保存边界仍传输二维谱；转换参数由官邸的通道规格给出。
[[nodiscard]] inline Result Calc(const LinearArray2D<double> &spectrum,
                                 const double metersPerSecondPerMHz, const double zeroWindMHz)
{
    if (!spectrum.complete() || !spectrum.hasValidAxes())
        return {};
    return detail::calculate(spectrum, spectrum.rows, spectrum.columns,
                             spectrum.rowStart, spectrum.rowStep, spectrum.columnStart, spectrum.columnStep,
                             metersPerSecondPerMHz, zeroWindMHz);
}
} // namespace RadialWindCalculation

static_assert(ValueObject<RadialWindCalculation::Result>);
