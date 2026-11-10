#pragma once
#include <algorithm>
#include <concepts>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <QByteArray>
#include <QDir>
#include <QString>
#include <QVector>

#include "GimbalModel.h"
#include "LinearArray.h"
#include "RadialWind.h" // 标准径向风 / CNR 算法（各卡共用）
#include "RunBase.h" // Collector 契约基类

// 一次采集请求的按值参数：累积时间与消费上下文（世界朝向、落盘根、会话目录、同步位）。
// 官邸 paramsFor 负责型号校验；协程按卡保存它并交族口 begin，模板层据此造出本次 Collector。
// 重试照抄，完成帧自带处理与保存所需信息。
struct DAQParameters
{
    int accumSec = 0;
    std::optional<WorldModel> pose;
    std::optional<QDir> saveRoot; // 空 = 不落盘
    QString sessionDir;
    bool sync = false;
};

// 一个通道：型号给出规格（名称、功率谱网格与两条物理轴、零风速参考），收集期间往 spectrum 里整理功率谱，
// 收齐后按标准方法算径向风与 CNR。显示、反演与落盘只认这些字段，不认识官邸。
struct DAQChannel
{
    QString name;
    // 功率谱：行 = 距离门（行轴为斜距 m），列 = 频点（列轴为频率 MHz）。型号在规格里给出网格与两条轴，
    // 收包期间只追加元素；频率起点随协议在首包确定的型号可在那时再写列轴。
    LinearArray2D<double> spectrum;
    double zeroWindMHz = std::numeric_limits<double>::quiet_NaN();
    // 逐门产物：轴取功率谱的行轴（斜距），由处理一并给出。
    LinearArray<double> radialWind; // m/s；零风速参考不在频带内时为 NaN
    LinearArray<double> cnr;        // dB；空 = 未处理

    // 已处理：径向风与 CNR 都已按功率谱的门数算出。
    [[nodiscard]] bool processed() const noexcept
    {
        const auto rows = static_cast<std::size_t>(spectrum.rows);
        return rows > 0 && radialWind.size() == rows && cnr.size() == rows;
    }

    // 标准处理（幂等，已处理即返回）：逐门高斯拟合出峰位与 CNR，峰位按频率轴与零风速参考换算为径向风速。
    // metersPerSecondPerMHz 由族基按本卡激光波长给出（λ/2）；网格不完整或轴无效时不出结果。
    void process(const double metersPerSecondPerMHz)
    {
        if (processed())
            return;
        auto result = RadialWind::Calc(spectrum, metersPerSecondPerMHz, zeroWindMHz);
        radialWind = std::move(result.ws);
        cnr = std::move(result.cnr);
    }
};

// 族基：通道数组即本帧全部数据（运行期定通道数）；型号负责按协议整理功率谱，收齐末包时调用 complete。
// 族基提供时刻、本次请求参数与本卡激光波长；处理在 worker 线程完成后才判齐，交付后只读。
class DAQCollector : public Collector<QByteArray>, public std::vector<DAQChannel>
{
public:
    // 当前帧的到料时刻由型号按协议确定；完成后供落盘文件名和风廓线时间轴使用。
    [[nodiscard]] RxTime frameTime() const noexcept { return frameTime_; }
    [[nodiscard]] const DAQParameters &parameters() const noexcept { return parameters_; }
    // 本卡激光波长（m）：各通道共用，决定频移→风速系数 λ/2。
    [[nodiscard]] double wavelengthM() const noexcept { return wavelengthM_; }

    // 型号调用 complete 之后才算收齐；判齐后公共 feed 不再进料。
    // 判齐 = 每个通道都已处理（由各通道的产物长度判断，不另存状态位）。
    [[nodiscard]] bool settled() const final
    {
        return !empty() && std::ranges::all_of(*this, [](const DAQChannel &channel) { return channel.processed(); });
    }

protected:
    // 派生构造必须显式交入请求参数、通道规格与激光波长（m）；参数已由官邸 paramsFor 校验。
    DAQCollector(DAQParameters parameters, std::vector<DAQChannel> channels, const double wavelengthM)
        : std::vector<DAQChannel>(std::move(channels)), parameters_(std::move(parameters)), wavelengthM_(wavelengthM) {}
    // 非虚 + protected：与 Collector / GMBCollector 同判例——销毁恒经控制块记下的具体删除器，
    // 从不经本类指针 delete；protected 顺带把裸删门封死。
    ~DAQCollector() = default;

    // [worker] 型号在 onFeed 收齐末包时调用一次：按通道串行做标准处理（算法内部 OpenMP），之后本帧判齐。
    void complete()
    {
        const double metersPerSecondPerMHz = RadialWind::metersPerSecondPerMHz(wavelengthM_);
        for (auto &channel : *this)
            channel.process(metersPerSecondPerMHz);
    }

    // 仅型号 onFeed 写：未完成时可随协议换帧更新，完成后公共 feed 不再改写。
    // makeSelf 只保留不可变配置，另建初始帧时刻，不继承旧尝试的观测。
    RxTime frameTime_{};

private:
    const DAQParameters parameters_;
    const double wavelengthM_;
};

// 具体类型须属于本族、沿用 feed；静态工厂按请求参数交出完整、可写的自身（已备齐收集区），makeSelf 供重试。
template <typename C>
concept DAQCollectorLike = std::derived_from<C, DAQCollector> && CollectorLike<C>
    && requires(const DAQParameters &parameters) {
        { C::create(parameters) } -> std::same_as<std::shared_ptr<C>>;
    };
