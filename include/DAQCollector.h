#pragma once
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
#include "RadialWind.h" // 标准径向风 / CNR 算法（各卡共用）
#include "RunBase.h" // Collector 契约基类

// 一次采集的不可变消费上下文；重试照抄，完成帧自带处理与保存所需信息。
struct DAQContext
{
    std::optional<WorldModel> pose;
    std::optional<QDir> saveRoot;
    QString sessionDir;
    bool sync = false;
};

// 一次采集请求的按值参数：累积时间与消费上下文。协程按卡保存它并交族口 begin，
// 由模板层经型号静态工厂造出本次 Collector；累积时间超出型号范围时由型号钳制。
struct DAQParameters
{
    int accumSec = 0;
    DAQContext context;
};

// 一个通道：型号给出规格（名称、门与频点的物理坐标、零风速参考、频移→风速系数），收集期间整理功率谱，
// 收齐后按标准方法算径向风与 CNR。显示、反演与落盘只认这些字段，不认识官邸。
struct DAQChannel
{
    QString name;
    int rows = 0;    // 距离门数
    int columns = 0; // 频点数
    QVector<double> rangeM;  // 每门斜距（m）；空 = 本帧没有可用距离，不算径向风
    QVector<double> freqMHz; // 每个频点的频率（MHz），等间距递增
    double zeroWindMHz = std::numeric_limits<double>::quiet_NaN();
    double metersPerSecondPerMHz = 0.0;
    QVector<double> spectrum;   // rows × columns 行主序（行 = 距离门）
    QVector<double> radialWind; // 每门（m/s）；零风速参考不在频带内时为 NaN
    QVector<double> cnr;        // 每门（dB）；空 = 未处理

    // 零风速参考落在本帧频带内才换算风速，否则只出 CNR。
    [[nodiscard]] bool calibrated() const noexcept
    {
        return !freqMHz.isEmpty() && zeroWindMHz >= freqMHz.front() && zeroWindMHz <= freqMHz.back();
    }

    // 标准处理：逐门高斯拟合出峰位与 CNR，峰位按频率轴与零风速参考线性换算为径向风速。
    void process()
    {
        if (rows <= 0 || columns < 2 || rangeM.size() != rows || freqMHz.size() != columns
            || spectrum.size() != qsizetype(rows) * columns)
            return;
        const double slope = (freqMHz[1] - freqMHz[0]) * metersPerSecondPerMHz;
        const double offset = calibrated() ? (freqMHz.front() - zeroWindMHz) * metersPerSecondPerMHz
                                           : std::numeric_limits<double>::quiet_NaN();
        auto result = RadialWind::Calc(spectrum, rows, columns, slope, offset);
        radialWind = std::move(result.ws);
        cnr = std::move(result.cnr);
    }
};

// 族基：通道数组即本帧全部数据（运行期定通道数）；型号负责按协议整理功率谱，收齐末包时调用 complete。
// 族基提供时刻、累积时间与消费上下文；处理在 worker 线程完成后才判齐，交付后只读。
class DAQCollector : public Collector<QByteArray>, public std::vector<DAQChannel>
{
public:
    // 当前帧的到料时刻由型号按协议确定；完成后供落盘文件名和风廓线时间轴使用。
    [[nodiscard]] RxTime frameTime() const noexcept { return frameTime_; }
    [[nodiscard]] const DAQContext &context() const noexcept { return context_; }
    [[nodiscard]] int accumSec() const noexcept { return accumSec_; }

    // 型号调用 complete 之后才算收齐；判齐后公共 feed 不再进料。
    [[nodiscard]] bool settled() const final { return processed_; }

protected:
    // 派生构造必须显式交入上下文与通道规格；确实不需要上下文时传 {}。
    DAQCollector(DAQContext context, int accumSec, std::vector<DAQChannel> channels)
        : std::vector<DAQChannel>(std::move(channels)), accumSec_(accumSec), context_(std::move(context)) {}
    // 非虚 + protected：与 Collector / GMBCollector 同判例——销毁恒经控制块记下的具体删除器，
    // 从不经本类指针 delete；protected 顺带把裸删门封死。
    ~DAQCollector() = default;

    // [worker] 型号在 onFeed 收齐末包时调用一次：按通道串行做标准处理（算法内部 OpenMP），之后本帧判齐。
    void complete()
    {
        for (auto &channel : *this)
            channel.process();
        processed_ = true;
    }

    // 仅型号 onFeed 写：未完成时可随协议换帧更新，完成后公共 feed 不再改写。
    // makeSelf 只保留不可变配置，另建初始帧时刻，不继承旧尝试的观测。
    RxTime frameTime_{};

private:
    const int accumSec_;
    const DAQContext context_;
    bool processed_ = false;
};

// 具体类型须属于本族、沿用 feed；静态工厂按请求参数交出完整、可写的自身（已备齐收集区），makeSelf 供重试。
template <typename C>
concept DAQCollectorLike = std::derived_from<C, DAQCollector> && CollectorLike<C>
    && requires(const DAQParameters &parameters) {
        { C::create(parameters) } -> std::same_as<std::shared_ptr<C>>;
    };
