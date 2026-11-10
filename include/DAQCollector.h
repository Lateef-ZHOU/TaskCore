#pragma once
#include <concepts>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <QByteArray>
#include <QByteArrayView>
#include <QDir>
#include <QString>

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

// 一个通道：型号解析出完整功率谱后构造，构造即按标准方法算出径向风与 CNR；存在即已处理。
// 显示、反演与落盘只认这些字段，不认识官邸；交付后经 const 收集器只读。
struct DAQChannel
{
    // 功率谱：行 = 距离门（行轴为斜距 m），列 = 频点（列轴为频率 MHz）。
    // 逐门高斯拟合出峰位与 CNR，峰位按频率轴与零风速参考换算为径向风速；wavelengthM 决定频移→风速系数 λ/2。
    DAQChannel(QString name, LinearArray2D<double> spectrum, const double zeroWindMHz, const double wavelengthM)
        : DAQChannel(std::move(name), RadialWind::Calc(spectrum, RadialWind::metersPerSecondPerMHz(wavelengthM), zeroWindMHz),
                     std::move(spectrum), zeroWindMHz) {}

    QString name;
    LinearArray2D<double> spectrum;
    double zeroWindMHz;
    // 逐门产物：轴取功率谱的行轴（斜距）；网格不完整或轴无效时为空。
    LinearArray<double> radialWind; // m/s；零风速参考不在频带内时为 NaN
    LinearArray<double> cnr;        // dB

private:
    // 功率谱按右值引用收：拟合结果先于功率谱移入成员算出，不依赖实参求值顺序。
    DAQChannel(QString name, RadialWind::Result wind, LinearArray2D<double> &&spectrum, const double zeroWindMHz)
        : name(std::move(name)), spectrum(std::move(spectrum)), zeroWindMHz(zeroWindMHz),
          radialWind(std::move(wind.ws)), cnr(std::move(wind.cnr)) {}
};

// 族基：每口料原样交型号 assemble；型号自己维护原始帧缓冲与凑帧状态，凑齐那一口解析出通道并返回。
// 型号收包期间只做协议校验、去掉不属于数组元素的包头并放置字节，不做换算；换算在凑齐后一次完成。
// 族基提供时刻、本次请求参数与本卡激光波长；交付后只读。
class DAQCollector : public Collector<QByteArray>
{
public:
    // 当前帧的到料时刻由型号按协议确定；完成后供落盘文件名和风廓线时间轴使用。
    [[nodiscard]] RxTime frameTime() const noexcept { return frameTime_; }
    [[nodiscard]] const DAQParameters &parameters() const noexcept { return parameters_; }
    // 本卡激光波长（m）：各通道共用，决定频移→风速系数 λ/2。
    [[nodiscard]] double wavelengthM() const noexcept { return wavelengthM_; }
    // 本帧全部通道；仅判齐后可读。
    [[nodiscard]] const std::vector<DAQChannel> &channels() const noexcept { return *channels_; }
    // 判齐 = 型号报告本帧已凑齐（返回了通道）；判齐后公共 feed 不再进料。
    [[nodiscard]] bool settled() const final { return channels_.has_value(); }

protected:
    // 派生构造必须显式交入请求参数与激光波长（m）；参数已由官邸 paramsFor 校验。
    DAQCollector(DAQParameters parameters, const double wavelengthM)
        : parameters_(std::move(parameters)), wavelengthM_(wavelengthM) {}
    // 非虚 + protected：与 Collector / GMBCollector 同判例——销毁恒经控制块记下的具体删除器，
    // 从不经本类指针 delete；protected 顺带把裸删门封死。
    ~DAQCollector() = default;

    // 仅型号 assemble 写：未完成时可随协议换帧更新，完成后公共 feed 不再改写。
    // makeSelf 只保留不可变配置，另建初始帧时刻，不继承旧尝试的观测。
    RxTime frameTime_{};

private:
    void onFeed(const std::optional<QByteArray> &item, RxTime at) final { channels_ = assemble(*item, at); }

    // [worker] 型号凑帧：校验并放置这一口字节；未凑齐返回空，凑齐的那一口按协议解析为各通道（构造即处理）。
    [[nodiscard]] virtual std::optional<std::vector<DAQChannel>> assemble(QByteArrayView bytes, RxTime at) = 0;

    const DAQParameters parameters_;
    const double wavelengthM_;
    std::optional<std::vector<DAQChannel>> channels_;
};

// 具体类型须属于本族、沿用 feed；静态工厂按请求参数交出完整、可写的自身（已备齐收集区），makeSelf 供重试。
template <typename C>
concept DAQCollectorLike = std::derived_from<C, DAQCollector> && CollectorLike<C>
    && requires(const DAQParameters &parameters) {
        { C::create(parameters) } -> std::same_as<std::shared_ptr<C>>;
    };
