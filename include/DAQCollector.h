#pragma once
#include <concepts>
#include <cstring>
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

// 族基：收包期间只把一帧原始字节凑进缓冲，凑齐即交型号 decode 解析成通道，之后本帧判齐。
// 型号 onFeed 只做协议校验、去掉包头（帧内数组元素原样保留）并以 store 放置字节，不做换算。
// 族基提供时刻、本次请求参数与本卡激光波长；交付后只读。
class DAQCollector : public Collector<QByteArray>
{
public:
    // 当前帧的到料时刻由型号按协议确定；完成后供落盘文件名和风廓线时间轴使用。
    [[nodiscard]] RxTime frameTime() const noexcept { return frameTime_; }
    [[nodiscard]] const DAQParameters &parameters() const noexcept { return parameters_; }
    // 本卡激光波长（m）：各通道共用，决定频移→风速系数 λ/2。
    [[nodiscard]] double wavelengthM() const noexcept { return wavelengthM_; }
    // 本帧全部通道；判齐前为空。
    [[nodiscard]] const std::vector<DAQChannel> &channels() const noexcept { return channels_; }
    // 判齐 = 一帧凑齐且已解析出通道；判齐后公共 feed 不再进料。
    [[nodiscard]] bool settled() const final { return !channels_.empty(); }

protected:
    // 派生构造必须显式交入请求参数、整帧字节数与激光波长（m）；参数已由官邸 paramsFor 校验。
    DAQCollector(DAQParameters parameters, const qsizetype frameBytes, const double wavelengthM)
        : parameters_(std::move(parameters)), wavelengthM_(wavelengthM), raw_(frameBytes, Qt::Uninitialized) {}
    // 非虚 + protected：与 Collector / GMBCollector 同判例——销毁恒经控制块记下的具体删除器，
    // 从不经本类指针 delete；protected 顺带把裸删门封死。
    ~DAQCollector() = default;

    // [worker] 把一段字节放到帧内 offset 处；越界（含已解析后）拒收并返回 false。
    // 累计字节凑满整帧即调用 decode，随后释放原始缓冲（算法内部 OpenMP）。
    bool store(const qsizetype offset, const QByteArrayView bytes)
    {
        if (offset < 0 || bytes.size() > raw_.size() - offset)
            return false;
        std::memcpy(raw_.data() + offset, bytes.data(), static_cast<std::size_t>(bytes.size()));
        received_ += bytes.size();
        if (received_ == raw_.size())
        {
            channels_ = decode(raw_);
            raw_ = QByteArray();
        }
        return true;
    }
    // [worker] 已收字节数：顺序到达的协议以它作为下一段的偏移。
    [[nodiscard]] qsizetype received() const noexcept { return received_; }
    // [worker] 协议换帧时弃旧启新：只清计数，不清缓冲——凑满即每个位置都被本帧写过。
    void restart() noexcept { received_ = 0; }

    // 仅型号 onFeed 写：未完成时可随协议换帧更新，完成后公共 feed 不再改写。
    // makeSelf 只保留不可变配置，另建初始帧时刻，不继承旧尝试的观测。
    RxTime frameTime_{};

private:
    // [worker] 型号按协议把凑齐的一帧解析为各通道（构造即处理）；解析失败返回空，本帧不判齐。
    [[nodiscard]] virtual std::vector<DAQChannel> decode(const QByteArray &frame) const = 0;

    const DAQParameters parameters_;
    const double wavelengthM_;
    QByteArray raw_;
    qsizetype received_ = 0;
    std::vector<DAQChannel> channels_;
};

// 具体类型须属于本族、沿用 feed；静态工厂按请求参数交出完整、可写的自身（已备齐收集区），makeSelf 供重试。
template <typename C>
concept DAQCollectorLike = std::derived_from<C, DAQCollector> && CollectorLike<C>
    && requires(const DAQParameters &parameters) {
        { C::create(parameters) } -> std::same_as<std::shared_ptr<C>>;
    };
