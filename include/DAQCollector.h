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

// 采集参数的通行证：构造与析构受保护，族外造不出来。DAQFamily 以 protected 方式继承它，
// 官邸 paramsFor 以 *this 出示；参数构造必须交入通行证，于是只有型号校验过的参数才存在。
class DAQParamsKey
{
protected:
    DAQParamsKey() = default;
    ~DAQParamsKey() = default;
};

// 一次采集请求的按值参数：累积时间与消费上下文（世界朝向、落盘根、会话目录、同步位）。
// 只能由官邸 paramsFor 校验后构造，存在即有效；字段只读。协程按卡保存它并交族口 begin，
// 模板层据此直接构造本次 Collector；重试照抄，完成帧自带处理与保存所需信息。
struct DAQParameters
{
    DAQParameters(const DAQParamsKey &, const int accumSec, std::optional<WorldModel> pose, std::optional<QDir> saveRoot,
                  QString sessionDir, const bool sync)
        : accumSec(accumSec), pose(std::move(pose)), saveRoot(std::move(saveRoot)), sessionDir(std::move(sessionDir)),
          sync(sync) {}

    const int accumSec;
    const std::optional<WorldModel> pose;
    const std::optional<QDir> saveRoot; // 空 = 不落盘
    const QString sessionDir;
    const bool sync;
};

// 一个通道：显示、反演与落盘只认这里的接口与字段，不认识官邸；交付后经 const 收集器只读。
// 名称、零风速参考、激光波长与功率谱两条轴都是该路的固定规格，由型号以静态常量给出（见 TypedDAQChannel），
// 本层只按虚函数读取，不允许运行时另给。
class DAQChannel
{
public:
    virtual ~DAQChannel() = default;
    DAQChannel(const DAQChannel &) = delete;
    DAQChannel &operator=(const DAQChannel &) = delete;

    [[nodiscard]] virtual QString name() const = 0;
    [[nodiscard]] virtual double zeroWindMHz() const = 0;
    [[nodiscard]] virtual double wavelengthM() const = 0;

    // 功率谱：行 = 距离门（行轴为斜距 m），列 = 频点（列轴为频率 MHz）；两条轴取自该路规格。
    const LinearArray2D<double> spectrum;
    // 逐门产物：轴取功率谱的行轴（斜距）；网格不完整时为空。
    const LinearArray<double> radialWind; // m/s
    const LinearArray<double> cnr;        // dB

protected:
    // 功率谱按右值引用收：拟合结果先于功率谱移入成员算出，不依赖实参求值顺序。
    DAQChannel(RadialWind::Result wind, LinearArray2D<double> &&spectrum)
        : spectrum(std::move(spectrum)), radialWind(std::move(wind.ws)), cnr(std::move(wind.cnr)) {}
};

// 通道规格：型号以静态常量写死一路的名称、波长、零风速参考与两条轴。
template <typename Spec>
concept DAQChannelSpec = requires {
    { Spec::name } -> std::convertible_to<const char *>;
    { Spec::wavelengthM } -> std::convertible_to<double>;
    { Spec::zeroWindMHz } -> std::convertible_to<double>;
    { Spec::rows } -> std::convertible_to<int>;
    { Spec::columns } -> std::convertible_to<int>;
    { Spec::rangeStartM } -> std::convertible_to<double>;
    { Spec::rangeStepM } -> std::convertible_to<double>;
    { Spec::frequencyStartMHz } -> std::convertible_to<double>;
    { Spec::frequencyStepMHz } -> std::convertible_to<double>;
};

// 一路的具体通道：型号只交本帧数值（行优先，rows × columns 个），轴与系数全部取自 Spec；
// 构造即按标准方法算出径向风与 CNR，存在即已处理。
template <DAQChannelSpec Spec>
class TypedDAQChannel final : public DAQChannel
{
public:
    explicit TypedDAQChannel(std::vector<double> values)
        : TypedDAQChannel(LinearArray2D<double>(Spec::rows, Spec::columns, Spec::rangeStartM, Spec::rangeStepM,
                                                Spec::frequencyStartMHz, Spec::frequencyStepMHz, std::move(values))) {}

    [[nodiscard]] QString name() const override { return QString::fromUtf8(Spec::name); }
    [[nodiscard]] double zeroWindMHz() const override { return Spec::zeroWindMHz; }
    [[nodiscard]] double wavelengthM() const override { return Spec::wavelengthM; }

private:
    explicit TypedDAQChannel(LinearArray2D<double> &&spectrum)
        : DAQChannel(RadialWind::Calc(spectrum, RadialWind::metersPerSecondPerMHz(Spec::wavelengthM), Spec::zeroWindMHz),
                     std::move(spectrum)) {}
};

// 一帧的全部通道。
using DAQChannels = std::vector<std::shared_ptr<const DAQChannel>>;

// 族基：每口料原样交型号 assemble；型号自己维护原始帧缓冲与凑帧状态，凑齐那一口解析出通道并返回。
// 型号收包期间只做协议校验、去掉不属于数组元素的包头并放置字节，不做换算；换算在凑齐后一次完成。
// 族基提供时刻与本次请求参数；交付后只读。
class DAQCollector : public Collector<QByteArray, DAQParameters>
{
public:
    // 当前帧的到料时刻由型号按协议确定；完成后供落盘文件名和风廓线时间轴使用。
    [[nodiscard]] RxTime frameTime() const noexcept { return frameTime_; }
    // 本帧全部通道；仅判齐后可读。
    [[nodiscard]] const DAQChannels &channels() const noexcept { return *channels_; }
    // 判齐 = 型号报告本帧已凑齐（返回了通道）；判齐后公共 feed 不再进料。
    [[nodiscard]] bool settled() const final { return channels_.has_value(); }

protected:
    // 派生构造必须显式交入请求参数；参数已由官邸 paramsFor 校验。
    explicit DAQCollector(DAQParameters parameters) : Collector(std::move(parameters)) {}
    // 非虚 + protected：与 Collector / GMBCollector 同判例——销毁恒经控制块记下的具体删除器，
    // 从不经本类指针 delete；protected 顺带把裸删门封死。
    ~DAQCollector() = default;

    // 仅型号 assemble 写：未完成时可随协议换帧更新，完成后公共 feed 不再改写。
    // 看门狗重试以同一参数另造新斗，帧时刻从初始值重来，不继承旧尝试的观测。
    RxTime frameTime_{};

private:
    void onFeed(const QByteArray &item, RxTime at) final { channels_ = assemble(item, at); }

    // [worker] 型号凑帧：校验并放置这一口字节；未凑齐返回空，凑齐的那一口按协议解析为各通道（构造即处理）。
    [[nodiscard]] virtual std::optional<DAQChannels> assemble(QByteArrayView bytes, RxTime at) = 0;

    std::optional<DAQChannels> channels_;
};

// 具体类型须属于本族、沿用 feed；公开构造函数按已校验的请求参数造出完整、可写的自身（首次启动与看门狗重试共用）。
template <typename C>
concept DAQCollectorLike = std::derived_from<C, DAQCollector> && CollectorLike<C>
    && std::constructible_from<C, const DAQParameters &>;
