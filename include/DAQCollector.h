#pragma once
#include <concepts>
#include <memory>
#include <optional>
#include <tuple>
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

// 一个通道：静态规格（名称、波长、零风速参考）+ 三个产物。显示、反演与落盘只认这些，不认识官邸。
// 功率谱：行 = 距离门（行轴为斜距 m），列 = 频点（列轴为频率 MHz）；径向风与 CNR 的轴取功率谱行轴。
template <typename C>
concept DAQChannelLike = requires(const C c) {
    { C::name } -> std::convertible_to<const char *>;
    { C::wavelengthM } -> std::convertible_to<double>;
    { C::zeroWindMHz } -> std::convertible_to<double>;
    { c.spectrum } -> std::same_as<const LinearArray2D<double> &>;
    { c.radialWind } -> std::same_as<const LinearArray<double> &>;
    { c.cnr } -> std::same_as<const LinearArray<double> &>;
};

// 标准通道：继承规格的静态常量；型号只交本帧数值（行优先，rows × columns 个），两条轴取自规格，
// 构造即按标准方法算出径向风与 CNR，存在即已处理。交付后经 const 收集器只读。
template <DAQChannelSpec Spec>
struct DAQChannel : Spec
{
    explicit DAQChannel(std::vector<double> values)
        : DAQChannel(LinearArray2D<double>(Spec::rows, Spec::columns, Spec::rangeStartM, Spec::rangeStepM,
                                           Spec::frequencyStartMHz, Spec::frequencyStepMHz, std::move(values))) {}

    LinearArray2D<double> spectrum;
    LinearArray<double> radialWind; // m/s；网格不完整时为空
    LinearArray<double> cnr;        // dB

private:
    explicit DAQChannel(LinearArray2D<double> &&spectrum)
        : DAQChannel(RadialWind::Calc(spectrum, RadialWind::metersPerSecondPerMHz(Spec::wavelengthM), Spec::zeroWindMHz),
                     std::move(spectrum)) {}
    // 功率谱按右值引用收：拟合结果先于功率谱移入成员算出，不依赖实参求值顺序。
    DAQChannel(RadialWind::Result wind, LinearArray2D<double> &&spectrum)
        : spectrum(std::move(spectrum)), radialWind(std::move(wind.ws)), cnr(std::move(wind.cnr)) {}
};

// 族基：等待器、任务与完成回调只认它（参数、帧时刻、是否判齐），不读通道；交付后只读。
class DAQCollector : public Collector<QByteArray, DAQParameters>
{
public:
    // 当前帧的到料时刻由型号按协议确定；完成后供落盘文件名和风廓线时间轴使用。
    [[nodiscard]] RxTime frameTime() const noexcept { return frameTime_; }

protected:
    // 派生构造必须显式交入请求参数；参数已由官邸 paramsFor 校验。
    explicit DAQCollector(DAQParameters parameters) : Collector(std::move(parameters)) {}
    // 非虚 + protected：与 Collector / GMBCollector 同判例——销毁恒经控制块记下的具体删除器，
    // 从不经本类指针 delete；protected 顺带把裸删门封死。
    ~DAQCollector() = default;

    // 仅型号 assemble 写：未完成时可随协议换帧更新，完成后公共 feed 不再改写。
    // 看门狗重试以同一参数另造新斗，帧时刻从初始值重来，不继承旧尝试的观测。
    RxTime frameTime_{};
};

// 模板层：一帧的通道个数与各路类型在编译期固定，按顺序存成元组。每口料原样交型号 assemble；型号自己维护
// 原始帧缓冲与凑帧状态，收包期间只做协议校验、去掉不属于数组元素的包头并放置字节，凑齐那一口解析出各路通道。
template <DAQChannelLike... Channels>
class TypedDAQCollector : public DAQCollector
{
public:
    using Frame = std::tuple<Channels...>;
    // 本帧全部通道；仅判齐后可读。
    [[nodiscard]] const Frame &channels() const noexcept { return *frame_; }
    // 判齐 = 型号报告本帧已凑齐（返回了通道）；判齐后公共 feed 不再进料。
    [[nodiscard]] bool settled() const final { return frame_.has_value(); }

protected:
    using DAQCollector::DAQCollector;
    ~TypedDAQCollector() = default;

private:
    void onFeed(const QByteArray &item, RxTime at) final { frame_ = assemble(item, at); }

    // [worker] 型号凑帧：校验并放置这一口字节；未凑齐返回空，凑齐的那一口按协议解析为各路通道（构造即处理）。
    [[nodiscard]] virtual std::optional<Frame> assemble(QByteArrayView bytes, RxTime at) = 0;

    std::optional<Frame> frame_;
};

// 具体类型须属于本族、沿用 feed；公开构造函数按已校验的请求参数造出完整、可写的自身（首次启动与看门狗重试共用）。
template <typename C>
concept DAQCollectorLike = std::derived_from<C, DAQCollector> && CollectorLike<C>
    && std::constructible_from<C, const DAQParameters &>
    && requires(const C c) { { c.channels() } -> std::same_as<const typename C::Frame &>; };
