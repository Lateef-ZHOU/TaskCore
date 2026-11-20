#pragma once
#include <concepts>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <utility>

#include "GMBFamily/GimbalModel.h"
#include "DAQFamily/RadialWind.h" // 标准径向风 / CNR 算法（各卡共用）
#include "DataArrays.h"
#include "AxisMapping.h" // 算法层的静态轴契约
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
    DAQParameters(const DAQParamsKey &, const int accumSec, std::optional<WorldModel> pose, std::optional<std::filesystem::path> saveRoot,
                  std::string sessionDir, const bool sync)
        : accumSec(accumSec), pose(std::move(pose)), saveRoot(std::move(saveRoot)), sessionDir(std::move(sessionDir)),
          sync(sync) {}

    const int accumSec;
    const std::optional<WorldModel> pose;
    const std::optional<std::filesystem::path> saveRoot; // nullopt = 不落盘
    const std::string sessionDir; // UTF-8
    const bool sync;
};

// 通道只持原帧的只读份额，轴规格不在帧中重复存储。型号直接填满最终谱数组，
// 交入唯一份额后计算径向风与 CNR；通道本身只复制共享指针，不复制数据。
template<MappingLike Range, MappingLike Frequency>
struct DAQChannel
{
    // 型号提供静态寿命的 UTF-8 通道名，供异步保存使用。
    const char *name = nullptr;

    // 前置条件：交入非空、已完整填写的谱；协议完整性由型号 assemble 负责。
    // 风速由型号交入的「下标 → 风速」映射换算；族基不认识波长与零风速参考。
    template<MappingLike Wind>
    explicit DAQChannel(std::unique_ptr<Grid2D<Range, Frequency>> values, Wind wind, const char* channelName = nullptr)
        : DAQChannel(RadialWindCalculation::Calc(*values, wind), values, channelName) {}

    std::shared_ptr<const Grid2D<Range, Frequency>> spectrum;
    std::shared_ptr<const Array1D<Range>> radialWind;
    std::shared_ptr<const Array1D<Range>> cnr;
private:
    // 以引用交接唯一份额：拟合结束后才移入成员，不依赖实参求值顺序。
    DAQChannel(RadialWindCalculation::Result<Range> wind, std::unique_ptr<Grid2D<Range, Frequency>> &values, const char* channelName)
        : name(channelName), spectrum(std::move(values)), radialWind(std::move(wind.ws)), cnr(std::move(wind.cnr)) {}
};

template<class> inline constexpr bool isDAQChannel = false;
template<class Range, class Frequency> inline constexpr bool isDAQChannel<DAQChannel<Range, Frequency>> = true;
template<class C> concept DAQChannelLike = isDAQChannel<C>;

// 族基：等待器、任务与完成回调只认它（参数、帧时刻、是否判齐），不读通道；交付后只读。
class DAQCollector : public Collector<std::span<const unsigned char>, DAQParameters>
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
    void onFeed(const Feed &item, RxTime at) final { frame_ = assemble(item, at); }

    // [worker] 仅在本次同步调用内借用输入。型号若需跨包留存，须写入自己的收帧缓冲。
    // 未凑齐返回空，凑齐的那一口按协议解析为各路通道。
    [[nodiscard]] virtual std::optional<Frame> assemble(std::span<const unsigned char> bytes, RxTime at) = 0;

    std::optional<Frame> frame_;
};

// 具体类型须属于本族、沿用 feed；公开构造函数按已校验的请求参数造出完整、可写的自身（首次启动与看门狗重试共用）。
template <typename C>
concept DAQCollectorLike = std::derived_from<C, DAQCollector> && CollectorLike<C>
    && std::constructible_from<C, const DAQParameters &>
    && requires(const C c) { { c.channels() } -> std::same_as<const typename C::Frame &>; };
