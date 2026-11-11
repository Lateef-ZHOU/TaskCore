#pragma once
#include <algorithm>
#include <chrono>
#include <concepts>
#include <memory>
#include <optional>
#include <utility>
#include <QByteArray>
#include <QList>


#include "GimbalModel.h" // PanTiltModel（目标 / 实测位形；distance）
#include "RunBase.h" // Collector<FeedT> 契约基类

// 一次运动请求的按值参数：云台位形目标、停稳等待及可选两轴速度（无速度表示本型号定位协议不携带速度）。
// 协程按束保存它并交族口 begin，由模板层据此造出本次 Collector；官邸 paramsFor 负责型号校验与量化。
// QML 仅在命令/任务启动边界交一份值，协程与工厂不回读界面。
// 运动参数的通行证：构造与析构受保护，族外造不出来。GMBFamily 以 protected 方式继承它，
// 官邸 paramsFor 以 *this 出示；参数构造必须交入通行证，于是只有型号校验过的参数才存在。
class GMBParamsKey
{
protected:
    GMBParamsKey() = default;
    ~GMBParamsKey() = default;
};

// 一次到位请求：目标位形、停稳等待与（型号量化后的）速度。只能由官邸 paramsFor 校验后构造，存在即有效；字段只读。
struct GMBParameters
{
    struct Speeds { double pan; double tilt; }; // °/s，两轴必须成组提供

    GMBParameters(const GMBParamsKey &, PanTiltModel target, const std::chrono::milliseconds hold,
                      const std::optional<Speeds> speeds)
        : target(std::move(target)), hold(hold), speeds(speeds) {}

    const PanTiltModel target;
    const std::chrono::milliseconds hold;
    const std::optional<Speeds> speeds;
};

// 云台族公共到位/驻留判据与位形读口；型号以 TypedGMBCollector 的标签区分。
// 指示区不可变，收集区只由收包线程写，完成后按 const 份额交付。
// 对象不可复制或移动的约束由 Collector 基类统一提供，调用方只共享指针份额。
class GMBCollector : public Collector<PanTiltModel, GMBParameters>
{
public:
    static constexpr std::chrono::milliseconds kMinHold{2000}; // 驻留下限：任务页的输入框同此下限，斗再兜一道

    // 型号收集器交齐一次尝试的不可变配置：已校验的运动参数、容差（°）、启动 / 停机指令和看门狗时限
    // （ms，非正 = 不设看门狗）。hold 对应任务页的“停稳等待”，不足 kMinHold 时取下限。进圈后开始计时，
    // 出圈即清零重新计时；须连续在圈内（样本首末时刻相差达到 hold）才完成。
protected:
    GMBCollector(const GMBParameters &parameters, const double tolerance, QList<QByteArray> start,
                 QList<QByteArray> stop, const int timeoutMs)
        : Collector(parameters), tolerance_(tolerance), hold_(std::max(parameters.hold, kMinHold)),
          start_(std::move(start)), stop_(std::move(stop)), timeoutMs_(timeoutMs)
    {
    }
    // 族基不拥有对象，销毁由具体类型的 shared_ptr 控制块负责。
    ~GMBCollector() = default;

public:
    // 仅由已收到的圈内样本判齐：未进圈或刚出圈时两个时刻为空，停止回报时 lastHitAt_ 不再推进。
    // 此处不读当前时间，避免仅因时间流逝就把缺少新回报的尝试判为完成。
    [[nodiscard]] bool settled() const override
    {
        return firstHitAt_ && lastHitAt_ && (*lastHitAt_ - *firstHitAt_) >= hold_;
    }

    // 本次尝试的**实测位形**：最近一次采样（进没进圈都记）。判齐之后由编排层读出，交给族基 `worldOf(位形)`
    // 换成世界朝向。**空 = 还没收到过采样**。
    [[nodiscard]] std::optional<PanTiltModel> measured() const { return last_; }

    // 本次 run **被命令去的**位形——运动参数目标的回读（收集器 → 位形）。每只斗都有目标
    // （扇扫 / 循环 / 方向键不造斗），故不是 optional。
    [[nodiscard]] PanTiltModel target() const { return parameters.target; }

    [[nodiscard]] QList<QByteArray> buildStart() const override { return start_; }
    [[nodiscard]] QList<QByteArray> buildStop() const override { return stop_; }
    [[nodiscard]] int timeoutMs() const override { return timeoutMs_; }


private:
    using Clock = std::chrono::steady_clock;

    // [worker] 每口先记录实测位形；圈外样本（距离大于容差）清空两个时刻，圈内样本首次进圈时记起点、
    // 每次推进末次时刻。settled 统一判断首末间隔；完成后 Collector::feed 不再调用本口，实测结果也随之冻结。
    // RxTime 是可能被校时调整的墙钟；这里量等待时长，使用处理样本时的单调钟。
    void onFeed(const PanTiltModel &sample, RxTime) override
    {
        last_ = sample;
        if (PanTiltModel::distance(sample, parameters.target) > tolerance_)
        {
            firstHitAt_ = std::nullopt;
            lastHitAt_ = std::nullopt;
            return;
        }
        const auto now = Clock::now();
        if (!firstHitAt_)
            firstHitAt_ = now;
        lastHitAt_ = now;
    }

    // ── 指示区（const：出生即定，多线程只读）──
protected:
    const double tolerance_;
    const std::chrono::milliseconds hold_;
    const QList<QByteArray> start_;
    const QList<QByteArray> stop_;
    const int timeoutMs_;
    // ── 收集区（[仅 worker] 可写；判齐后 worker 不再触斗，主线程只读）──
private:
    std::optional<PanTiltModel> last_;       // 最近一次采样（给 measured()，与判据无关）
    std::optional<Clock::time_point> firstHitAt_; // 本段连续在圈内的起点（**空 = 不在圈内**）
    std::optional<Clock::time_point> lastHitAt_;  // 本段最近一次圈内样本的时刻（与起点同空同有）
};

// 型号标签给出本型号的容差 tolerance（°）、看门狗时限 timeoutMs、按参数成帧的 startOf 与停机 stopOf；
// 三型号共享同一到位/驻留实现。
template <typename Model>
class TypedGMBCollector final : public GMBCollector
{
public:
    // 族模板层按已校验的运动参数直接构造完整、可写的本次 Collector，独占交给 Device。
    explicit TypedGMBCollector(const GMBParameters &parameters)
        : GMBCollector(parameters, Model::tolerance, Model::startOf(parameters), Model::stopOf(), Model::timeoutMs) {}

    // 本型号的停机命令与参数无关；族模板层据此实现 Device 的 stopCommands（无 run 时也能停）。
    [[nodiscard]] static QList<QByteArray> stopOf() { return Model::stopOf(); }
};

template <typename C>
concept GMBCollectorLike = std::derived_from<C, GMBCollector> && CollectorLike<C>
    && std::constructible_from<C, const GMBParameters &>
    && requires { { C::stopOf() } -> std::same_as<QList<QByteArray>>; };
