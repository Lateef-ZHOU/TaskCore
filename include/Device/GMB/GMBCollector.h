#pragma once
#include <algorithm>
#include <chrono>
#include <concepts>
#include <memory>
#include <optional>
#include <utility>
#include <QByteArray>
#include <QList>

#include "Algorithm/Geometry/GimbalModel.h" // PanTiltModel（目标 / 实测位形；distance）
#include "Device/Base/RunBase.h" // Collector<FeedT> 契约基类

// 云台族公共到位/驻留判据与位形读口；型号以 TypedGMBCollector 的标签区分。
// 指示区不可变，收集区只由收包线程写，完成后按 const 份额交付。
// 对象不可复制或移动的约束由 Collector 基类统一提供，调用方只共享指针份额。
class GMBCollector : public Collector<PanTiltModel>
{
public:
    static constexpr std::chrono::milliseconds kMinHold{2000}; // 驻留下限：任务页的输入框同此下限，斗再兜一道

    // 型号通过 makeMoveCollector 交齐一次尝试的不可变配置：目标、容差（°）、等待时长、
    // 启动 / 停机指令和看门狗时限（ms，非正 = 不设看门狗）。
    // hold 对应任务页的“停稳等待”，不足 kMinHold 时取下限。首次进圈后保留起点，
    // 中途出圈不重新计时；再次收到圈内样本且首末时刻相差达到 hold 才完成，不要求连续在圈内。
protected:
    GMBCollector(PanTiltModel target, const double tolerance, const std::chrono::milliseconds hold,
                 QList<QByteArray> start, QList<QByteArray> stop, const int timeoutMs)
        : target_(std::move(target)), tolerance_(tolerance), hold_(std::max(hold, kMinHold)), start_(std::move(start)),
          stop_(std::move(stop)), timeoutMs_(timeoutMs)
    {
    }
    // 族基不拥有对象，销毁由具体类型的 shared_ptr 控制块负责。
    ~GMBCollector() = default;

public:
    // 仅由已收到的圈内样本判齐：未进圈时 firstHitAt_ 为空，停止回报时 lastHitAt_ 不再推进。
    // 此处不读当前时间，避免仅因时间流逝就把缺少新回报的尝试判为完成。
    [[nodiscard]] bool settled() const override
    {
        return firstHitAt_ && (lastHitAt_ - *firstHitAt_) >= hold_;
    }

    // 本次尝试的**实测位形**：最近一次采样（进没进圈都记）。判齐之后由编排层读出，交给族基 `worldOf(位形)`
    // 换成世界朝向。**空 = 还没收到过采样**。
    [[nodiscard]] std::optional<PanTiltModel> measured() const { return last_; }

    // 本次 run **被命令去的**位形——`collectorFor` 的反方向（收集器 → 位形）。每只斗都有目标
    // （扇扫 / 循环 / 方向键不造斗），故不是 optional。
    [[nodiscard]] PanTiltModel target() const { return target_; }

    [[nodiscard]] QList<QByteArray> buildStart() const override { return start_; }
    [[nodiscard]] QList<QByteArray> buildStop() const override { return stop_; }
    [[nodiscard]] int timeoutMs() const override { return timeoutMs_; }


private:
    using Clock = std::chrono::steady_clock;

    // [worker] 每口先记录实测位形；圈内样本设置首次时刻并推进末次时刻，圈外样本不改两个时刻。
    // settled 统一判断首末间隔；完成后 Collector::feed 不再调用本口，实测结果也随之冻结。
    // RxTime 是可能被校时调整的墙钟；这里量等待时长，使用处理样本时的单调钟。
    void onFeed(const PanTiltModel &sample, RxTime) override
    {
        last_ = sample;
        if (PanTiltModel::distance(sample, target_) >= tolerance_)
            return;
        const auto now = Clock::now();
        if (!firstHitAt_)
            firstHitAt_ = now;
        lastHitAt_ = now;
    }

    // ── 指示区（const：出生即定，多线程只读）──
protected:
    const PanTiltModel target_;
    const double tolerance_;
    const std::chrono::milliseconds hold_;
    const QList<QByteArray> start_;
    const QList<QByteArray> stop_;
    const int timeoutMs_;
    // ── 收集区（[仅 worker] 可写；判齐后 worker 不再触斗，主线程只读）──
private:
    std::optional<PanTiltModel> last_;       // 最近一次采样（给 measured()，与判据无关）
    std::optional<Clock::time_point> firstHitAt_; // 第一次落进容差圈的时刻（**空 = 还没进过圈**）
    Clock::time_point lastHitAt_{};               // 最近一次落进容差圈的时刻（firstHitAt_ 有值时才有意义）
};

// 标签只决定 C++ 身份；三型号共享同一到位/驻留实现和重试工厂。
template <typename Model>
class TypedGMBCollector final : public GMBCollector
{
public:
    [[nodiscard]] static std::shared_ptr<const TypedGMBCollector> createPrototype(
        PanTiltModel target, double tolerance, std::chrono::milliseconds hold,
        QList<QByteArray> start, QList<QByteArray> stop, int timeoutMs)
    {
        return std::shared_ptr<const TypedGMBCollector>(new const TypedGMBCollector(
            std::move(target), tolerance, hold, std::move(start), std::move(stop), timeoutMs));
    }

    // 云台收集区是内嵌的到位观测，不需谱数组；每次创建独立初始观测。
    [[nodiscard]] std::shared_ptr<TypedGMBCollector> makeSelf() const
    {
        return std::shared_ptr<TypedGMBCollector>(new TypedGMBCollector(target_, tolerance_, hold_, start_, stop_, timeoutMs_));
    }

private:
    TypedGMBCollector(PanTiltModel target, double tolerance, std::chrono::milliseconds hold,
                      QList<QByteArray> start, QList<QByteArray> stop, int timeoutMs)
        : GMBCollector(std::move(target), tolerance, hold, std::move(start), std::move(stop), timeoutMs) {}
};

template <typename C>
concept GMBCollectorLike = std::derived_from<C, GMBCollector> && CollectorLike<C>;
