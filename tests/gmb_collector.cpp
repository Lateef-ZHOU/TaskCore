#include "GMBCollector.h"
#include <QtLogging>
#include <chrono>
#include <thread>
#include <type_traits>

namespace {
void require(bool ok, const char *why) { if (!ok) qFatal("%s", why); }

// 只提供测试型号的参数入口与命令，不替换生产时钟或驻留判据。
struct TestKey : GMBParamsKey {};
struct TestModel
{
    static constexpr double tolerance = 0.1;
    static constexpr int timeoutMs = 10000;
    static QList<QByteArray> startOf(const GMBParameters &) { return {"start"}; }
    static QList<QByteArray> stopOf() { return {"stop"}; }
};
using TestCollector = TypedGMBCollector<TestModel>;
template<class C> concept HasPublicClock = requires { typename C::Clock; };
template<class C> concept HasTargetGetter = requires(const C &c) { c.target(); };
template<class C> concept CanFeedConst = requires(const C &c, const PanTiltModel &pose) { c.feed(pose, RxTime{}); };
static_assert(GMBCollectorLike<TestCollector>);
static_assert(!HasPublicClock<TestCollector> && !HasTargetGetter<TestCollector> && !CanFeedConst<TestCollector>);
static_assert(!std::is_copy_constructible_v<TestCollector> && !std::is_move_constructible_v<TestCollector>
              && !std::is_copy_assignable_v<TestCollector> && !std::is_move_assignable_v<TestCollector>);
static_assert(std::is_destructible_v<TestCollector> && TestCollector::kMinHold == std::chrono::seconds(2));
}

int main()
{
    const TestKey key;
    const PanTiltModel target(10, 20), inside(10.05, 20.02), outside(30, 40);
    const auto hold = GMBCollector::kMinHold;
    const GMBParameters parameters(key, target, hold, GMBParameters::Speeds{10, 5});
    TestCollector stopped(parameters), reentered(parameters);
    TestCollector minimum(GMBParameters(key, target, std::chrono::milliseconds(0), std::nullopt));
    TestCollector longer(GMBParameters(key, target, hold + std::chrono::seconds(1), std::nullopt));
    const RxTime at{std::chrono::seconds(100000)};
    require(!stopped.measured() && !stopped.settled(), "new attempt already has observations");
    require(!stopped.feed(target, at) && !reentered.feed(target, at) && !minimum.feed(target, at)
            && !longer.feed(target, at), "first in-range sample completed before hold elapsed");
    require(!reentered.feed(outside, at), "out-of-range sample completed an unfinished attempt");

    std::this_thread::sleep_for(hold + std::chrono::milliseconds(100));
    require(!stopped.settled() && !reentered.settled(), "elapsed time alone completed an attempt");
    // 与 Device 看门狗一致：只复制参数构造新尝试，不能继承观测和已开始的驻留区间。
    TestCollector retry(stopped.parameters);
    require(!retry.measured() && !retry.settled()
            && retry.parameters.target.pan() == target.pan() && retry.parameters.target.tilt() == target.tilt()
            && retry.parameters.hold == hold && retry.parameters.speeds->pan == 10
            && retry.parameters.speeds->tilt == 5 && retry.buildStart() == stopped.buildStart()
            && retry.buildStop() == stopped.buildStop() && retry.timeoutMs() == stopped.timeoutMs(),
            "retry copied observations or lost request parameters");
    require(!retry.feed(inside, at) && !reentered.feed(inside, at), "retry or reentry retained the old hold interval");
    require(minimum.feed(inside, at) && !longer.feed(inside, at), "minimum or configured hold changed");
    require(stopped.feed(inside, at), "fresh in-range report did not complete elapsed attempt");
    const auto completed = stopped.measured();
    require(completed && PanTiltModel::distance(*completed, inside) == 0.0, "finishing observation was lost");
    require(stopped.feed(outside, at) && stopped.feed(target, at) && stopped.settled()
            && PanTiltModel::distance(*stopped.measured(), *completed) == 0.0,
            "completed attempt was reopened or its measured pose was overwritten");

    std::this_thread::sleep_for(hold + std::chrono::milliseconds(100));
    require(!retry.settled() && !reentered.settled() && !longer.settled(), "missing reports advanced hold intervals");
    // 回报墙钟倒退不影响单调钟驻留判定。
    require(retry.feed(target, at - std::chrono::seconds(10)) && reentered.feed(target, at) && longer.feed(target, at),
            "fresh intervals did not complete after their own hold and finishing reports");
    qInfo("PASS: fresh retry, reentry reset, stopped reports, hold bounds and completed observation freeze");
}
