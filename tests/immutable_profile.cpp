#include "DAQFamily/WindProfile.h"
#include "WallClockHistory.h"
#include <type_traits>
#include <utility>

struct Spec {
    static constexpr const char *name = "immutable-test";
    static constexpr int rangeGateCount = 3, frequencyBinCount = 2;
    static constexpr double rangeStartM = 30, rangeStepM = 15;
    static constexpr double frequencyStartMHz = 79, frequencyStepMHz = 1;
    static constexpr double wavelengthM = 1.55e-6, zeroWindMHz = 80;
};
using Frame = VerticalWindProfileUnit<Spec>;
struct ThrowingSpec : Spec {
    ThrowingSpec() noexcept(false) {}
};
static_assert(DAQChannelSpec<ThrowingSpec>);
static_assert(std::is_nothrow_constructible_v<Frame, std::chrono::system_clock::time_point, double>);
static_assert(!std::is_nothrow_constructible_v<VerticalWindProfileUnit<ThrowingSpec>,
    std::chrono::system_clock::time_point, double>);
static_assert(ImmutableWallClockFrame<Frame>);
static_assert(!std::is_copy_constructible_v<Spectrum<Spec>> && !std::is_move_constructible_v<Spectrum<Spec>>);
static_assert(!std::is_copy_assignable_v<Spectrum<Spec>>);
static_assert(!std::is_copy_constructible_v<RadialWind<Spec>> && !std::is_move_constructible_v<RadialWind<Spec>>);
static_assert(!std::is_copy_assignable_v<RadialWind<Spec>>);
static_assert(StationaryObject<Frame>);
static_assert(std::derived_from<Frame, Spec>);
static_assert(std::derived_from<Frame, std::array<std::optional<LinAlg::Vec3>, 3>>);
static_assert(std::is_const_v<decltype(Frame::at)>);
static_assert(std::is_const_v<decltype(Frame::elevationDeg)>);
static_assert(std::same_as<decltype(std::declval<Frame&>()[0]), std::optional<LinAlg::Vec3>&>);
static_assert(!std::is_copy_constructible_v<Frame> && !std::is_move_constructible_v<Frame>);
static_assert(!std::is_copy_assignable_v<Frame> && !std::is_move_assignable_v<Frame>);
template<class T> concept RvalueBorrow = requires(T&& frame) { std::move(frame).values(); };
static_assert(!RvalueBorrow<Frame>);
struct MutableTime { std::chrono::system_clock::time_point at; };
static_assert(!ImmutableWallClockFrame<MutableTime>);
static constexpr auto timeOf = [](const auto &frame) { return frame->at; };
static_assert(WallClockElement<std::shared_ptr<const Frame>, timeOf>);
int main()
{
    const auto at = std::chrono::system_clock::time_point(std::chrono::seconds(100));
    std::array<std::optional<LinAlg::Vec3>, 3> values{LinAlg::Vec3(3,4,1), std::nullopt, LinAlg::Vec3::Zeros()};
    auto collecting = std::make_shared<Frame>(at, 30.0);
    (*collecting)[0] = values[0];
    (*collecting)[2] = values[2];
    const auto *original = collecting->data();
    std::shared_ptr<const Frame> first = std::move(collecting);
    if (collecting || first->data() != original) return 9;
    const auto *data = first->data();
    if (first->size() != 3 || first->end() - first->begin() != 3 || &first->front() != data) return 8;
    values[0] = LinAlg::Vec3(9,9,9);
    if ((*first)[0]->x() != 3 || (*first)[1] || !(*first)[2] || (*first)[2]->x() != 0) return 1;
    if (std::abs(first->heightStartM() - 15.0) > 1e-12
        || std::abs(first->heightStepM() - 7.5) > 1e-12) return 2;
    std::weak_ptr<const Frame> weak = first;
    auto view = first;
    if (view->values().data() != data || view->at != at || view->values().size() != 3) return 3;
    WallClockHistory<std::shared_ptr<const Frame>, timeOf> history(1);
    history.append(first);
    first.reset();
    history.append(std::make_shared<const Frame>(at + std::chrono::seconds(1), 60.0));
    if (weak.expired() || view->values()[0]->x() != 3 || history.back()->elevationDeg != 60) return 4;
    view.reset();
    if (!weak.expired()) return 5;
    if (history.extract(at, at + std::chrono::seconds(1)).size() != 1) return 6;
    if (std::shared_ptr<const Frame>{}) return 7;
    auto source = std::make_unique<Frame>(at, 90.0);
    const auto *dynamicData = source->data();
    auto target = std::move(source);
    if (target->data() != dynamicData) return 10;
    return 0;
}
