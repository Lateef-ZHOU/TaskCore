#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <type_traits>
#include <vector>
#include "DAQFamily/RadialWind.h"
#include "LinearArray.h"
#include "DAQFamily/WindProfile.h"
#include "WallClockHistory.h"
#include "SO3.h"
#include "GMBFamily/GimbalModel.h"
#include "GMBFamily/SiteCalibration.h"
static_assert(std::is_same_v<decltype(&SiteCalibration::estimateFromSamples),
    std::optional<SiteCalibration::Estimate> (*)(const std::span<const SiteCalibration::Sample>&) noexcept>);
static_assert(noexcept(SiteCalibration::estimateFromSamples(std::span<const SiteCalibration::Sample>{})));
static_assert(noexcept(Wrap360(0.0)));
static_assert(std::is_base_of_v<std::vector<Joint>, GimbalModel>);
template<class T> concept PublicArrayAccess = requires(T &v) { v[0]; v.data(); v.fill(0.0); };
static_assert(!PublicArrayAccess<Joint>);
struct MissingInverse : GimbalModel {};
struct WrongInverseResult : GimbalModel {
    static WorldModel fromOrientation(const SO3 &);
};
struct InheritedInverse : WorldModel {};
struct UnrelatedModel {
    static UnrelatedModel fromOrientation(const SO3 &);
};
class PrivateModel : private GimbalModel {
public:
    static PrivateModel fromOrientation(const SO3 &);
};
static_assert(!GimbalModelLike<GimbalModel> && !GimbalModelLike<MissingInverse>);
static_assert(!GimbalModelLike<WrongInverseResult> && !GimbalModelLike<InheritedInverse>);
static_assert(!GimbalModelLike<UnrelatedModel> && !GimbalModelLike<PrivateModel>);
template<class T> concept ModelDistanceAvailable = requires(const T &a, const T &b) {
    { GimbalModel::distance(a, b) } -> std::same_as<double>;
};
static_assert(ModelDistanceAvailable<WorldModel> && ModelDistanceAvailable<PanTiltModel>);
static_assert(!ModelDistanceAvailable<GimbalModel> && !ModelDistanceAvailable<MissingInverse>);
static_assert(!ModelDistanceAvailable<WrongInverseResult> && !ModelDistanceAvailable<InheritedInverse>);
static_assert(!ModelDistanceAvailable<UnrelatedModel> && !ModelDistanceAvailable<PrivateModel>);
static_assert(!std::is_convertible_v<WorldModel*, std::vector<Joint>*>);
static_assert(!std::is_convertible_v<PanTiltModel*, std::vector<Joint>*>);
template<class T> concept PublicJointContainer = requires(T &v) { v.clear(); v[0]; v.begin(); };
template<class T> concept PublicEquality = requires(const T &a, const T &b) { a == b; };
static_assert(!PublicJointContainer<WorldModel> && !PublicJointContainer<PanTiltModel>);
static_assert(!PublicEquality<WorldModel> && !PublicEquality<PanTiltModel>);
template<class T> concept PublicAxis = requires(const T &v) { v.axis(0); };
static_assert(!PublicAxis<GimbalModel> && !PublicAxis<WorldModel> && !PublicAxis<PanTiltModel>);
// 测试派生类可读关节，生产模型不公开该入口。
struct AxisProbe : WorldModel { using GimbalModel::axis; };
static_assert(std::is_same_v<decltype(std::declval<const AxisProbe&>().axis(0)), const Joint&>);
static_assert(noexcept(std::declval<const AxisProbe&>().axis(0)));
static_assert([] {
    constexpr Joint joint({0, 0, -1}, 0, 35);
    return joint.value() == 35 && joint.z() == -1;
}());
static_assert(noexcept(Joint({0, 0, -1}, 0, 35)));
static_assert(noexcept(std::declval<const Joint&>().value()));
static_assert(noexcept(std::declval<const WorldModel&>().azimuth()));
static_assert(noexcept(std::declval<const WorldModel&>().elevation()));
static_assert(noexcept(std::declval<const WorldModel&>().roll()));
static_assert(noexcept(std::declval<const PanTiltModel&>().pan()));
static_assert(noexcept(std::declval<const PanTiltModel&>().tilt()));
static_assert(noexcept(std::declval<const GimbalModel&>().toOrientation()));
static_assert(std::is_copy_assignable_v<WorldModel> && std::is_move_assignable_v<WorldModel>);
static_assert(std::is_copy_assignable_v<PanTiltModel> && std::is_move_assignable_v<PanTiltModel>);
static_assert(std::is_copy_constructible_v<WorldModel> && std::is_copy_constructible_v<PanTiltModel>);
static_assert(std::is_nothrow_move_constructible_v<WorldModel> && std::is_nothrow_move_assignable_v<WorldModel>);
static_assert(std::is_nothrow_move_constructible_v<PanTiltModel> && std::is_nothrow_move_assignable_v<PanTiltModel>);
using RxTime = std::chrono::system_clock::time_point;
template<class Model>
bool modelStorageTransfer()
{
    struct Probe : Model {
        using Model::Model;
        using GimbalModel::axis;
    };
    Probe source(35.0, 60.0);
    const Joint *storage = &source.axis(0);
    Probe copied(source);
    if (&copied.axis(0) == storage || copied.axis(0).value() != 35.0) return false;
    Probe moved(std::move(source));
    if (&moved.axis(0) != storage || moved.axis(1).value() != 60.0) return false;
    Probe assigned(10.0, 20.0);
    assigned = std::move(moved);
    if (&assigned.axis(0) != storage || assigned.axis(0).value() != 35.0) return false;
    // 移出对象先重新赋值，再读取；不依赖移后容器是否为空。
    source = copied;
    moved = Probe(15.0, -20.0);
    return &source.axis(0) != &copied.axis(0)
        && source.axis(0).value() == 35.0 && source.axis(1).value() == 60.0
        && moved.axis(0).value() == 15.0 && moved.axis(1).value() == -20.0;
}

template<int Gates, int Start = 0, int Step = 15>
struct ProfileSpec {
    static constexpr const char *name = "test";
    static constexpr int rangeGateCount = Gates, frequencyBinCount = 1;
    static constexpr double rangeStartM = Start, rangeStepM = Step, frequencyStartMHz = 0, frequencyStepMHz = 1;
    static constexpr double wavelengthM = 1.55e-6, zeroWindMHz = 80;
};

template<int ZeroWind>
struct SpectrumSpec : ProfileSpec<2, 30> {
    static constexpr int frequencyBinCount = 9;
    static constexpr double frequencyStartMHz = 10, frequencyStepMHz = 2;
    static constexpr double zeroWindMHz = ZeroWind;
};
template<class Spec> auto wind(std::initializer_list<double> values)
{
    auto frame = std::make_shared<RadialWind<Spec>>();
    std::copy(values.begin(), values.end(), frame->begin());
    return frame;
}

int main()
{
    const auto now = std::chrono::system_clock::now();
    using Frame = VerticalWindProfileUnit<ProfileSpec<1, 100, 25>>;
    auto frameOwner = std::make_shared<Frame>(now, 90.0);
    (*frameOwner)[0] = LinAlg::Vec3{1,2,3};
    auto &frame = *frameOwner;
    if (!WindProfile::hasValidGrid(frame) || frame.heightStartM() != 100.0 || frame.at != now) return 1;
    const WorldModel world(35.0, 60.0);
    const WorldPoseHistory history{{now, world}};
    if (history.front().first != now) return 2;
    WallClockHistory<std::shared_ptr<const Frame>, [](const std::shared_ptr<const Frame> &profile) { return profile->at; }> history2(2);
    auto column = std::shared_ptr<const Frame>(std::move(frameOwner));
    std::weak_ptr<const Frame> old = column;
    history2.append(column);
    if (history2.at(0).get() != column.get()) return 3;
    column.reset();
    history2.append(std::make_shared<const Frame>(
        now + std::chrono::seconds(1), 90.0));
    if (history2.size() != 2 || old.expired()) return 4;
    history2.append(std::make_shared<const Frame>(
        now + std::chrono::seconds(2), 90.0));
    if (history2.size() != 2 || !old.expired() || history2.at(0)->at != now + std::chrono::seconds(1)
        || history2.back()->at != now + std::chrono::seconds(2)) return 5;
    const LinearArray<double> axis(10, 15, {0, 0, 0, 0});
    if (axis.positionAt(3) != 55 || !axis.hasValidAxis() || LinearArray<double>(0, 0).hasValidAxis()
        || !axis.sameAxis(LinearArray<int>(10, 15, {1, 2, 3, 4})) || axis.sameAxis(LinearArray<double>(10, 15, {0}))) return 6;
    std::array<WindProfile::Input<ProfileSpec<3>>, 4> beams;
    for (int i = 0; i < 4; ++i)
        beams[i] = {WorldModel(90.0 * i, 30.0), wind<ProfileSpec<3>>({1, 2, 3})};
    // 30° 仰角：高度 = 斜距 / 2；起点 0、门距 15 m。
    const auto column3 = WindProfile::Calc<ProfileSpec<3>>(beams, now);
    if (!column3 || column3->at != now || column3->heightStartM() != 0 || std::abs((column3->heightStartM() + 2 * column3->heightStepM()) - 15) > 1e-9
        || std::abs(column3->values()[2]->z() - 6) > 1e-9) return 7;
    // 不同网格不能装入同一输入；缺失实测份额仍拒绝反演。
    beams[3].ws.reset();
    if (WindProfile::Calc<ProfileSpec<3>>(beams, now)) return 8;
    Spectrum<SpectrumSpec<18>> spectrum;
    Spectrum<SpectrumSpec<16>> shiftedSpectrum;
    Spectrum<SpectrumSpec<80>> outOfBandSpectrum;
    for (int gate = 0; gate < 2; ++gate)
        for (int bin = 0; bin < 9; ++bin)
            spectrum[gate * 9 + bin] = shiftedSpectrum[gate * 9 + bin] = outOfBandSpectrum[gate * 9 + bin]
                = 1.0 + 100.0 * std::exp(-(bin - 4.0) * (bin - 4.0) / 2.0);
    const auto calm = RadialWindCalculation::Calc(spectrum);
    if (calm.ws->size() != 2 || calm.ws->rangeStartM != 30.0 || calm.ws->rangeStepM != 15.0
        || std::abs((*calm.ws)[0]) > 0.01) return 11;
    const auto shifted = RadialWindCalculation::Calc(shiftedSpectrum);
    if (std::abs((*shifted.ws)[1] - 2.0 * 0.775) > 0.01) return 12;
    // 参考频率在不在频带内是硬件配置的事，算法照常换算。
    const auto outOfBand = RadialWindCalculation::Calc(outOfBandSpectrum);
    if (std::abs((*outOfBand.ws)[0] - (18.0 - 80.0) * 0.775) > 0.01 || (*outOfBand.cnr)[0] != (*calm.cnr)[0]) return 13;
    const auto zero = [](const LinAlg::Vec3 &v) { return v.x() == 0 && v.y() == 0 && v.z() == 0; };
    const LinAlg::Vec3 a(1, 2, 3), b(4, 5, 6);
    VerticalWindProfileUnit<ProfileSpec<3, 0, 10>> hole(now, 90);
    hole[0] = a; hole[2] = b;
    if (!WindProfile::windAt(hole, 0) || WindProfile::windAt(hole, 5) || WindProfile::windAt(hole, 10) || WindProfile::windAt(hole, 15)
        || !WindProfile::windAt(hole, 20) || WindProfile::windAt(hole, 0)->x() != 1 || WindProfile::windAt(hole, 20)->z() != 6) return 17;
    VerticalWindProfileUnit<ProfileSpec<2, 0, 10>> still(now, 90);
    still.fill(LinAlg::Vec3::Zeros());
    if (!WindProfile::windAt(still, 5) || !zero(*WindProfile::windAt(still, 5))) return 18;
    VerticalWindProfileUnit<ProfileSpec<2, 0, 10>> ramp(now, 90);
    ramp[0] = a; ramp[1] = b;
    if (!WindProfile::windAt(ramp, 5) || WindProfile::windAt(ramp, 5)->x() != 2.5 || WindProfile::windAt(ramp, 5)->z() != 4.5) return 19;
    std::array<WindProfile::Input<ProfileSpec<2>>, 4> missingBeams;
    for (int i = 0; i < 4; ++i)
        missingBeams[i] = {WorldModel(90.0 * i, 30.0), wind<ProfileSpec<2>>({0, 0})};
    missingBeams[0].ws = wind<ProfileSpec<2>>({0, std::numeric_limits<double>::quiet_NaN()});
    const auto missingLayer = WindProfile::Calc<ProfileSpec<2>>(missingBeams, now);
    if (!missingLayer || !missingLayer->values()[0] || !zero(*missingLayer->values()[0]) || missingLayer->values()[1]) return 26;
    // 多轴样本覆盖标定的外积、补秩及朝向求和；单轴保持原退化行为。
    const auto base = SO3::exp(LinAlg::Vec3(0.1, -0.2, 0.3));
    const auto offset = SO3::exp(LinAlg::Vec3(-0.05, 0.08, 0.02));
    std::array<SiteCalibration::Sample, 8> samples;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto gimbal = PanTiltModel(static_cast<double>(i) * 40.0, static_cast<double>(i % 2) * 40.0).toOrientation();
        samples[i] = {gimbal, base * gimbal * offset};
    }
    const auto estimate = SiteCalibration::estimateFromSamples(samples);
    if (!estimate || !estimate->offsetResolved || estimate->residualDeg > 1e-5) return 32;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            if (std::abs(estimate->base(r, c) - base(r, c)) > 1e-9
                || std::abs(estimate->offset(r, c) - offset(r, c)) > 1e-9) return 33;
    std::array<SiteCalibration::Sample, 4> singleAxisSamples;
    for (std::size_t i = 0; i < singleAxisSamples.size(); ++i) {
        const auto gimbal = PanTiltModel(static_cast<double>(i) * 30.0, 0.0).toOrientation();
        singleAxisSamples[i] = {gimbal, base * gimbal};
    }
    const auto singleAxis = SiteCalibration::estimateFromSamples(singleAxisSamples);
    if (!singleAxis || singleAxis->offsetResolved || singleAxis->residualDeg > 1e-5) return 34;
    for (std::size_t count = 0; count < 3; ++count)
        if (SiteCalibration::estimateFromSamples(std::span<const SiteCalibration::Sample>(samples).first(count))) return 35;
    // 动态容器同样可借用；输入不被改写，结果与固定数组一致。
    const std::vector<SiteCalibration::Sample> dynamicSamples(samples.begin(), samples.end());
    const auto dynamicEstimate = SiteCalibration::estimateFromSamples(dynamicSamples);
    if (!dynamicEstimate || dynamicEstimate->offsetResolved != estimate->offsetResolved
        || dynamicEstimate->residualDeg != estimate->residualDeg) return 44;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            if (dynamicEstimate->base(r, c) != estimate->base(r, c)
                || dynamicEstimate->offset(r, c) != estimate->offset(r, c)
                || singleAxis->offset(r, c) != (r == c ? 1.0 : 0.0)) return 45;
            for (std::size_t i = 0; i < samples.size(); ++i)
                if (samples[i].gimbal(r, c) != dynamicSamples[i].gimbal(r, c)
                    || samples[i].measured(r, c) != dynamicSamples[i].measured(r, c)) return 46;
        }
    // 相邻转动为 120°，地基求和退化；失败只交 nullopt，不暴露部分结果。
    const std::array<SiteCalibration::Sample, 3> contradictory{{
        {SO3::Identity(), SO3::Identity()},
        {SO3::Identity(), SO3::exp({0, 0, 2.0 * std::numbers::pi / 3.0})},
        {SO3::Identity(), SO3::exp({0, 0, 4.0 * std::numbers::pi / 3.0})}}};
    if (SiteCalibration::estimateFromSamples(contradictory)) return 47;
    // 角度回绕归装配模型；保留负角、整圈与带偏置关节窗的行为。
    if (Wrap360(-30.0) != 330.0 || Wrap360(390.0) != 30.0
        || Wrap360(360.0) != 0.0 || Wrap360(-360.0) != 0.0
        || Joint({1, 0, 0}, -180.0, 190.0).value() != -170.0) return 36;
    // 位形复制、赋值及跨零距离保持。
    WorldModel assignedWorld;
    assignedWorld = world;
    PanTiltModel assignedPose;
    assignedPose = PanTiltModel(359.0, -20.0);
    const auto restoredWorld = WorldModel::fromOrientation(assignedWorld.toOrientation());
    const auto restoredPose = PanTiltModel::fromOrientation(assignedPose.toOrientation());
    if (WorldModel::distance(world, assignedWorld) != 0.0
        || WorldModel::distance(world, restoredWorld) > 1e-9
        || PanTiltModel::distance(assignedPose, restoredPose) > 1e-9
        || PanTiltModel::distance(assignedPose, PanTiltModel(1.0, -20.0)) != 2.0) return 38;
    // 三轴反解归 WorldModel；普通姿态、极点及阈值两侧仍保持原代表解。
    for (const double elevation : {-90.0, -89.95, -89.85, 0.0, 89.85, 89.95, 90.0}) {
        const WorldModel input(35.0, elevation, 20.0);
        const SO3 orientation = input.toOrientation();
        const WorldModel restored = WorldModel::fromOrientation(orientation);
        const PanTiltModel projected = PanTiltModel::fromOrientation(orientation);
        if (AngularDistance(restored.elevation(), elevation) > 1e-6
            || projected.pan() != restored.azimuth()
            || projected.tilt() != restored.elevation()) return 39;
        if (std::abs(elevation) >= 89.95) {
            if (restored.roll() != 0.0) return 40;
            if (std::abs(elevation) == 90.0
                && AngularDistance(restored.azimuth(), elevation > 0 ? 55.0 : 15.0) > 1e-9) return 41;
        } else if (WorldModel::distance(input, restored) > 1e-9) return 42;
    }
    if (!modelStorageTransfer<WorldModel>() || !modelStorageTransfer<PanTiltModel>()) return 43;
    return 0;
}
