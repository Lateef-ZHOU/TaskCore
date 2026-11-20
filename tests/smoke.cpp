#include <vector>
#include <optional>
#include <chrono>
#include <type_traits>
#include <QMetaType>
#include <QCoreApplication>
#include <QByteArray>
#include "DeviceCoro.h"
#include "FamilyBase.h"
#include "HandleBase.h"
#include "RunBase.h"
#include "DAQFamily/DAQDevice.h"
#include "GMBFamily/GMBDevice.h"
#include "DAQFamily/WindProfile.h"
static_assert(AwaitableDevice<GMBFamily>);
static_assert(AwaitableDevice<DAQFamily>);
namespace {
// 在不构造设备或调用 begin 的情况下检查模板入口，避免错误延后到构造体或回调转换。
class ContractCollectorBase : public Collector<int, int>
{
protected:
    explicit ContractCollectorBase(const int &value) : Collector(value) {}
};
class OtherCollectorBase : public Collector<int, int>
{
protected:
    explicit OtherCollectorBase(const int &value) : Collector(value) {}
};
template<class Base>
class ContractCollector : public Base
{
public:
    explicit ContractCollector(const int &value) : Base(value) {}
    bool settled() const override { return false; }
    std::vector<std::vector<unsigned char>> buildStart() const override { return {}; }
    int timeoutMs() const override { return 1; }
private:
    void onFeed(const int &, RxTime) override {}
};
using MatchingCollector = ContractCollector<ContractCollectorBase>;
using ForeignCollector = ContractCollector<OtherCollectorBase>;
using ContractFamily = FamilyBase<int, ContractCollectorBase>;
using WrongRequestFamily = FamilyBase<long, ContractCollectorBase>;
template<class C, class F>
concept CanNameFamilyDevice = requires { typename FamilyDevice<Handle<int, int>, C, F>; };
static_assert(CollectorLike<MatchingCollector> && CollectorLike<ForeignCollector>);
static_assert(CanNameFamilyDevice<MatchingCollector, ContractFamily>);
// 相同 Params 不能替代族归属；相同族归属也不能替代精确请求类型。
static_assert(!CanNameFamilyDevice<ForeignCollector, ContractFamily>);
static_assert(!CanNameFamilyDevice<MatchingCollector, WrongRequestFamily>);
static_assert(!CanNameFamilyDevice<ContractCollectorBase, ContractFamily>);
static_assert(!CanNameFamilyDevice<int, ContractFamily>);
static_assert(!CanNameFamilyDevice<MatchingCollector, int>);
}
inline constexpr DAQParams<UniformRangeMapping<100.0, 25.0, 1>> probeParams{
    .wavelengthM = 1.55e-6, .zeroWindMHz = 80,
    .frequencyStartMHz = 0, .frequencyStepMHz = 1,
    .frequencyBinCount = 1,
};
using Frame = VerticalWindProfileUnit<probeParams>;
static_assert(std::is_same_v<RxTime, std::chrono::system_clock::time_point>);
static_assert(std::is_same_v<decltype(Frame::at), const RxTime>);
static_assert(!std::is_copy_constructible_v<DeviceLock>);
int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    if (!QMetaType::fromType<WorldModel>().isValid()
        || !QMetaType::fromType<PanTiltModel>().isValid()) return 37;
    const auto now = std::chrono::system_clock::now();
    Frame frame{now, 90.0};
    frame[0] = LinAlg::Vec3{1,2,3};
    if (!WindProfile::hasValidGrid(frame) || frame.heightStartM() != 100.0 || frame.at != now) return 1;
    const WorldModel world(35.0, 60.0);
    const WorldPoseHistory history{{now, world}};
    if (history.front().first != now) return 2;
    return 0;
}
