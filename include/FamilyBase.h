#pragma once
#include <concepts>
#include <memory>
#include <utility>
#include <QString>

#include "DeviceBase.h" // DeviceBase / Device<H,C>
#include "RunBase.h"    // DoneFn<C> / DeviceLock

// 设备族的共同骨架：一个族 = 一种请求 + 一种收集器族基。族基（GMBFamily / DAQFamily）继承本模板并
// 定下两个类型参数，本身仍不带模板；纯 C++，不继承 QObject（唯一 QObject 来自末端继承的 Handle）。
// 编排只持族引用：经 begin / stop 下单与撤单，经 device() 使用锁，不认识具体 Handle 或 Collector。
template <class RequestT, class CollectorBaseT>
class FamilyBase
{
public:
    // 等待器账本的请求槽与元素型：协程按设备保存请求，族口 begin 据此造 Collector。
    using Request = RequestT;
    using CollectorBase = CollectorBaseT;

    FamilyBase() = default;
    Q_DISABLE_COPY_MOVE(FamilyBase)
    virtual ~FamilyBase() = default;

    // 同一对象的机制面：族模板层返回自身的 DeviceBase 基类，供编排验锁和读取身份。
    [[nodiscard]] virtual DeviceBase &device() noexcept = 0;

    // 按请求启动一次 run（等待器在挂起成立后调）。请求只能由官邸 paramsFor 造出、存在即有效，FamilyDevice 以它
    // 直接构造本型号的具体 Collector 交 Device。返 false = 设备不收（被占 / 未连）。
    [[nodiscard]] virtual bool begin(const Request &request, DoneFn<CollectorBase> done,
                                     const std::shared_ptr<const DeviceLock> &lock) = 0;

    // 停机 / 撤单（等待器析构时对未结的单调）。与 begin 同一笔账：编排只认族基，停机的实现住机制层
    // Device<>，由 FamilyDevice 转过去。可反复调用（幂等）：对交付 / 取消 / 拒收回滚三条出路一视同仁。
    virtual void stop(const std::shared_ptr<const DeviceLock> &lock) = 0;
};

// 族基判据：派生自以自身 Request / CollectorBase 实例化的 FamilyBase。
template <typename T>
concept FamilyLike = std::derived_from<T, FamilyBase<typename T::Request, typename T::CollectorBase>>;

// 族模板层的共同实现：同时继承具体 Device 与族基，device() / stop() 统一落到机制层；begin 与族特有逻辑归各族模板。
template <HandleLike HandleT, typename CollectorT, FamilyLike FamilyT>
class FamilyDevice : public Device<HandleT, CollectorT>, public FamilyT
{
protected:
    using DeviceT = Device<HandleT, CollectorT>;

    FamilyDevice(QString name, QString subtitle, QObject *parent)
        : DeviceT(std::move(name), std::move(subtitle), parent)
    {
        static_assert(std::same_as<typename CollectorT::Params, typename FamilyT::Request>,
                      "型号收集器的参数类型必须就是本族的请求类型");
    }

public:
    // 返回自身的 DeviceBase 基类。不用协变返回 DeviceT&：MSVC 对结构体非类型模板实参（如国耀规格）
    // 与协变返回的组合报 C2062；持具体类型者直接在设备对象上使用完整 Device 接口。
    [[nodiscard]] DeviceBase &device() noexcept final { return *this; }

    // 两族同一实现：以请求直接构造本型号收集器交 Device，不再复核（请求存在即有效）。
    [[nodiscard]] bool begin(const typename FamilyT::Request &request, DoneFn<typename FamilyT::CollectorBase> done,
                             const std::shared_ptr<const DeviceLock> &lock) final
    {
        return DeviceT::start(request, std::move(done), lock);
    }

    // 族基的纯虚 stop 在此落地；机制层 Device::stop 不是虚的，停机虚口只在族基。
    void stop(const std::shared_ptr<const DeviceLock> &lock) final { DeviceT::stop(lock); }
};
