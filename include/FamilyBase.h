#pragma once
#include <string>
#include <concepts>
#include <memory>
#include <utility>

#include "ObjectPolicy.h"
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
    DISABLE_COPY_MOVE(FamilyBase)
    virtual ~FamilyBase() = default;

    // 同一对象的机制面：族模板层返回自身的 DeviceBase 基类，供编排验锁和读取身份。
    [[nodiscard]] virtual DeviceBase &device() noexcept = 0;

    // [主线程] 请求由官邸 paramsFor 校验生成；提交期间不泵事件。true 仅表示接受请求，false 表示被占或未连接。
    // 接受后的完成通知须排队到主线程，在 await_suspend 返回后交付，不得在 begin 内同步完成；
    // 等待器在全部请求被接受后才登记续体和取消铃。FamilyDevice 交 Device::start 构造本型号 Collector。
    [[nodiscard]] virtual bool begin(const Request &request, DoneFn<CollectorBase> done,
                                     const std::shared_ptr<const DeviceLock> &lock) = 0;

    // [主线程] 停机 / 撤单：FamilyDevice 交 Device 验锁，失败不撤单；通过后可同步以空结果通知等待者。
    // 等待器析构沿此口撤销未完成请求；允许重复调用，族口不返回验锁结果。
    virtual void stop(const std::shared_ptr<const DeviceLock> &lock) = 0;
};

// 族基判据：派生自以自身 Request / CollectorBase 实例化的 FamilyBase。
template <typename T>
concept FamilyLike = std::derived_from<T, FamilyBase<typename T::Request, typename T::CollectorBase>>;

// 族模板层的共同实现：同时继承具体 Device 与族基，统一实现 device() / begin() / stop()；族特有逻辑归各族模板。
template <HandleLike HandleT, CollectorLike CollectorT, FamilyLike FamilyT>
    requires std::derived_from<CollectorT, typename FamilyT::CollectorBase>
          && std::same_as<typename CollectorT::Params, typename FamilyT::Request>
class FamilyDevice : public Device<HandleT, CollectorT>, public FamilyT
{
protected:
    using DeviceT = Device<HandleT, CollectorT>;

    FamilyDevice(const std::string &name, const std::string &subtitle, QObject *parent)
        : DeviceT(name, subtitle, parent) {}

public:
    // 返回自身的 DeviceBase 基类。不用协变返回 DeviceT&：MSVC 对结构体非类型模板实参（如国耀规格）
    // 与协变返回的组合报 C2062；持具体类型者直接在设备对象上使用完整 Device 接口。
    [[nodiscard]] DeviceBase &device() noexcept final { return *this; }

    // 两族共用：将已校验的请求交 Device::start，由其构造本次收集器并启动运行槽。
    [[nodiscard]] bool begin(const typename FamilyT::Request &request, DoneFn<typename FamilyT::CollectorBase> done,
                             const std::shared_ptr<const DeviceLock> &lock) final
    {
        return DeviceT::start(request, std::move(done), lock);
    }

    // 族基的纯虚 stop 在此落地；机制层 Device::stop 不是虚的，停机虚口只在族基。
    void stop(const std::shared_ptr<const DeviceLock> &lock) final { DeviceT::stop(lock); }
};
