#pragma once
#include <chrono>
#include <memory>
#include <optional>
#include <utility>
#include <QDir>
#include <QString>
#include <QThread>
#include <QUrl>

#include "Device/DAQ/DAQCollector.h"          // 采集帧与不可变消费上下文
#include "Device/DAQ/DAQWorker.h"             // 族级帧处理契约（兼投递锚点）
#include "Device/Base/DeviceCoro.h"           // DeviceCoro / LockRequest / HeldLock / RunAwaiter（测试采集协程）
#include "Device/Base/DeviceBase.h" // DeviceBase / Device<H,C>
#include "Device/Base/RunBase.h"    // DoneFn<C> / DeviceLock / CollectorLike

// 采集卡族接口供任务多选组合；模板实现持具体 Collector、worker 和测试协程。
// begin 是唯一的族基到型号验型边界；成功帧由主线程触发 onConsume，独立于协程是否仍等待。
class DAQDeviceBase
{
public:
    // 本族的收集器基（`RunAwaiter<DAQDeviceBase>` 由此取账本的元素型——族基是非模板类，
    // 故这条别名就是等待器"存得下"的全部依据）。
    using CollectorBase = DAQCollector;

    // ── pcap 读取政策：**族级常量** ────────
    // 住族基而不是型号宪法：这两个数由**本族的流量形状**决定——帧是成批爆发的，
    // 一帧上百个包、二三十万字节（各卡的帧常量见各自的 RunSide），且**判齐后整帧才有意义**，丢一个包
    // 整帧作废。与云台族恰好相反的取舍：**可以慢，不能丢**。
    //
    //   · **100 ms 唤醒**：成批交付正合适，本族的时间戳按帧算、不按包算，批量无害。
    //     （实际上多半是 mintocopy 先到——16 KB 在线速下瞬间攒满，这个数很少真用上。）
    //   · **4 MB 缓冲**：十帧以上的余量。**真约束是爆发**：一帧线速打出，硬下限是一帧。
    //
    // 100 ms 是长期实跑过的值，缓冲比当时的 1 MB 更宽——用户据此认为本族无需再验。
    static constexpr std::chrono::milliseconds kPcapReadTimeout{100};
    static constexpr int kPcapKernelBufferBytes = 4 * 1024 * 1024;

    // 交出 DeviceBase 面（族模板层一行实现）。**旁挂形制的必需口**，理由见 GMBDeviceBase。
    [[nodiscard]] virtual DeviceBase &device() noexcept = 0;

    // 只按传入的采集参数与消费上下文创建原型，不改变设备或发送命令。
    [[nodiscard]] virtual std::shared_ptr<const DAQCollector> collectorFor(int accumSec, DAQContext context) const = 0;

    // 启动一只本族的斗（等待器在挂起成立后调）。返 false = 设备不收。形制与 GMBDeviceBase
    // 逐条对仗，理由见那边。
    [[nodiscard]] virtual bool begin(const std::shared_ptr<const DAQCollector> &proto,
                                     DoneFn<DAQCollector> done,
                                     const std::shared_ptr<const DeviceLock> &lock) = 0;

    // 停机 / 撤单：与 begin 同一笔账，形制逐条对仗 GMBDeviceBase::stop（见那边）。
    virtual void stop(const std::shared_ptr<const DeviceLock> &lock) = 0;

protected:
    DAQDeviceBase() = default;
    ~DAQDeviceBase() = default; // 非虚 + protected：本类从不拥有对象，理由同 GMBDeviceBase
};

// 同一具体 Collector 类型贯穿启动、运行、重试和处理器。
template <HandleLike HandleT, DAQCollectorLike CollectorT>
class DAQDevice : public Device<HandleT, CollectorT>, public DAQDeviceBase
{
protected:
    using Base = Device<HandleT, CollectorT>;

    explicit DAQDevice(QObject *parent, std::unique_ptr<DAQWorker<CollectorT>> worker)
        : Base(parent), sink_(std::move(worker))
    {
        static_assert(DeviceLifecycleLayer<DAQDevice, decltype(&DAQDevice::PreDestruction)>);
        sink_->moveToThread(&worker_);
        worker_.start(); // QThread::run 的默认体就是 exec()，正是我们要的事件循环
    }

protected:
    // 本层独占处理器；官邸只借用接线，不得转移所有权或另存处理器成员。
    // const unique_ptr 不能复制、release 或 reset；实际接线处自行 get/解引用。
    [[nodiscard]] const std::unique_ptr<DAQWorker<CollectorT>> &worker() const noexcept
    {
        return sink_;
    }

public:
    // 构造失败时末端析构不会执行；此处只兜底退出独立 worker，不触发设备业务回调。
    ~DAQDevice() override { stopWorker(); }

protected:
    void PreDestruction() noexcept override
    {
        stopWorker();
        cancelTest();
        Base::PreDestruction();
    }

private:
    // 本层是处理线程源：统一退出事件循环并等待在途处理结束，再清理本层工作。
    // 正常预析构及构造失败的析构共用；收包线程由 Handle 负责关闭。
    void stopWorker() noexcept
    {
        if (QThread::currentThread() == &worker_)
            qFatal("DAQ worker cannot wait for itself");
        worker_.quit();
        if (!worker_.wait())
            qFatal("DAQ worker did not finish");
    }

private:
    // 采集卡族原样交回字节；解析由具体 Collector 在进料时负责。
    [[nodiscard]] std::optional<QByteArray> onStream(const QByteArray &raw, const RxTime) final { return raw; }

public:
    [[nodiscard]] DeviceBase &device() noexcept final { return *this; }

    // 保留族接口的公开入口，按具体 Collector 类型创建只读原型。
    [[nodiscard]] std::shared_ptr<const DAQCollector> collectorFor(int accumSec, DAQContext context) const final
    {
        // 静态工厂只造本型号轻量原型；begin 内 makeSelf 另建可写收集区。
        return CollectorT::createPrototype(accumSec, std::move(context)); // 具体份额隐式上行成族份额
    }

    // 外部族请求只在此验型；拒收发生在装运行槽、发送命令之前。
    [[nodiscard]] bool begin(const std::shared_ptr<const DAQCollector> &proto,
                             DoneFn<DAQCollector> done,
                             const std::shared_ptr<const DeviceLock> &lock) final
    {
        const auto typed = std::dynamic_pointer_cast<const CollectorT>(proto);
        if (!typed)
        {
            Q_EMIT this->logSource().logMessage(QStringLiteral("采集请求的 Collector 型号不匹配，未启动"));
            return false;
        }
        return this->start(typed, std::move(done), lock);
    }

    // 停机：族基的纯虚 stop 在此落地（final），实现就是机制层 Device::stop（同 GMBDevice::stop）。
    void stop(const std::shared_ptr<const DeviceLock> &lock) final { Base::stop(lock); }

protected:
    virtual void onConsume(const std::shared_ptr<const CollectorT> &frame) const = 0;

private:
    void onCompleted(const std::shared_ptr<const CollectorT> &frame) final { Q_EMIT onConsume(frame); }

public:
    // ── 采集测试（设备页的两个按钮）：只占本卡的锁，循环采集直到停止 ─────────────────────
    // 没有云台：朝向传空、同步位恒假，触发不了风廓线反演。同一时刻只跑一个测试；与编排的互斥由设备锁
    // 天然保证，UI 不做门控。
    // 官邸交入设置与会话名快照；族层不决定文件命名格式。目录仅在启用保存时转换。
    void startTest(const int accumSec, const bool save, const QUrl &folder, QString sessionDir)
    {
        if (testCoro_ && !testCoro_->done())
        {
            Q_EMIT this->logSource().logMessage(QStringLiteral("测试正在进行中"));
            return;
        }
        std::optional<QDir> saveRoot; // 空 = 不落盘
        if (save)
        {
            if (!folder.isLocalFile() || folder.toLocalFile().isEmpty())
            {
                Q_EMIT this->logSource().logMessage(QStringLiteral("选中的不是本机目录，未启动采集"));
                return;
            }
            saveRoot = QDir(folder.toLocalFile());
        }
        // 裸 new 刻意：make_unique 走转发必索 move（DeviceCoro 不可动），prvalue 直构保证省略。
        // 锁请求交给准备模块（promise 的构造函数）：本卡只占自己这一台。
        testCoro_.reset(new DeviceCoro(
            runTest(LockRequest{QStringLiteral("设备控制"), {*this}}, accumSec, std::move(saveRoot), std::move(sessionDir))));
    }
    // 预析构及停止按钮共用；取消时设备各层仍在，等待器可安全撤单。
    void cancelTest() noexcept { testCoro_.reset(); }

private:
    // 测试采集协程：取令牌 → 每轮 collectorFor 造原型，交等待器经 begin 启动；
    // 完成帧由设备自行提交处理器；启动时固定带 "Test" 后缀的会话目录名，各轮共用。
    DeviceCoro runTest(LockRequest, const int accumSec, const std::optional<QDir> saveRoot, const QString sessionDir)
    {
        // 试锁已由**准备模块**（promise 的构造函数）在本行之前做完；这里只取那枚令牌——begin 要出示它。
        const auto lock = co_await HeldLock{};
        if (!lock)
        {
            Q_EMIT this->logSource().logMessage(QStringLiteral("设备已被占用或未连接"));
            co_return;
        }
        while (true)
        {
            Q_EMIT this->logSource().logMessage(QString("开始采集（累积 %1 秒）%2")
                                 .arg(accumSec)
                                 .arg(saveRoot ? QStringLiteral("，落盘到 ") + QDir::toNativeSeparators(saveRoot->absolutePath())
                                               : QStringLiteral("，不落盘")));
            const auto framed = co_await RunAwaiter<DAQDeviceBase>{lock, *this,
                collectorFor(accumSec, {std::nullopt, saveRoot, sessionDir, false})};
            if (framed.empty())
            {
                Q_EMIT this->logSource().logMessage(QStringLiteral("采集失败 / 设备断开 / 已取消"));
                co_return;
            }
        }
    }

    // 本层预析构先等待 worker，再取消自有测试协程，最后接续父层清理。
    // 构造失败的线程兜底在本层析构；队列信号持有只读帧份额，以 worker 为接收上下文。
    QThread worker_;
    // wait 后由主线程析构处理器，不迁回线程归属；QThread 对象须比处理器晚销毁。
    const std::unique_ptr<DAQWorker<CollectorT>> sink_;

    // 预析构停止处理线程后撤测试协程；声明最末也使其先于其它族层成员销毁。
    std::unique_ptr<DeviceCoro> testCoro_;
};
