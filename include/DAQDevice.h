#pragma once
#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <QDir>
#include <QString>
#include <QUrl>
#include <QVector>

#include "AcqSaver.h"             // 采集卡族唯一落盘入口
#include "DAQCollector.h"          // 通道数组与不可变消费上下文
#include "DeviceCoro.h"           // DeviceCoro / LockRequest / HeldLock / RunAwaiter（测试采集协程）
#include "DeviceBase.h" // DeviceBase / Device<H,C>
#include "RunBase.h"    // DoneFn<C> / DeviceLock / CollectorLike
#include "WindProfile.h"          // 四束反演（标准方法）

// 采集卡族定义标准能力；纯 C++ 旁支，不继承 QObject（唯一 QObject 来自末端继承的 Handle）。
// 模板层同时继承具体 Device 与本类；编排只持族引用，通过 device() 使用锁，不认识具体 Handle 或 Collector。
// 没有处理线程：Collector 在收包线程收齐末包时完成标准处理，完成帧在主线程显示、落盘并参与风廓线反演。
// 请求为采集参数（协程按卡保存），账本元素为 DAQCollector；begin / stop / device() 由 FamilyBase 声明。
class DAQDeviceBase : public FamilyBase<DAQParameters, DAQCollector>
{
public:

    // ── pcap 读取政策：**族级常量** ────────
    // 住族基而不是型号宪法：这两个数由**本族的流量形状**决定——帧是成批爆发的，
    // 一帧上百个包、二三十万字节（各卡的帧常量见各自的 RunSide），且**判齐后整帧才有意义**，丢一个包
    // 整帧作废。与云台族恰好相反的取舍：**可以慢，不能丢**。
    //
    //   · **100 ms 唤醒**：成批交付正合适，本族的时间戳按帧算、不按包算，批量无害。
    //     （实际上多半是 mintocopy 先到——16 KB 在线速下瞬间攒满，这个数很少真用上。）
    //   · **4 MB 缓冲**：十帧以上的余量。**真约束是爆发**：一帧线速打出，硬下限是一帧。
    //     收末包后的处理占住收包线程期间，迟到包同样由它兜住（下一次 START 要等本帧完成）。
    //
    // 100 ms 是长期实跑过的值，缓冲比当时的 1 MB 更宽——用户据此认为本族无需再验。
    static constexpr std::chrono::milliseconds kPcapReadTimeout{100};
    static constexpr int kPcapKernelBufferBytes = 4 * 1024 * 1024;

    // 由原始量构造本型号的采集参数：累积时间超出本型号范围时返回空并记录原因；不造 Collector、不发令。
    // 任务与单卡测试在发令前调用，任一为空即拒绝启动。
    [[nodiscard]] virtual std::optional<DAQParameters> paramsFor(int accumSec, std::optional<WorldModel> pose,
        std::optional<QDir> saveRoot, QString sessionDir, bool sync) const = 0;

    // 通知契约：末端在 Q_SIGNALS 中覆写，由 moc 提供实现；均在主线程按通道下标发出。
    virtual void spectrumReady(int channel, const LinearArray2D<double> &spectrum) = 0;
    // 逐门剖面带斜距线性轴；页面原样交给径向风图，图按创建配置核对网格。
    virtual void radialReady(int channel, const LinearArray<double> &speeds, const LinearArray<double> &cnr, bool calibrated) = 0;
    virtual void profileReady(int channel, VerticalWindProfilePtr profile) = 0;
};

// 模板层继承具体 Device 与族基：按参数造斗、主线程消费（显示、落盘、四束环与风廓线）及单卡测试协程。
// 型号只给通道规格并按协议整理功率谱，不写处理或落盘代码。
template <HandleLike HandleT, DAQCollectorLike CollectorT>
class DAQDevice : public FamilyDevice<HandleT, CollectorT, DAQDeviceBase>
{
    using DeviceT = Device<HandleT, CollectorT>;
protected:
    DAQDevice(QString name, QString subtitle, QObject *parent)
        : FamilyDevice<HandleT, CollectorT, DAQDeviceBase>(std::move(name), std::move(subtitle), parent)
    {
        static_assert(DeviceLifecycleLayer<DAQDevice, decltype(&DAQDevice::PreDestruction)>);
    }

    // 本层的访问源是测试协程：它借用设备锁和等待器，先于 Device 层撤销。
    void PreDestruction() noexcept override
    {
        cancelTest();
        DeviceT::PreDestruction();
    }

public:
    ~DAQDevice() override = default;

    // 按参数经型号静态工厂直接造出完整的具体 Collector 交 Device：无原型、无验型、启动时不 makeSelf。
    // begin 是公开入口，先以 paramsFor 同一判据复核，不合即拒收（原因由型号记录）。
    [[nodiscard]] bool begin(const DAQParameters &parameters,
                             DoneFn<DAQCollector> done,
                             const std::shared_ptr<const DeviceLock> &lock) final
    {
        const auto checked = this->paramsFor(parameters.accumSec, parameters.pose, parameters.saveRoot,
                                       parameters.sessionDir, parameters.sync);
        return checked && DeviceT::start(CollectorT::create(*checked), std::move(done), lock);
    }

    // ── 采集测试（设备页的两个按钮）：只占本卡的锁，循环采集直到停止 ─────────────────────
    // 没有云台：朝向传空、同步位恒假，触发不了风廓线反演。同一时刻只跑一个测试；与编排的互斥由设备锁
    // 天然保证，UI 不做门控。
    // 官邸交入设置与会话名快照；族层不决定文件命名格式。目录仅在启用保存时转换。
    void startTest(const int accumSec, const bool save, const QUrl &folder, QString sessionDir)
    {
        if (testCoro_ && !testCoro_->done())
        {
            Q_EMIT this->logMessage(QStringLiteral("测试正在进行中"));
            return;
        }
        std::optional<QDir> saveRoot; // 空 = 不落盘
        if (save)
        {
            if (!folder.isLocalFile() || folder.toLocalFile().isEmpty())
            {
                Q_EMIT this->logMessage(QStringLiteral("选中的不是本机目录，未启动采集"));
                return;
            }
            saveRoot = QDir(folder.toLocalFile());
        }
        // 没有云台：朝向为空、同步位恒假；参数不合本型号即不起协程。
        auto parameters = this->paramsFor(accumSec, std::nullopt, std::move(saveRoot), std::move(sessionDir), false);
        if (!parameters)
            return;
        // 裸 new 刻意：make_unique 走转发必索 move（DeviceCoro 不可动），prvalue 直构保证省略。
        // 锁请求交给准备模块（promise 的构造函数）：本卡只占自己这一台。
        testCoro_.reset(new DeviceCoro(runTest(LockRequest{QStringLiteral("设备控制"), {*this}}, std::move(parameters))));
    }
    // 预析构及停止按钮共用；取消时设备各层仍在，等待器可安全撤单。
    void cancelTest() noexcept { testCoro_.reset(); }

private:
    // 测试采集协程：取令牌 → 每轮交等待器一份采集参数，经 begin 启动；
    // 完成帧由设备自行消费；启动时固定带 "Test" 后缀的会话目录名，各轮共用。
    DeviceCoro runTest(LockRequest, const std::optional<DAQParameters> parameters)
    {
        // 试锁已由**准备模块**（promise 的构造函数）在本行之前做完；这里只取那枚令牌——begin 要出示它。
        const auto lock = co_await HeldLock{};
        if (!lock)
        {
            Q_EMIT this->logMessage(QStringLiteral("设备已被占用或未连接"));
            co_return;
        }
        while (true)
        {
            Q_EMIT this->logMessage(QString("开始采集（累积 %1 秒）%2")
                                 .arg(parameters->accumSec)
                                 .arg(parameters->saveRoot ? QStringLiteral("，落盘到 ") + QDir::toNativeSeparators(parameters->saveRoot->absolutePath())
                                                           : QStringLiteral("，不落盘")));
            const auto framed = co_await RunAwaiter<DAQDeviceBase>{lock, *this, *parameters};
            if (framed.empty())
            {
                Q_EMIT this->logMessage(QStringLiteral("采集失败 / 设备断开 / 已取消"));
                co_return;
            }
        }
    }

    // [Handle 收包线程] 采集卡的一口上行原样交给收集器，按协议整理归型号的 onFeed。
    std::optional<QByteArray> onStream(const QByteArray &raw, RxTime) final { return raw; }

    // [主] 完成帧：按通道发功率谱与径向风、整帧落盘，进四束环；同步位为真时逐通道反演风廓线。
    // 一组四束须是同一通道、同一距离轴且均已换算风速；任一束朝向未知则不反演。
    void onConsume(const std::shared_ptr<const CollectorT> &frame) override
    {
        const auto &context = frame->parameters();
        for (std::size_t i = 0; i < frame->size(); ++i)
        {
            const auto &channel = (*frame)[i];
            Q_EMIT this->spectrumReady(int(i), channel.spectrum);
            Q_EMIT this->radialReady(int(i), channel.radialWind, channel.cnr,
                                         RadialWind::inBand(channel.spectrum, channel.zeroWindMHz));
        }
        if (context.saveRoot)
            AcqSaver::saveFrame(frame, this->objectName());
        beams_.push_back(frame);
        if (!context.sync || !beams_.full())
            return;
        std::vector<AcqSaver::ProfileChannel> profiles;
        for (std::size_t i = 0; i < frame->size(); ++i)
        {
            const auto &channel = (*frame)[i];
            std::array<WindProfile::Input, WindProfile::kBeams> inputs;
            bool usable = true;
            for (std::size_t beam = 0; beam < inputs.size(); ++beam)
            {
                const auto &past = (*beams_[beam])[i];
                usable = usable && RadialWind::inBand(past.spectrum, past.zeroWindMHz)
                         && past.radialWind.sameAxis(channel.radialWind);
                inputs[beam] = {beams_[beam]->parameters().pose, past.radialWind};
            }
            if (!usable)
                continue;
            auto profile = WindProfile::Calc(inputs, frame->frameTime());
            if (!profile)
                continue;
            Q_EMIT this->profileReady(int(i), profile);
            profiles.push_back({channel.name, std::move(profile)});
        }
        if (context.saveRoot)
            AcqSaver::saveWindProfiles(frame, std::move(profiles));
    }

    // 最近四束完成帧（主线程独写），满后新帧覆盖最旧一帧。
    boost::circular_buffer<std::shared_ptr<const CollectorT>> beams_{WindProfile::kBeams};
    std::unique_ptr<DeviceCoro> testCoro_;
};
