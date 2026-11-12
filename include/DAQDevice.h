#pragma once
#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <QDir>
#include <QString>
#include <QUrl>
#include <QVector>

#include "AcqSaver.h"             // 采集卡族唯一落盘入口
#include "DAQCollector.h"          // 通道元组与不可变请求参数
#include "DeviceCoro.h"           // DeviceCoro / LockRequest / HeldLock / RunAwaiter（测试采集协程）
#include "DeviceBase.h" // DeviceBase / Device<H,C>
#include "RunBase.h"    // DoneFn<C> / DeviceLock / CollectorLike
#include "WindProfile.h"          // 四束反演（标准方法）

// 采集卡族定义标准能力；纯 C++ 旁支，不继承 QObject（唯一 QObject 来自末端继承的 Handle）。
// 模板层同时继承具体 Device 与本类；编排只持族引用，通过 device() 使用锁，不认识具体 Handle 或 Collector。
// Collector 在收包线程收齐末包时完成标准处理；主线程发显示信号、提交独立线程保存，并反演风廓线。
// 请求为采集参数（协程按卡保存），账本元素为 DAQCollector；begin / stop / device() 由 FamilyBase 声明。
class DAQFamily : public FamilyBase<DAQParameters, DAQCollector>, protected DAQParamsKey
{
public:

    // pcap 读取政策：批量收谱采用 100 ms 读取超时与 4 MB 内核缓冲，具体帧尺寸由型号定义。
    // 缓冲可容纳的帧数随型号变化，不保证固定帧数或不丢包。末包解析与拟合占住收包线程期间，
    // 新到报文仍需缓冲；正常等待链在完成后才发下一请求，但看门狗可能在处理期间重试。
    // 因此时限需要涵盖收包及处理耗时，大帧、多卡下的余量需实测。
    static constexpr std::chrono::milliseconds kPcapReadTimeout{100};
    static constexpr int kPcapKernelBufferBytes = 4 * 1024 * 1024;

    // 由原始量构造本型号的采集参数：累积时间超出本型号范围时返回空并记录原因；不造 Collector、不发令。
    // 这是造出 DAQParameters 的唯一入口：型号以 *this 出示通行证。任务与单卡测试在发令前调用，为空即拒绝启动。
    [[nodiscard]] virtual std::optional<DAQParameters> paramsFor(int accumSec, std::optional<WorldModel> pose,
        std::optional<QDir> saveRoot, QString sessionDir, bool sync) const = 0;

    // 通知契约：末端在 Q_SIGNALS 中覆写，由 moc 提供实现；均在主线程按通道下标发出。
    virtual void spectrumReady(int channel, const LinearArray2D<double> &spectrum) = 0;
    // 逐门剖面带斜距线性轴；页面原样交给径向风图，图按创建配置核对网格。
    virtual void radialReady(int channel, const LinearArray<double> &speeds, const LinearArray<double> &cnr) = 0;
    virtual void profileReady(int channel, VerticalWindProfilePtr profile) = 0;
};

// 模板层继承具体 Device 与族基：Device 按参数造斗；本层在主线程显示、提交保存、维护四束环与反演，并提供单卡测试。
// 型号只给通道规格并按协议整理功率谱，不写处理或落盘代码。
template <HandleLike HandleT, DAQCollectorLike CollectorT>
class DAQDevice : public FamilyDevice<HandleT, CollectorT, DAQFamily>
{
    using DeviceT = Device<HandleT, CollectorT>;
protected:
    DAQDevice(QString name, QString subtitle, QObject *parent)
        : FamilyDevice<HandleT, CollectorT, DAQFamily>(std::move(name), std::move(subtitle), parent)
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
    // 完成帧由设备自行消费；会话目录名取自启动时交入的参数，各轮共用。
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
            const auto framed = co_await RunAwaiter<DAQFamily>{lock, *this, *parameters};
            if (framed.empty())
            {
                Q_EMIT this->logMessage(QStringLiteral("采集失败 / 设备断开 / 已取消"));
                co_return;
            }
        }
    }

    // [Handle 收包线程] 一口上行原样交给收集器；TypedDAQCollector 的 onFeed 调用型号 assemble 凑帧并构造通道。
    std::optional<QByteArray> onStream(const QByteArray &raw, RxTime) final { return raw; }

    // [主] 完成帧：按通道发功率谱与径向风、提交整帧保存，进四束环；同步位为真时逐通道反演风廓线。
    // 四束是同一收集器类型的帧，第 I 路在每帧都是同一类型（同一规格、同一距离轴），无须运行时核对。
    void onConsume(const std::shared_ptr<const CollectorT> &frame) override
    {
        constexpr auto indices = std::make_index_sequence<std::tuple_size_v<typename CollectorT::Frame>>{};
        const auto &context = frame->parameters;
        [&]<std::size_t... I>(std::index_sequence<I...>) {
            ((Q_EMIT this->spectrumReady(int(I), std::get<I>(frame->channels()).spectrum),
              Q_EMIT this->radialReady(int(I), std::get<I>(frame->channels()).radialWind, std::get<I>(frame->channels()).cnr)), ...);
        }(indices);
        if (context.saveRoot)
            AcqSaver::saveFrame(frame, this->objectName());
        beams_.push_back(frame);
        if (!context.sync || !beams_.full())
            return;
        std::vector<AcqSaver::ProfileChannel> profiles;
        [&]<std::size_t... I>(std::index_sequence<I...>) { (invert<I>(*frame, profiles), ...); }(indices);
        if (context.saveRoot)
            AcqSaver::saveWindProfiles(frame, std::move(profiles));
    }

    // 第 I 路：四束各取同一路径向风反演；朝向未知或网格不完整时 WindProfile 返回空、不反演。
    template <std::size_t I>
    void invert(const CollectorT &frame, std::vector<AcqSaver::ProfileChannel> &profiles)
    {
        using Channel = std::tuple_element_t<I, typename CollectorT::Frame>;
        std::array<WindProfile::Input, WindProfile::kBeams> inputs;
        for (std::size_t beam = 0; beam < inputs.size(); ++beam)
            inputs[beam] = {beams_[beam]->parameters.pose, std::get<I>(beams_[beam]->channels()).radialWind};
        auto profile = WindProfile::Calc(inputs, frame.frameTime());
        if (!profile)
            return;
        Q_EMIT this->profileReady(int(I), profile);
        profiles.push_back({QString::fromUtf8(Channel::name), std::move(profile)});
    }

    // 最近四束完成帧（主线程独写），满后新帧覆盖最旧一帧。
    boost::circular_buffer<std::shared_ptr<const CollectorT>> beams_{WindProfile::kBeams};
    std::unique_ptr<DeviceCoro> testCoro_;
};
