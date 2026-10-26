#pragma once
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <QByteArray>
#include <QMetaObject>
#include <QString>
#include <QVariantMap>

#include "Algorithm/Geometry/GimbalModel.h"                // WorldModel / PanTiltModel（世界朝向与云台位形）
#include "Device/Base/DeviceBase.h" // DeviceBase / Device<H,C>
#include "Device/Base/RunBase.h"    // DoneFn<C> / DeviceLock / CollectorLike
#include "Device/Base/RxTime.h"
#include "Device/GMB/GMBCollector.h"          // 云台族读口与型号收集器（喂位形、判到位 + 驻留）

// 一次运动的按值参数；无速度表示本型号的定位协议不携带速度。
// QML 仅在命令/任务启动边界交一份值，协程与工厂不回读界面。
struct GMBMoveParameters
{
    struct Speeds { double pan; double tilt; }; // °/s，两轴必须成组提供
    std::optional<Speeds> speeds;

    [[nodiscard]] static std::optional<GMBMoveParameters> fromQml(const QVariantMap &values)
    {
        if (values.isEmpty()) return GMBMoveParameters{};
        if (values.size() != 2 || !values.contains("panSpeed") || !values.contains("tiltSpeed"))
            return std::nullopt;
        bool panOk = false, tiltOk = false;
        const double pan = values.value("panSpeed").toDouble(&panOk);
        const double tilt = values.value("tiltSpeed").toDouble(&tiltOk);
        if (!panOk || !tiltOk || !std::isfinite(pan) || !std::isfinite(tilt)) return std::nullopt;
        return GMBMoveParameters{Speeds{pan, tilt}};
    }
};

// 云台族定义标准能力并持有安装姿态；具体官邸拥有一台 Device 运行机制。
// 编排只持族引用，通过 device() 使用锁，不认识具体 Handle 或 Collector。
class GMBDeviceBase : public QObject
{
    Q_OBJECT
    Q_PROPERTY(DeviceBase *device READ deviceObject CONSTANT)
    Q_PROPERTY(WorldModel mounting READ mounting NOTIFY mountingChanged)
    Q_PROPERTY(bool slipRing READ hasSlipRing CONSTANT)
public:
    explicit GMBDeviceBase(QObject *parent = nullptr) : QObject(parent) {}
    ~GMBDeviceBase() override = default;
    [[nodiscard]] DeviceBase *deviceObject() noexcept { return &device(); }

    // 本族的收集器基（`RunAwaiter<GMBDeviceBase>` 由此取账本的元素型——族基是非模板类，
    // 故这条别名就是等待器"存得下"的全部依据）。
    using CollectorBase = GMBCollector;

    // ── pcap 读取政策：**族级常量** ──────────────────────────────
    // 住族基而不是型号宪法：这两个数由**本族的流量形状**决定，不由型号决定——任何一台
    // 走网口的云台都是"小包、恒速、不成批"，换个厂家也一样。型号夹里只该留协议事实。
    // 串口云台用不上这两个数（PelcoDHandle 不读），留着是零成本的两个 constexpr。
    //
    //   · **20 ms 唤醒** = 上报周期（100 Hz）的两倍。延迟由它**独自**决定：Npcap 默认
    //     mintocopy = 16 KB，而本族 100 Hz × 约 80 B/包 ≈ 8 KB/s，攒满要两秒——**永远是
    //     超时先到**（实测坐实：to_ms = 100 时 113 ms 的团簇 100% 来自它）。
    //   · **1 MB 缓冲** = 约 128 秒余量。缓冲大小**不参与交付条件**（交付看 mintocopy ‖ 超时），
    //     调小它一毫秒延迟都省不下；两个旋钮是正交的——**降延迟靠更频繁地唤醒，防丢包靠更大的
    //     环**（用户定）。大环唯一的反方向风险是 bufferbloat，而那要求消费端被限速；本链的消费端
    //     是 worker 跑 pcap_loop、无节拍，吞吐远高于 100 包/秒，卡顿后毫秒级清空。
    //（别误用"积压是守恒量"那条——它说的是被事件循环限速的**主线程队列**，不是内核环。）
    static constexpr std::chrono::milliseconds kPcapReadTimeout{20};
    static constexpr int kPcapKernelBufferBytes = 1024 * 1024;

    // 借出本族持有的机制对象，用于锁与状态读取。
    [[nodiscard]] virtual DeviceBase &device() noexcept = 0;

    // ── 这台云台的方位轴上有没有滑环（铭辰有、视强没有）───────────────
    // **是族级特性，不是型号知识的泄漏**：每台云台都答得上（族级验收条件"无一字段是某成员填不出的"），
    // 而答案由官邸按自己的机械事实给。
    // 有滑环 = 方位可一直同向转，跨 360° 不必回绕；没有 = 线缆会绞，越过零点只能沿原路退回来。
    // **只影响编排与显示，不影响下发**：下发的是绝对位形，走哪条路是云台自己的事。今天的唯一读者是
    // 任务页（轮末那一笔怎么画、提示词怎么写）；将来自动排行进顺序也得看它。
    [[nodiscard]] virtual bool hasSlipRing() const noexcept = 0;


    // ── 安装标定：这台云台架在什么斜面上 ───────────────────────────────────────────
    // **它是设备的属性，不是任务的参数**：云台架好就不动，直到重新架设。
    // 形参是**世界系作动量**（三正交轴虚拟云台的 {方位, 高度, 翻滚}），即基座零位的朝向。
    // **这三个数都当真**：基座的翻滚是实打实的物理，歪着架就会让同一组 pan/tilt 指到别处去。
    // 出生态是水平正北架设（全零），故没标定过也跑得动——只是世界系与云台系重合。
    //
    // **验锁（lockMatches：空闲，或出示的正是持有者的令牌）**，返"写没写进去"。
    // **为什么要验**：同一束要读标定两次——去程（gimbalOf，造斗之前）与回程（worldOf，交付时换回
    // 世界朝向）。两次之间若被人改掉，去程回程用的是两份标定，风廓线里混进一束错参考系的数据，
    // 而且不响。
    // 空令牌 = "我不持锁"：云台空闲时照样写得进（没有 run 依赖它），有人持锁时一律拒。
    [[nodiscard]] bool setMounting(const WorldModel &zeroPose, const std::shared_ptr<const DeviceLock> &lock)
    {
        if (!device().lockMatches(lock))
        {
            // 族基借用的是 QObject 日志源，直接调用现有 Handle 信号，不增设设备中继。
            QMetaObject::invokeMethod(device().logSourceObject(), "logMessage", Qt::DirectConnection,
                                      Q_ARG(QString, QStringLiteral("设备正被占用，安装标定未写入")));
            return false; // 有人持锁、且不是出示者：此刻有 run 依赖这份标定
        }
        mounting_ = zeroPose;
        Q_EMIT mountingChanged(mounting_); // 直接交出完整姿态，订阅者无需回读设备
        return true;
    }

    // 借出当前安装标定；QML 创建时读取初值，后续直接接收 mountingChanged 的完整姿态。
    [[nodiscard]] const WorldModel &mounting() const noexcept { return mounting_; }

    // ── 站点标定流程的默认值（标定页未做，暂无读者——用户定留着）────────────────
    // 四个测量点：水平角指令 0 / 90 / 180 / 270，隔点抬俯仰（tilt 0 / 激励 / 0 / 激励）；每点录入手机读到的
    // **全朝向**（方位 / 高度 / 翻滚三个角都要：SiteCalibration 吃相邻两点的相对转动，只有指向拼不出来）。
    // 水平角指令同时是录入框里方位角的默认（正北架设时手机本该读到的数），高度角录入框默认 0。
    // 激励角：相邻相对转轴不共线 → 手机偏差 R_offset 可解；全 0（只掰水平）退化成只解地基 R_base。
    // 条件数随激励增大而改善但边际递减、近 90° 撞万向锁，故限 [30, 60]；偏大越稳、偏小手机越不易滑。
    // 录入框的范围与零位默认不在这儿：范围是整圈（QML 里写死），零位默认就是 mounting_ 的出生值。
    static constexpr double kCalibPan[4] = {0.0, 90.0, 180.0, 270.0};
    static constexpr double kCalibElevation = 0.0;
    static constexpr double kCalibTiltExcite = 40.0;
    static constexpr double kCalibTiltExciteMin = 30.0;
    static constexpr double kCalibTiltExciteMax = 60.0;

    // ── 正向：云台位形 → 世界朝向，R_世界 = R_安装 · R_云台 ─────────────────────────
    // 与 gimbalOf 一去一回，**两边都只认位形**：编排层先从斗上 `measured()` 取实测位形，再调本口。
    // 回程的翻滚是**被钉死的那个值**（双轴没有第三个自由度），WorldModel 如实带着它；风廓线只读
    // 方位与高度。实测俯仰越过 90°（如 MCZT 协议上沿 92°）照样换得出——那是位形环面的另一半。
    // **不验锁**：只读；它读到的标定之所以可信，是 setMounting 那一侧验锁保证的。角度图的实时
    // 世界角也走这里。
    [[nodiscard]] WorldModel worldOf(const PanTiltModel &pose) const
    {
        return WorldModel::fromOrientation(mounting_.toOrientation() * pose.toOrientation());
    }

    // 造斗：照一个**云台位形**造一只斗交出来——位形 → 收集器（它不让云台动，只造斗）。
    // 调用面 `co_await RunAwaiter{lock, gmb, gmb.collectorFor(位形, hold, parameters)}`。反方向（收集器 → 位形）是
    // 收集器自己的读口 `target()`（命令位形）/ `measured()`（实测位形）——转换所需的信息全在斗里，
    // 不在设备上另开一口。
    // **收位形**：世界角 → 位形归族基 gimbalOf，调用方先换，再造斗。
    // **越出本台行程就返空份额**：官邸造斗（makeMoveCollector）时查，`begin` 随即拒收 → 等待器整批
    // 判败 → 协程报失败。任务协程在设备发令前逐个造斗，任一为空即拒绝启动；
    // QML 的计划表只做角度换算和行程预览，不调用本工厂。
    // 本次运动参数由启动者交入；工厂不读取界面设置、不修改设备。
    // 返族基份额而非具体型：族基是非模板的，具体型在此必须擦掉——**擦到族基为止**，编排层的
    // 账本遂族级有型（等待器直接交回一支 `shared_ptr<const GMBCollector>` 的名单）。
    // hold 对应任务页的“停稳等待”，下限为 GMBCollector::kMinHold。首次进圈后中途出圈不重置计时，
    // 完成仍要求收到新的圈内回报，且与首次进圈的单调钟间隔达到 hold；型号不另加驻留常量。
    [[nodiscard]] virtual std::shared_ptr<const GMBCollector> collectorFor(const PanTiltModel &target,
                                                                         const std::chrono::milliseconds hold,
                                                                         const GMBMoveParameters &parameters) const = 0;

    // 世界角 → 云台位形（反解）：R_云台 = R_安装⁻¹ · R_世界(目标)，再由 `PanTiltModel::fromOrientation`
    // 解出。`worldOf` 的去程，**不造斗**（设备页的"前往世界角"只要位形、直发即可，不需要结果）。
    // 交出的是位形（水平 [0°,360°)、俯仰 [−90°,90°]）：目标的翻滚被忽略（用户定）。
    // **不查本台行程**：行程是型号的事，官邸造斗 / 组包时验这个位形在不在自己的范围内（"小解耦"：
    // 两边不互抄数）。**住族基**：它不碰 HandleT，只读安装标定，而标定是族级事实。
    // **目标翻滚忽略、恒取 |俯仰| ≤ 90° 那支**（用户定 10-08）：(p,t) 与 (p+180°,180°−t) 指向相同、朝向差一个
    // 绕光轴的 180°，对一束激光无所谓（光没有翻滚）；倒装也因此照样落在两台的行程里。**云台自己的翻滚
    // 不是自由量而是被钉死的**：双轴只有两个自由度，换成有朝向的载荷（相机像旋、偏振轴）就要命。
    [[nodiscard]] PanTiltModel gimbalOf(const WorldModel &worldTarget) const
    {
        return PanTiltModel::fromOrientation(mounting_.toOrientation().inverse() * worldTarget.toOrientation());
    }

    Q_INVOKABLE virtual bool withinTravel(double pan, double tilt) const = 0;

    // 启动一只本族的斗（等待器在挂起成立后调）。返 false = 设备不收（空斗 / 被占 / 未连 / 正忙）。
    // **本口是协议转换，不是薄转发**：机制层的启动口按 `Device` 的模板实参说话，而编排层只认
    // 族基——两套词汇之间要有一道翻译，翻译落在族层。这是"擦到族基为止"这条选择的账面成本。
    [[nodiscard]] virtual bool begin(const std::shared_ptr<const GMBCollector> &proto,
                                     DoneFn<GMBCollector> done,
                                     const std::shared_ptr<const DeviceLock> &lock) = 0;

    // 停机 / 撤单（等待器析构时对未结的单调）。**与 begin 同一笔账**：编排层只认族基，而停机的实现住机制层
    // Device<>——族基组合持有机制对象，故本族声明停机虚口，由族模板层转到机制层。
    // 可反复调用（幂等）：对交付 / 取消 / 拒收回滚三条出路一视同仁。
    virtual void stop(const std::shared_ptr<const DeviceLock> &lock) = 0;

Q_SIGNALS:
    void mountingChanged(const WorldModel &mounting);
    void anglesReady(RxTime at, double pan, double tilt, double azimuth, double elevation);

protected:
    virtual void PreDestruction() noexcept = 0;

private:
    // 安装标定：这台云台架在哪（世界系作动量；出生态 = 水平正北）。读写全在主线程，无需同步。
    WorldModel mounting_;
};

// 模板层完成具体 Collector 验型及收包接入；型号只解析协议、检查行程和组装命令。
template <HandleLike HandleT, GMBCollectorLike CollectorT>
class GMBDevice : public GMBDeviceBase
{
    using Core = Device<HandleT, CollectorT>;
protected:
    void PreDestruction() noexcept override { delete device_; }

public:
    GMBDevice(QString name, QString subtitle, QObject *parent = nullptr)
        : GMBDeviceBase(parent), device_(new Core(std::move(name), std::move(subtitle), makeOnStream(this), {}, this))
    {
        static_assert(DeviceLifecycleLayer<GMBDevice, decltype(&GMBDevice::PreDestruction)>);
    }

    [[nodiscard]] Core &device() noexcept final { return *device_; }
    [[nodiscard]] const Core &device() const noexcept { return *device_; }

    // 族口落地（final：管线是族级不变量，型号无权改）。本口只有"造哪只斗"这一个决定——交给官邸的
    // makeMoveCollector（它查本台行程并组装命令），本层只将其结果上转为族基份额。
    // **皮与可变点分开**：makeMoveCollector 是型号的私有可变点（NVI），本口是族基上的公开契约，
    // 型号已交出只读原型；本口仅擦到族类型，不泄露可写份额。
    // **公开而非私有**：它在族基上就是公开契约，拿具体官邸引用的调用点同样有资格调——覆写降级成
    // 私有会让访问检查按静态类型把它们挡在门外。NVI 的私有是给**可变点**留的，不是给族口留的。
    // 本口不碰几何：世界角 → 位形的换算与安装标定整个在族基 gimbalOf。
    [[nodiscard]] std::shared_ptr<const GMBCollector> collectorFor(const PanTiltModel &target,
                                                                   const std::chrono::milliseconds hold,
                                                                   const GMBMoveParameters &parameters) const final
    {
        return this->makeMoveCollector(target, hold, parameters);
    }

    // 模板层完成 QML 值到强类型的边界转换；末端仅提供同名 Qt 槽。
    bool issueGoTo(const double pan, const double tilt, const QVariantMap &values)
    {
        const auto parameters = GMBMoveParameters::fromQml(values);
        if (!parameters)
        {
            Q_EMIT device().logSource().logMessage(QStringLiteral("云台运动参数无效，未下发"));
            return false;
        }
        return issueGoTo(PanTiltModel(pan, tilt), *parameters);
    }

    // 族请求在入口核对型号，内部启动/重试均持具体 Collector。
    [[nodiscard]] bool begin(const std::shared_ptr<const GMBCollector> &proto,
                             DoneFn<GMBCollector> done,
                             const std::shared_ptr<const DeviceLock> &lock) final
    {
        const auto typed = std::dynamic_pointer_cast<const CollectorT>(proto);
        if (!typed)
        {
            Q_EMIT device().logSource().logMessage(QStringLiteral("云台请求的 Collector 型号不匹配，未启动"));
            return false;
        }
        return device().start(typed, std::move(done), lock);
    }

    // 停机：族基的纯虚 stop 在此落地（final），实现就是机制层 Device::stop（它不是虚的，停机虚口只在族基）。
    void stop(const std::shared_ptr<const DeviceLock> &lock) final { device().stop(lock); }

private:

    // Device 独持此回调。解析在收包线程；安装姿态只在主线程读取和换算。
    static typename Core::OnStream makeOnStream(GMBDevice *self)
    {
        return [self](const QByteArray &raw, const RxTime at) -> std::optional<PanTiltModel> {
            const auto pose = self->decodeStream(raw);
            if (!pose) return std::nullopt;
            QMetaObject::invokeMethod(self, [self, at, p = *pose] {
                const auto world = self->worldOf(p);
                Q_EMIT self->anglesReady(at, p.pan(), p.tilt(), world.azimuth(), world.elevation());
            }, Qt::QueuedConnection);
            return pose;
        };
    }

    // [worker] **型号该填**：一口上行 → 实测位形（云台系）。
    //   · **空** = 这一口没有可交的角度（不是角度上报 / 校验不过 / 两轴还没凑齐）；
    //     本口是族层接口，官邸在此摆位形合乎"位形只在接口处出现"。
    // **非 const**：型号要攒另一根轴（sqaf 的 Pelco 应答经、纬分帧），MCZT 还要记报警状态。
    [[nodiscard]] virtual std::optional<PanTiltModel> decodeStream(const QByteArray &) = 0;

    // 本层另一个可变点：照本次位形、驻留时长和运动参数造 Collector。
    // 私有虚 = NVI 可变点（派生只实现、不调用）。返回具体型号收集器：官邸把启动 / 停机指令、容差、
    // 驻留时长（调用方交的 hold，斗里有下限）、时限交给 GMBCollector。
    // 注意形参是**云台位形**不是世界朝向——换算那一步由调用方经族基 gimbalOf 做完。越出本台行程返空份额，
    // 而后 `begin` 拒收 → 等待器整批判败 → 协程报失败，是响的。
    [[nodiscard]] virtual std::shared_ptr<const CollectorT> makeMoveCollector(const PanTiltModel &target,
                                                                           const std::chrono::milliseconds hold,
                                                                           const GMBMoveParameters &parameters) const = 0;

    // 手动直发接收云台位形和本次运动参数，不造 Collector、不等到位；
    // 失败由发现原因的层记录并返回 false，成功只返 true，由 QML 最终调用处记日志。
    // QML 通过官邸的同名 Qt 入口调用；世界角先用安装姿态值换算，不把文案传入设备。
    [[nodiscard]] virtual bool issueGoTo(const PanTiltModel &target, const GMBMoveParameters &parameters) = 0;
    [[nodiscard]] virtual bool issueStop() = 0;

    // QObject 父子关系兜底构造失败；正常预析构先显式销毁，保证回调依赖仍在。
    // 删除期间保留指针值，retiring 同步撤单仍可访问 Device；没有可继续使用的预析构阶段。
    Core *const device_;
};
