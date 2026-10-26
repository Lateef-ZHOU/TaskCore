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

// ============================================================
// GMBDevice.h — **云台族**的两层：纯接口族基 GMBDeviceBase（编排层拿的就是它）+ 模板族层
// GMBDevice<HandleT, CollectorT>（族级实现写一次）。
//
// **为什么要有族，以及族分在哪**：编排层要能对"任意型号、任意台数"的云台下单，那就得有一个
// 能同时指向 SqafDevice 与 McztDevice 的指针 —— 即类型擦除，即非模板基类。而"是云台还是采集
// 卡"这件事编排层**必须**知道（任务组合一台云台和多选采集卡），"是哪个型号"则**不该**
// 知道（知道了就被型号污染，加一个型号就要改编排层）。族基正落在这两句话之间。
//
// **族是旁挂的，不串在设备链上**（用户定）：
//     GMBDeviceBase（不继承任何东西）
//     Device<HandleT, CollectorT> : DeviceBase : QObject      ← 机制链，一个字不动
//     GMBDevice<HandleT, CollectorT> : Device<H, CollectorT>, GMBDeviceBase  ← 在此汇合
//     官邸 : GMBDevice<...>
// **不菱形**：族基不继承 DeviceBase，故官邸只有一份 DeviceBase 子对象。**QObject 也只有一份**
// （Qt 禁止多重继承 QObject；族基不是 QObject，且 QObject 那条基必须写在前面）。
// 族基通过 `device()` 交出 DeviceBase 面以访问锁；`begin` / `stop` 由本族声明，
// 族模板层分别接到 Device 的启动与撤单机制。
// 族基不是 QObject，族级通知以纯虚函数声明，由带 Q_OBJECT 的官邸用同名 Qt 信号覆写，
// moc 提供发射实现；模板层继承该契约，不把业务通知塞进 DeviceBase，也不增加转发层。
// （不做成串链形 `Device<H,C,BaseT=GMBDeviceBase>`——单链、族基指针免费上行、族基还能带
// Q_OBJECT——是因为机制层的基类可配置这件事本身不可接受，且那套形制在本仓库无先例、扩展性
// 未知（用户定）。）
//
// 族口统一定位与完成结果；运动参数按值交入，定位协议不带速度的型号使用空参数。
// 扇扫、循环和方向点动仍是型号专有命令，不进入任务编排。
// ============================================================

// ── GMBDeviceBase：云台族的**契约**（不继承任何东西 = 编排层的擦除口）。各口都不碰 HandleT，
//    故本类不必是模板：
//      · device —— 交出自己的 DeviceBase 面。**旁挂形制的必需口**：编排层拿族基引用要加锁
//        （锁请求收 DeviceBase&），而族基不在那条链上，只能由实现方把 this 交出来。
//      · collectorFor —— 按云台位形创建只读原型，不启动设备；越出本台行程由
//        官邸拒收（空份额）。编排层把它连同本族基一起交给 `RunAwaiter<GMBDeviceBase>`。
//      · begin / stop —— 启动那只斗 / 撤单。等待器在挂起成立后调 begin 一次（返"设备收不收"），
//        析构时对未结的单调 stop。
//      · setMounting / mounting —— 安装标定（写口验锁）。
//      · gimbalOf / worldOf —— 世界朝向 ⇄ 云台位形（含安装标定），一去一回，**两边都只认位形**。
//      · hasSlipRing —— 方位轴有没有滑环（机械事实，官邸答）。
//
// **几何换算住本类**（用户定：层次更准）：`mounting_` 私有持有，`gimbalOf`（反解）/ `worldOf`
// （正向）/ `setMounting` 都是非虚的族级实现——它们不碰 HandleT。
// setMounting 经 device().lockMatches 验锁，与其他设置写口共用机制层谓词。
//
// **收集器是收集器，位形是位形**（用户定）：族口里没有"收斗、在里面取位形"的混合口——
// `collectorFor` 收位形，从斗取位形（`measured()` / `target()`）是调用方的事；
// 世界角 → 位形（gimbalOf）→ 造斗（collectorFor，官邸查行程）各是一步。
// **族基不碰角度数值**（用户定）：本类只经手 `WorldModel` / `PanTiltModel` 两个类型，不读其中
// 任何一个 double。所以它也不替任务排行进顺序——那由人工在任务页排；将来要自动排，归官邸。
class GMBDeviceBase
{
public:
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

    // 交出 DeviceBase 面（族模板层一行实现：`return *this;` 走隐式上行）。
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

    // 启动一只本族的斗（等待器在挂起成立后调）。返 false = 设备不收（空斗 / 被占 / 未连 / 正忙）。
    // **本口是协议转换，不是薄转发**：机制层的启动口按 `Device` 的模板实参说话，而编排层只认
    // 族基——两套词汇之间要有一道翻译，翻译落在族层。这是"擦到族基为止"这条选择的账面成本。
    [[nodiscard]] virtual bool begin(const std::shared_ptr<const GMBCollector> &proto,
                                     DoneFn<GMBCollector> done,
                                     const std::shared_ptr<const DeviceLock> &lock) = 0;

    // 停机 / 撤单（等待器析构时对未结的单调）。**与 begin 同一笔账**：编排层只认族基，而停机的实现住机制层
    // Device<>——族基不在那条链上（旁挂形制），故本族声明停机虚口，由族模板层转到机制层。
    // 可反复调用（幂等）：对交付 / 取消 / 拒收回滚三条出路一视同仁。
    virtual void stop(const std::shared_ptr<const DeviceLock> &lock) = 0;

protected:
    GMBDeviceBase() = default;
    // 非虚 + protected：族接口只供借用；具体后端由对应 QML 官邸通过 QObject 所有权持有。
    // protected 封死 `delete (GMBDeviceBase *)p`，非虚遂安全——同 Collector 的判例。
    ~GMBDeviceBase() = default;

    // [主] 每次成功写入完整安装姿态后通知；拒绝写入不发。官邸声明为同名 Qt 信号。
    virtual void mountingChanged(const WorldModel &mounting) = 0;

private:
    // 安装标定：这台云台架在哪（世界系作动量；出生态 = 水平正北）。读写全在主线程，无需同步。
    WorldModel mounting_;
};

// ── GMBDevice<HandleT, CollectorT>：云台族的**实现层**（模板 = 族级样板写一次）。
//    **QObject 那条基必须写在前面**（Qt 规矩），族接口旁挂在后。
//    族口在此一律 final：`device` / `collectorFor` / `begin` / `stop`（`gimbalOf` / `worldOf` /
//    `setMounting` 住族基、非虚），以及常态灶 `onStream`（画角度图、交回喂斗的位形）。这些是
//    **族级管线与机制层词汇，官邸本不该认识**。
//    官邸的可变点：`makeMoveCollector`（给我一只斗）、`decodeStream`（把一口上行
//    解开）和 `issueGoTo` / `issueStop`（设备页直发怎么发）；页面与分区由 QML 官邸组织。
// 族口借出公共位形读口；具体型号运行类型在 begin 验型后保持到完成。
template <HandleLike HandleT, GMBCollectorLike CollectorT>
class GMBDevice : public Device<HandleT, CollectorT>, public GMBDeviceBase
{
protected:
    using Base = Device<HandleT, CollectorT>;

    void PreDestruction() noexcept override { Base::PreDestruction(); }

public:
    explicit GMBDevice(QObject *parent = nullptr) : Base(parent)
    {
        static_assert(DeviceLifecycleLayer<GMBDevice, decltype(&GMBDevice::PreDestruction)>);
    }

    // 旁挂形制的接线：本类**是**一台 Device，故 `*this` 隐式上行即得 DeviceBase 面。
    [[nodiscard]] DeviceBase &device() noexcept final { return *this; }

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
            Q_EMIT this->logSource().logMessage(QStringLiteral("云台运动参数无效，未下发"));
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
            Q_EMIT this->logSource().logMessage(QStringLiteral("云台请求的 Collector 型号不匹配，未启动"));
            return false;
        }
        return this->start(typed, std::move(done), lock);
    }

    // 停机：族基的纯虚 stop 在此落地（final），实现就是机制层 Device::stop（它不是虚的，停机虚口只在族基）。
    void stop(const std::shared_ptr<const DeviceLock> &lock) final { Base::stop(lock); }

private:

    // GMB 连续回显已由 onStream 处理；到位不再绘图或提交处理器。
    void onCompleted(const std::shared_ptr<const CollectorT> &) final {}

    // ── 常态灶：本层接管（用户定）──────────────────────────────────
    // [worker] 模板层 `onStream` 的族级落地，**final**：每一口上行都过，有没有 run 都照跑。
    // 型号只做一件事：`decodeStream` 把字节解成**实测位形**。本层拿它做两件事：
    //   · 投回主线程换世界角、画角度图（设备页的实时角度也取自那条通路）；
    //   · 返回给机制层喂斗（喂料类型 = GMBCollector::Feed = PanTiltModel）。
    //
    // 每口有效位形都投递、都喂斗，不在这里抽样；GMBCollector 完成等待仍需新的圈内回报。
    // SeriesChart 的显示抽稀由绘图层负责，不改变交给 Collector 的样本。
    // **final 的代价**：官邸不能覆写常态灶；型号的额外解析（如 MCZT 的 BIT 报警日志）在 decodeStream 里做。
    [[nodiscard]] std::optional<PanTiltModel> onStream(const QByteArray &raw, const RxTime at) final
    {
        auto pose = decodeStream(raw);
        if (!pose)
            return std::nullopt;
        // **时间戳不在这里盖**：`at` 是 Handle 层交上来的到料时刻——网口是网卡驱动在收包回调里
        // 盖的章，串口是帧首那块料的时刻（在链路末端盖章会把内核缓冲、定时唤醒、线程调度的延迟
        // 全算进去，见 RxTime）。
        // 世界角换算要读 mounting_ [主]，故投回主线程再算。
        // 上下文对象 = 本设备（QObject）：设备先亡则排队中的调用随之作废，捕获裸 this 由此成立（receiver = 自身）。
        // 每一格都投递，不节流；按值携带到料时刻，排队延迟不改变采样时间。
        QMetaObject::invokeMethod(
            this, [this, at, p = *pose] {
                const WorldModel world = this->worldOf(p);
                Q_EMIT anglesReady(at, p.pan(), p.tilt(), world.azimuth(), world.elevation());
            }, Qt::QueuedConnection);
        return pose; // 同一格位形喂斗
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

    // [主] 一个回显采样 → 换算世界角 → 发布给读数与图表。
    // 官邸交来的就是位形（decodeStream），世界角走族基 worldOf（正向）。
    virtual void anglesReady(RxTime at, double pan, double tilt, double azimuth, double elevation) = 0;

};
