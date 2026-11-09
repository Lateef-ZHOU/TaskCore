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

#include "GimbalModel.h"                // WorldModel / PanTiltModel（世界朝向与云台位形）
#include "DeviceBase.h" // DeviceBase / Device<H,C>
#include "RunBase.h"    // DoneFn<C> / DeviceLock / CollectorLike
#include "GMBCollector.h"          // 云台族读口、运动参数与型号收集器（喂位形、判到位 + 驻留）

// 云台族定义标准能力；纯 C++ 旁支，不继承 QObject（唯一 QObject 来自末端继承的 Handle）。
// 模板层同时继承具体 Device 与本类，持有唯一安装姿态；编排只持族引用，通过 device() 使用锁，
// 不认识具体 Handle 或 Collector。Qt 属性、信号和 QML 入口由末端官邸声明。
class GMBDeviceBase
{
public:
    GMBDeviceBase() = default;
    Q_DISABLE_COPY_MOVE(GMBDeviceBase)
    virtual ~GMBDeviceBase() = default;

    // 本族的收集器基（`RunAwaiter<GMBDeviceBase>` 由此取账本的元素型——族基是非模板类，
    // 故这条别名就是等待器"存得下"的全部依据）。
    using CollectorBase = GMBCollector;
    // 等待器账本的请求槽：协程按束保存运动参数，族口 begin 据此造 Collector。
    using Request = GMBMoveParameters;

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

    // 同一对象的机制面：模板层返回自身的 DeviceBase 基类，供编排验锁和读取身份。
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
    [[nodiscard]] virtual bool setMounting(const WorldModel &zeroPose,
                                          const std::shared_ptr<const DeviceLock> &lock) = 0;

    // 借出当前安装标定；QML 创建时读取初值，后续直接接收 mountingChanged 的完整姿态。
    [[nodiscard]] virtual const WorldModel &mounting() const noexcept = 0;

    // ── 站点标定流程的默认值（标定页未做，暂无读者——用户定留着）────────────────
    // 四个测量点：水平角指令 0 / 90 / 180 / 270，隔点抬俯仰（tilt 0 / 激励 / 0 / 激励）；每点录入手机读到的
    // **全朝向**（方位 / 高度 / 翻滚三个角都要：SiteCalibration 吃相邻两点的相对转动，只有指向拼不出来）。
    // 水平角指令同时是录入框里方位角的默认（正北架设时手机本该读到的数），高度角录入框默认 0。
    // 激励角：相邻相对转轴不共线 → 手机偏差 R_offset 可解；全 0（只掰水平）退化成只解地基 R_base。
    // 条件数随激励增大而改善但边际递减、近 90° 撞万向锁，故限 [30, 60]；偏大越稳、偏小手机越不易滑。
    // 录入框的范围与零位默认不在这儿：范围是整圈（QML 里写死），零位默认由模板层的唯一标定值提供。
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
        return WorldModel::fromOrientation(mounting().toOrientation() * pose.toOrientation());
    }

    // 运动参数的型号校验与量化（取代原 collectorFor）：交入草稿（目标位形、停稳、速度），越出本台行程或
    // 速度不合本型号协议时返回空；成功则返回本次要提交的规范化参数，不造 Collector、不发令、不写日志以外的状态。
    // 任务协程在发令前对全部波束调用它，任一为空即拒绝启动；世界角 → 位形的换算由调用方经 gimbalOf 先做完。
    [[nodiscard]] virtual std::optional<GMBMoveParameters> paramsFor(const GMBMoveParameters &draft) const = 0;

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
        return PanTiltModel::fromOrientation(mounting().toOrientation().inverse() * worldTarget.toOrientation());
    }

    // 预览用的型号行程判据；末端以 Q_INVOKABLE 覆写供 QML 调用。
    [[nodiscard]] virtual bool withinTravel(double pan, double tilt) const = 0;

    // 按运动参数启动一次到位（等待器在挂起成立后调）。模板层据参数直接造出本型号的具体 Collector 交 Device，
    // 不经原型、验型或 makeSelf。返 false = 设备不收（参数不合本型号 / 被占 / 未连）。
    [[nodiscard]] virtual bool begin(const GMBMoveParameters &parameters,
                                     DoneFn<GMBCollector> done,
                                     const std::shared_ptr<const DeviceLock> &lock) = 0;

    // 停机 / 撤单（等待器析构时对未结的单调）。**与 begin 同一笔账**：编排层只认族基，而停机的实现住机制层
    // Device<>——族基不认识机制层，故本族声明停机虚口，由同时继承两者的模板层转到 Device::stop。
    // 可反复调用（幂等）：对交付 / 取消 / 拒收回滚三条出路一视同仁。
    virtual void stop(const std::shared_ptr<const DeviceLock> &lock) = 0;

    // 通知契约：末端在 Q_SIGNALS 中覆写，由 moc 提供实现；均在主线程发出。
    virtual void mountingChanged(const WorldModel &mounting) = 0;
    virtual void anglesReady(RxTime at, double pan, double tilt, double azimuth, double elevation) = 0;
};

// 模板层继承具体 Device 与族基，完成具体 Collector 验型及收包接入；型号只解析协议、检查行程和组装命令。
// 本层没有需要单独清理的线程源，不声明预析构；末端析构首句调用 Device 的预析构链。
template <HandleLike HandleT, GMBCollectorLike CollectorT>
class GMBDevice : public Device<HandleT, CollectorT>, public GMBDeviceBase
{
    using DeviceT = Device<HandleT, CollectorT>;
protected:
    GMBDevice(QString name, QString subtitle, QObject *parent)
        : DeviceT(std::move(name), std::move(subtitle), makeOnStream(this), {}, parent) {}

public:
    ~GMBDevice() override = default;

    // 协变返回自身的机制层基类：族引用只见 DeviceBase，持具体类型者可用完整 Device 接口。
    [[nodiscard]] DeviceT &device() noexcept final { return *this; }
    [[nodiscard]] const DeviceT &device() const noexcept { return *this; }

    [[nodiscard]] const WorldModel &mounting() const noexcept final { return mounting_; }
    [[nodiscard]] bool setMounting(const WorldModel &zeroPose,
                                   const std::shared_ptr<const DeviceLock> &lock) final
    {
        if (!this->lockMatches(lock))
        {
            Q_EMIT this->logMessage(QStringLiteral("设备正被占用，安装标定未写入"));
            return false;
        }
        mounting_ = zeroPose;
        Q_EMIT this->mountingChanged(mounting_);
        return true;
    }

    // 模板层完成 QML 值到强类型的边界转换；末端仅提供同名 Qt 槽。手动直发不等到位，停稳等待不用。
    bool issueGoTo(const double pan, const double tilt, const QVariantMap &values)
    {
        const auto parameters = GMBMoveParameters::fromQml(PanTiltModel(pan, tilt), {}, values);
        if (!parameters)
        {
            Q_EMIT this->logMessage(QStringLiteral("转台运动参数无效，未下发"));
            return false;
        }
        return issueGoTo(*parameters);
    }

    // 按参数直接造出本型号的具体 Collector 交 Device：无原型、无验型、启动时不 makeSelf。
    // 型号造斗时复用 paramsFor 的判据；参数不合本型号即拒收，拒绝原因由型号记录。
    [[nodiscard]] bool begin(const GMBMoveParameters &parameters,
                             DoneFn<GMBCollector> done,
                             const std::shared_ptr<const DeviceLock> &lock) final
    {
        auto collector = this->makeMoveCollector(parameters);
        if (!collector)
            return false;
        return DeviceT::start(std::move(collector), std::move(done), lock);
    }

    // 停机：族基的纯虚 stop 在此落地（final），实现就是机制层 Device::stop（它不是虚的，停机虚口只在族基）。
    void stop(const std::shared_ptr<const DeviceLock> &lock) final { DeviceT::stop(lock); }

private:

    // Device 独持此回调；构造时只记下 this，收包开始前不会调用。
    // 解析在收包线程；安装姿态只在主线程读取和换算。
    static typename DeviceT::OnStream makeOnStream(GMBDevice *self)
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

    // 本层另一个可变点（NVI，派生只实现、不调用）：照本次运动参数造出完整、可写的具体 Collector，
    // 把启动 / 停机指令、容差、驻留时长（斗里有下限）、时限交给 GMBCollector。形参的目标是**云台位形**，
    // 世界角换算由调用方经 gimbalOf 做完。参数不合本型号（与 paramsFor 同一判据）返空，begin 随即拒收。
    [[nodiscard]] virtual std::shared_ptr<CollectorT> makeMoveCollector(const GMBMoveParameters &parameters) const = 0;

    // 手动直发接收本次运动参数（目标位形与速度），不造 Collector、不等到位；
    // 失败由发现原因的层记录并返回 false，成功只返 true，由 QML 最终调用处记日志。
    // QML 通过官邸的同名 Qt 入口调用；世界角先用安装姿态值换算，不把文案传入设备。
    [[nodiscard]] virtual bool issueGoTo(const GMBMoveParameters &parameters) = 0;
    [[nodiscard]] virtual bool issueStop() = 0;

    // 唯一安装标定；主线程读写，族基通过 mounting() 使用。
    WorldModel mounting_;
};
