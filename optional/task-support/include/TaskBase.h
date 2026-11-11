#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <QDateTime>
#include <QDir>
#include <QObject>
#include <QStringList>
#include <QUrl>
#include <QVariantMap>
#include "DeviceCoro.h"
#include "AcqSaver.h"
#include "DAQDevice.h"
#include "GMBDevice.h"

// 任务族基只管理设备登记、启动校验、协程及运行状态。
// 可编辑设置由 QML 官邸持有，启动时一次传入；运行期间不回读界面。
// 设备退出时同步收回协程并移出候选；任务末端析构首句 cancelRun()。
class TaskBase : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString displayName READ displayName CONSTANT)
    Q_PROPERTY(QString subtitle READ subtitle CONSTANT)
    Q_PROPERTY(QVariantMap settingsParameters READ settingsParameters CONSTANT)
    Q_PROPERTY(int beamCount READ beamCount CONSTANT)
    Q_PROPERTY(bool running READ running NOTIFY runChanged)
    Q_PROPERTY(QString phaseText READ phaseText NOTIFY runChanged)
    Q_PROPERTY(QString beamText READ beamText NOTIFY runChanged)
    Q_PROPERTY(QString deliveredText READ deliveredText NOTIFY runChanged)
    Q_PROPERTY(QString scopeText READ scopeText NOTIFY runChanged)
    Q_PROPERTY(QString sinceText READ sinceText NOTIFY runChanged)
    Q_PROPERTY(int currentBeam READ currentBeam NOTIFY runChanged)

public:
    using Cards = std::vector<std::reference_wrapper<DAQFamily>>;
    explicit TaskBase(QObject *parent = nullptr) : QObject(parent) {}
    ~TaskBase() override = default;

    [[nodiscard]] virtual QString displayName() const = 0;
    [[nodiscard]] virtual QString subtitle() const = 0;
    [[nodiscard]] virtual int beamCount() const noexcept = 0;

    // 创建参数只交默认值与边界，不保存表单的当前值。
    [[nodiscard]] static QVariantMap settingsParameters()
    {
        return {{"settleMin", kSettleMin}, {"settleMax", kSettleMax}, {"settleDefault", kSettleMin},
                {"accumMin", 1}, {"accumMax", 3600}, {"accumDefault", 1},
                {"speedMin", 0.1}, {"speedMax", 360.0}, {"speedDefault", 10.0}, {"saveDirDefault", QUrl::fromLocalFile(QDir::currentPath())}};
    }

    // 任务只登记族对象身份；运行参数由任务官邸按次提交。
    // 云台族基不是 QObject：QML 交入设备对象本身，在此交叉转换验族，非云台拒收。
    Q_INVOKABLE bool addGimbal(QObject *object)
    {
        if (!dynamic_cast<GMBFamily *>(object) || std::ranges::find(gimbals_, object) != gimbals_.end()) return false;
        gimbals_.push_back(object);
        // destroyed 时族基子对象已析构，此处只比较指针；运行借用已由设备预析构的锁铃收回。
        connect(object, &QObject::destroyed, this, [this, object] {
            cancelRun();
            std::erase(gimbals_, object);
            Q_EMIT deviceRemoved(object);
            refreshRunState();
        });
        return true;
    }
    // 采集卡族基同样不是 QObject：QML 交入设备对象本身，在此交叉转换验族，非采集卡拒收。
    Q_INVOKABLE bool addCard(QObject *object)
    {
        if (!dynamic_cast<DAQFamily *>(object) || std::ranges::find(cards_, object) != cards_.end()) return false;
        cards_.push_back(object);
        // destroyed 时族基子对象已析构，此处只比较指针；运行借用已由设备预析构的锁铃收回。
        connect(object, &QObject::destroyed, this, [this, object] {
            cancelRun();
            std::erase(cards_, object);
            Q_EMIT deviceRemoved(object);
            refreshRunState();
        });
        return true;
    }

    // 初次进度在急启动协程落槽前发出，装槽期间也算运行中。
    [[nodiscard]] bool running() const { return starting_ || (coro_ && !coro_->done()); }
    [[nodiscard]] QDateTime startedAt() const { return startedAt_; }

public Q_SLOTS:
    bool start(QObject *gimbal, const QList<QObject *> &cards, QList<double> worldAzimuths,
               const double worldElevation, const int accumSec, const int settleSec,
               const bool save, const QUrl &folder, const QVariantMap &movement)
    {
        if (running()) { Q_EMIT logMessage(QStringLiteral("任务正在进行中")); return false; }
        // 先按身份查登记，只对已登记的活对象交叉转换：调用方可能交来已退场设备的旧指针。
        auto *gmb = std::ranges::find(gimbals_, gimbal) != gimbals_.end() ? dynamic_cast<GMBFamily *>(gimbal) : nullptr;
        if (!gmb) {
            Q_EMIT logMessage(QStringLiteral("没有可用的已登记转台")); return false;
        }
        // 停稳与速度在启动时固定；协程逐束以目标位形和它们经官邸 paramsFor 构造运动参数。
        const auto speeds = GMBMoveParameters::speedsFromQml(movement);
        if (!speeds) { Q_EMIT logMessage(QStringLiteral("转台运动参数无效，任务未启动")); return false; }
        Cards picked;
        QStringList scope{gmb->device().displayName + QStringLiteral(" · ") + gmb->device().subtitle};
        for (auto *object : cards)
        {
            auto *card = std::ranges::find(cards_, object) != cards_.end() ? dynamic_cast<DAQFamily *>(object) : nullptr;
            if (!card
                || std::ranges::any_of(picked, [card](const auto &entry) { return &entry.get() == card; })) {
                Q_EMIT logMessage(QStringLiteral("采集卡未登记或重复选择，任务未启动")); return false;
            }
            // 发令前按型号预验累积时间；朝向与会话逐束确定，begin 会以同一判据复核。
            if (!card->paramsFor(accumSec, std::nullopt, std::nullopt, {}, false)) return false;
            picked.emplace_back(*card);
            const auto &device = card->device();
            scope << device.displayName + QStringLiteral(" · ") + device.subtitle;
        }
        if (picked.empty()) { Q_EMIT logMessage(QStringLiteral("至少选一张采集卡")); return false; }
        if (settleSec < kSettleMin || settleSec > kSettleMax) {
            Q_EMIT logMessage(QStringLiteral("停稳等待超出范围，任务未启动")); return false;
        }
        std::optional<QDir> saveRoot;
        if (save) {
            if (!folder.isValid() || !folder.isLocalFile() || folder.toLocalFile().isEmpty()) {
                Q_EMIT logMessage(QStringLiteral("保存目录不是有效的本机目录，任务未启动")); return false;
            }
            saveRoot.emplace(folder.toLocalFile());
        }
        if (worldAzimuths.size() != beamCount() || !std::isfinite(worldElevation)
            || !std::all_of(worldAzimuths.cbegin(), worldAzimuths.cend(), [](double az) { return std::isfinite(az); }))
        {
            Q_EMIT logMessage(QStringLiteral("世界角参数无效：束数不符或角度不是有限数"));
            return false;
        }
        std::vector<std::reference_wrapper<DeviceBase>> devices{gmb->device()};
        for (DAQFamily &card : picked) devices.emplace_back(card.device());
        LockRequest request{displayName(), std::move(devices)};
        delivered_ = 0;
        scopeText_ = scope.join(QStringLiteral("；"));
        // DeviceCoro 不可移动，同类型 prvalue 直接构造；不能用 make_unique 转发。
        starting_ = true;
        coro_.reset(new DeviceCoro(run(std::move(request), *gmb, std::move(picked), std::move(worldAzimuths),
                                      worldElevation, accumSec, std::chrono::seconds(settleSec), *speeds, std::move(saveRoot))));
        starting_ = false;
        const bool acquired = coro_->locked();
        refreshRunState();
        return acquired;
    }
    void stop()
    {
        if (!running()) { Q_EMIT logMessage(QStringLiteral("没有在跑的任务")); return; }
        cancelRun();
        Q_EMIT logMessage(QStringLiteral("任务已停止"));
        refreshRunState();
    }

Q_SIGNALS:
    void logMessage(const QString &text) const;
    void deviceRemoved(QObject *device);
    void runChanged();

protected:
    // 协程退出时排队回读 done，不借用设备状态事件推断任务是否结束。
    void refreshRunState()
    {
        if (!running()) { beam_ = -1; phase_ = Phase::Idle; delivered_ = 0; }
        Q_EMIT runChanged();
    }
    using GmbOrders = RunAwaiter<GMBFamily>::Orders;
    // hold 与 speeds 为启动时固定的快照；协程逐束连同目标位形交官邸 paramsFor 构造运动参数。
    virtual DeviceCoro run(LockRequest, GMBFamily &, Cards, QList<double>, double, int,
                          std::chrono::milliseconds hold, std::optional<GMBMoveParameters::Speeds> speeds,
                          std::optional<QDir>) = 0;

    // 不发业务状态通知，不调用虚函数；末端析构在自己的成员拆除前调用。
    void cancelRun() noexcept { coro_.reset(); }
    [[nodiscard]] QDateTime beginSession() { return startedAt_ = QDateTime::currentDateTime(); }
    void completeBeam() noexcept { ++delivered_; }
    // 只构造等待器，共用本帧的锁/取消入口；co_await 和失败出口在每个具体协程内。
    [[nodiscard]] RunAwaiter<GMBFamily> moveTo(const std::shared_ptr<DeviceLock> &lock,
        GMBFamily &gmb, const GMBMoveParameters &parameters, const int beam)
    {
        const auto &target = parameters.target;
        step(beam, Phase::Moving,
             QString("转台 前往 (水平 %1°, 俯仰 %2°)...").arg(target.pan(), 0, 'f', 2).arg(target.tilt(), 0, 'f', 2));
        return RunAwaiter<GMBFamily>{lock, gmb, parameters};
    }
    [[nodiscard]] RunAwaiter<DAQFamily> acquireAt(const std::shared_ptr<DeviceLock> &lock,
        GMBFamily &gmb, const Cards &cards, const GmbOrders &moved, const int beam, const int accumSec,
        const std::optional<QDir> &saveRoot, const QDateTime &sessionTime, const bool sync)
    {
        step(beam, Phase::Acquiring, QString("%1 张卡同时采集%2...").arg(static_cast<int>(cards.size()))
             .arg(saveRoot ? QStringLiteral("（落盘）") : QStringLiteral("（不落盘）")));
        const auto measured = moved[0].result->measured();
        const auto world = measured ? std::optional{gmb.worldOf(*measured)} : std::nullopt;
        RunAwaiter<DAQFamily>::Orders orders;
        // 族基不是 QObject：会话目录的 ASCII 标识取同一对象上的 objectName（交叉转换必然成功）。
        // 参数逐卡经官邸 paramsFor 构造；任一张卡拒绝（原因由型号记录）即交空名单，等待器不挂起、返回空结果。
        for (DAQFamily &card : cards)
        {
            auto parameters = card.paramsFor(accumSec, world, saveRoot,
                AcqSaver::sessionDir(sessionTime, dynamic_cast<const QObject &>(card).objectName()), sync);
            if (!parameters)
                return RunAwaiter<DAQFamily>{lock, {}};
            orders.push_back({card, std::move(*parameters)});
        }
        return RunAwaiter<DAQFamily>{lock, std::move(orders)};
    }

private:
    enum class Phase { Idle, Moving, Acquiring };
    void step(const int beam, const Phase phase, const QString &message)
    {
        beam_ = beam;
        phase_ = phase;
        Q_EMIT logMessage(message);
        Q_EMIT runChanged();
    }
    [[nodiscard]] QString phaseText() const
    {
        if (!running()) return QStringLiteral("空闲");
        switch (phase_)
        {
        case Phase::Moving: return QStringLiteral("转台前往");
        case Phase::Acquiring: return QStringLiteral("采集中");
        case Phase::Idle: return QStringLiteral("准备中");
        }
        return {};
    }
    [[nodiscard]] QString beamText() const
    {
        return running() && beam_ >= 0 && beamCount() > 1
            ? QStringLiteral("第 %1/%2 束").arg(beam_ + 1).arg(beamCount()) : QString();
    }
    [[nodiscard]] QString deliveredText() const
    {
        return delivered_ > 0 ? QStringLiteral("%1 次").arg(delivered_) : QStringLiteral("—");
    }
    [[nodiscard]] QString scopeText() const { return running() ? scopeText_ : QStringLiteral("—"); }
    [[nodiscard]] QString sinceText() const
    {
        return running() ? startedAt_.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss")) : QStringLiteral("—");
    }
    [[nodiscard]] int currentBeam() const { return running() ? beam_ : -1; }

    static constexpr int kSettleMin = 2, kSettleMax = 600;
    // 登记表只存设备的 QObject 身份：机制对象先同步撤单，再由 QObject::destroyed 移除身份；
    // 两族族基与设备是同一对象，destroyed 时族基已析构，登记表只作指针比较、不解引用。
    std::vector<QObject *> gimbals_;
    std::vector<QObject *> cards_;
    int beam_ = -1, delivered_ = 0;
    Phase phase_ = Phase::Idle;
    QString scopeText_;
    QDateTime startedAt_;
    bool starting_ = false; // 急启动落槽期间，不是关停状态。
    std::unique_ptr<DeviceCoro> coro_; // 最先析构，覆盖基类自身状态的寿命。
};
