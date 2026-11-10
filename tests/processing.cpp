#include <atomic>
#include <cmath>
#include <numbers>
#include <functional>
#include "DAQDevice.h"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QSemaphore>

struct Transport
{
    QSemaphore input;
    std::atomic<bool> stopped{false};
};
struct Parameters
{
    std::shared_ptr<Transport> transport;
    bool operator==(const Parameters &) const = default;
};

class TestHandle : public Handle<Parameters, Transport>
{
    Q_OBJECT
public:
    using Handle::Handle;
    ~TestHandle() override = default;
Q_SIGNALS:
    void connectionChanged(HandleCore::ResourcePhase) override;
    void logMessage(const QString &) const override;
private:
    std::shared_ptr<Transport> connOpen(const Parameters &p) override { return p.transport; }
    bool connLoop(std::shared_ptr<Transport> c,
        std::function<void(std::shared_ptr<Transport>, const QByteArray &, RxTime)> sink) override
    {
        for (;;) {
            c->input.acquire();
            if (c->stopped.load()) return true;
            sink(c, QByteArray("frame"), std::chrono::system_clock::now());
        }
    }
    void connBreak(std::shared_ptr<Transport> c) const override
    {
        if (c) { c->stopped.store(true); c->input.release(); }
    }
    bool connSend(std::shared_ptr<Transport> c, const QByteArray &bytes) const override
    {
        if (bytes == "start") c->input.release();
        return true;
    }
    RXRuling onRX(const QByteArray &raw, RxTime at, std::shared_ptr<Transport>) override
    { return {raw, RXRuling::Decode, at}; }
    TXRuling onTX(const QByteArray &raw, std::shared_ptr<Transport>) const override
    { return {raw, TXRuling::Encode}; }
};

// 一路两门、八个频点：每次进料即一整帧，峰在 4 MHz，零风速参考 3.5 MHz → 径向风 0.5 m/s。
class TestCollector final : public DAQCollector
{
public:
    static std::shared_ptr<TestCollector> create(const DAQParameters &p)
    { return std::shared_ptr<TestCollector>(new TestCollector(p)); }
    std::shared_ptr<TestCollector> makeSelf() const { return create(parameters()); }
    QList<QByteArray> buildStart() const override { return {"start"}; }
    int timeoutMs() const override { return 0; }
    [[nodiscard]] QThread *processedOn() const noexcept { return processedOn_; }
private:
    // 测试型号取 2 μm：λ/2 = 1 m/s/MHz，便于手算。
    explicit TestCollector(DAQParameters p) : DAQCollector(std::move(p), {channel()}, 2e-6) {}
    static DAQChannel channel()
    {
        DAQChannel c;
        c.name = QStringLiteral("T");
        c.spectrum = LinearArray2D<double>(2, 8, 100, 30, 0, 1);
        c.zeroWindMHz = 3.5;
        return c;
    }
    void onFeed(const std::optional<QByteArray> &, RxTime at) override
    {
        auto &c = front();
        c.spectrum.clear();
        for (int row = 0; row < c.spectrum.rows; ++row)
            for (int column = 0; column < c.spectrum.columns; ++column)
                c.spectrum.push_back(1.0 + 100.0 * std::exp(-(column - 4.0) * (column - 4.0) / 2.0));
        frameTime_ = at;
        processedOn_ = QThread::currentThread();
        complete();
    }
    QThread *processedOn_ = nullptr;
};

class TestDevice final : public DAQDevice<TestHandle, TestCollector>
{
    Q_OBJECT
public:
    TestDevice() : DAQDevice("test", {}, nullptr) {}
    ~TestDevice() override { PreDestruction(); }
    // 测试型号接受 1~10 秒。
    std::optional<DAQParameters> paramsFor(int accumSec, std::optional<WorldModel> pose, std::optional<QDir> saveRoot,
                                           QString sessionDir, bool sync) const override
    {
        if (accumSec < 1 || accumSec > 10) return std::nullopt;
        return DAQParameters{accumSec, std::move(pose), std::move(saveRoot), std::move(sessionDir), sync};
    }
Q_SIGNALS:
    void lockChanged(HandleCore::ResourcePhase) override;
    void spectrumReady(int channel, const LinearArray2D<double> &spectrum) override;
    void radialReady(int channel, const LinearArray<double> &speeds,
                     const LinearArray<double> &cnr, bool calibrated) override;
    void profileReady(int channel, VerticalWindProfilePtr profile) override;
private:
    void PreDestruction() noexcept final
    {
        static_assert(FinalDevice<TestDevice, decltype(&TestDevice::PreDestruction)>);
        DAQDevice::PreDestruction();
    }
};
static_assert(!std::derived_from<DAQDeviceBase, QObject> && std::derived_from<TestDevice, TestHandle>);

template<class Predicate> bool until(Predicate predicate)
{
    QElapsedTimer timer;
    timer.start();
    while (!predicate() && timer.elapsed() < 3000) {
        QCoreApplication::processEvents();
        QThread::msleep(1);
    }
    return predicate();
}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    TestDevice device;
    int spectra = 0, radials = 0, profiles = 0, completed = 0;
    QObject::connect(&device, &TestDevice::spectrumReady, &app, [&](int channel, const LinearArray2D<double> &values) {
        if (channel != 0 || values.size() != 16 || QThread::currentThread() != app.thread()) qFatal("spectrum signal differs");
        ++spectra;
    });
    QObject::connect(&device, &TestDevice::radialReady, &app,
                     [&](int channel, const LinearArray<double> &speeds, const LinearArray<double> &cnr, bool calibrated) {
        if (channel != 0 || speeds.size() != 2 || speeds.start != 100 || speeds.step != 30 || !cnr.sameAxis(speeds) || cnr.size() != 2 || !calibrated
            || std::abs(speeds[0] - 0.5) > 0.05)
            qFatal("standard radial processing differs");
        ++radials;
    });
    QObject::connect(&device, &TestDevice::profileReady, &app, [&](int channel, VerticalWindProfilePtr profile) {
        // 第 0 束 100 m 起、30 m 门距，60° 仰角：高度起点 100·sin60°、层距 30·sin60°。
        if (channel != 0 || !profile || profile->winds.size() != 2 || std::abs(profile->winds.start - 100 * std::sin(std::numbers::pi / 3)) > 1e-9
            || std::abs(profile->winds.step - 30 * std::sin(std::numbers::pi / 3)) > 1e-9) qFatal("wind profile differs");
        ++profiles;
    });
    if (!device.open(Parameters{std::make_shared<Transport>()}) || !until([&] { return device.isOpen(); })) return 1;
    // 四个方位各一帧，第四帧带同步位：前三帧不反演，第四帧对满环反演一次。
    for (int i = 0; i < 4; ++i) {
        const DAQParameters parameters{1, WorldModel(i * 90.0, 60.0), std::nullopt, QStringLiteral("s"), i == 3};
        if (!device.begin(parameters, [&](auto frame) {
            const auto typed = std::dynamic_pointer_cast<const TestCollector>(frame);
            if (!typed || !typed->settled() || typed->processedOn() == app.thread() || QThread::currentThread() != app.thread())
                qFatal("frame was not processed on the receive thread before completion");
            ++completed;
        }, {}) || !until([&] { return completed == i + 1 && radials == i + 1; })) return 2;
        if (profiles != (i == 3 ? 1 : 0)) return 3;
    }
    // 越出型号范围的参数在 begin 复核时拒收，不发启动命令。
    if (device.begin(DAQParameters{11}, [](auto) { qFatal("out-of-range request completed"); }, {})) return 5;
    return spectra == 4 ? 0 : 4;
}
#include "processing.moc"
