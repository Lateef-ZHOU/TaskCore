#include "Device/DAQ/DAQDevice.h"
#include <atomic>
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

class TestHandle final : public Handle<Parameters, Transport>
{
    Q_OBJECT
public:
    using Handle::Handle;
    ~TestHandle() override { stop(); }
Q_SIGNALS:
    void connectionChanged(bool) override;
    void logMessage(const QString &) const override;
private:
    std::shared_ptr<Transport> connOpen(const Parameters &p) override { return p.transport; }
    bool connLoop(std::shared_ptr<Transport> c, Sink sink) override
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
    { return {RXRuling::Decode, raw, at}; }
    TXRuling onTX(const QByteArray &raw, std::shared_ptr<Transport>) const override
    { return {TXRuling::Encode, raw}; }
};

class TestCollector final : public DAQCollector
{
public:
    static std::shared_ptr<const TestCollector> createPrototype(int, DAQContext c)
    { return std::shared_ptr<const TestCollector>(new TestCollector(std::move(c))); }
    std::shared_ptr<TestCollector> makeSelf() const
    { return std::shared_ptr<TestCollector>(new TestCollector(context())); }
    bool settled() const override { return complete_; }
    QList<QByteArray> buildStart() const override { return {"start"}; }
    int timeoutMs() const override { return 0; }
private:
    explicit TestCollector(DAQContext c) : DAQCollector(std::move(c)) {}
    void onFeed(const QByteArray &, RxTime at) override { frameTime_ = at; complete_ = true; }
    bool complete_ = false;
};

class TestDevice final : public DAQDevice<TestHandle, TestCollector>
{
    Q_OBJECT
public:
    TestDevice() : DAQDevice("test", {}, nullptr) { startProcessing(); }
    ~TestDevice() override { PreDestruction(); }
Q_SIGNALS:
    void processed(int sequence, bool onWorker);
private:
    void run() override
    {
        consumeFrames([this, sequence = 0](const std::shared_ptr<const TestCollector> &frame) mutable {
            if (!frame || !frame->settled()) qFatal("invalid completed frame");
            Q_EMIT processed(++sequence, QThread::currentThread() == this);
        });
    }
    void PreDestruction() noexcept final { DAQDevice::PreDestruction(); }
};

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
    int received = 0, completed = 0;
    QObject::connect(&device, &TestDevice::processed, &app, [&](int sequence, bool onWorker) {
        if (!onWorker || QThread::currentThread() != app.thread() || sequence != received + 1)
            qFatal("persistent closure or cross-thread signal delivery failed");
        received = sequence;
    });
    if (!device.device().open({std::make_shared<Transport>()}) || !until([&] { return device.device().isOpen(); })) return 1;
    for (int i = 1; i <= 3; ++i) {
        if (!device.begin(device.collectorFor(1, {}), [&](auto frame) {
            if (!frame || QThread::currentThread() != app.thread()) qFatal("wrong completion thread");
            ++completed;
        }, {}) || !until([&] { return completed == i && received == i; })) return 2;
    }
    device.stopWorker();
    // 已关闭的线程不再消费，也不再引用原锚点；机制完成仍可安全交付。
    if (!device.begin(device.collectorFor(1, {}), [&](auto frame) { if (frame) ++completed; }, {})
        || !until([&] { return completed == 4; }) || received != 3) return 3;
    return 0;
}
#include "processing.moc"
