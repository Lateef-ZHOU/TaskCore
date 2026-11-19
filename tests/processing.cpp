#include <vector>
#include <string>
#include <atomic>
#include <cmath>
#include <numbers>
#include <functional>
#include "DAQFamily/DAQDevice.h"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QSemaphore>

// 包含 NUL 和高位字节，检查接收链按长度借用原地址，不把二进制输入当字符串或另复制一份。
static constexpr unsigned char inputBytes[] = {'f', 0, 0xff, 'm', 'e'};

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
    void logMessage(const std::string &) const override;
private:
    std::shared_ptr<Transport> connOpen(const Parameters &p) override { return p.transport; }
    bool connLoop(std::shared_ptr<Transport> c,
        std::function<void(std::shared_ptr<Transport>, std::span<const unsigned char>, RxTime)> sink) override
    {
        for (;;) {
            c->input.acquire();
            if (c->stopped.load()) return true;
            sink(c, inputBytes, std::chrono::system_clock::now());
        }
    }
    void connBreak(std::shared_ptr<Transport> c) const override
    {
        if (c) { c->stopped.store(true); c->input.release(); }
    }
    bool connSend(std::shared_ptr<Transport> c, const std::vector<unsigned char> &view) const override
    {
        const auto bytes = QByteArray::fromRawData(reinterpret_cast<const char *>(view.data()), static_cast<qsizetype>(view.size()));
        if (bytes == "start") c->input.release();
        return true;
    }
    RXRuling onRX(std::span<const unsigned char> view, RxTime at, std::shared_ptr<Transport>) override
    {
        return {view, RXRuling::Decode, at}; }
    TXRuling onTX(const std::vector<unsigned char> &payload, std::shared_ptr<Transport>) const override
    {
        return {{payload.begin(), payload.end()}, TXRuling::Encode}; }
};

// 一路两门、八个频点：每次进料即一整帧，峰在 4 MHz，零风速参考 3.5 MHz → 径向风 0.5 m/s。
// 测试型号取 2 μm：λ/2 = 1 m/s/MHz，便于手算。
inline constexpr DAQParams testChannelParams{
    .wavelengthM = 2e-6, .zeroWindMHz = 3.5,
    .rangeStartM = 100, .rangeStepM = 30,
    .frequencyStartMHz = 0, .frequencyStepMHz = 1,
    .rangeGateCount = 2, .frequencyBinCount = 8,
};
class TestCollector final : public TypedDAQCollector<DAQChannel<testChannelParams>>
{
public:
    explicit TestCollector(const DAQParameters &p) : TypedDAQCollector(p) {}
    std::vector<std::vector<unsigned char>> buildStart() const override { return {{'s', 't', 'a', 'r', 't'}}; }
    int timeoutMs() const override { return 0; }
    [[nodiscard]] QThread *processedOn() const noexcept { return processedOn_; }
private:
    // 每口料即一整帧：在收包线程解析与处理。
    std::optional<Frame> assemble(QByteArrayView bytes, RxTime at) override
    {
        if (bytes.data() != reinterpret_cast<const char *>(inputBytes)
            || bytes.size() != static_cast<qsizetype>(sizeof(inputBytes)) || bytes[1] != '\0'
            || static_cast<unsigned char>(bytes[2]) != 0xff)
            return std::nullopt;
        frameTime_ = at;
        processedOn_ = QThread::currentThread();
        auto values = std::make_unique<Spectrum<testChannelParams>>();
        for (int row = 0; row < testChannelParams.rangeGateCount; ++row)
            for (int column = 0; column < testChannelParams.frequencyBinCount; ++column)
                (*values)[row * testChannelParams.frequencyBinCount + column] = 1.0 + 100.0 * std::exp(-(column - 4.0) * (column - 4.0) / 2.0);
        return Frame{DAQChannel<testChannelParams>(std::move(values))};
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
        return DAQParameters(*this, accumSec, std::move(pose), std::move(saveRoot), std::move(sessionDir), sync);
    }
    std::vector<std::vector<unsigned char>> stopCommands() const override { return {}; }
Q_SIGNALS:
    void lockChanged(HandleCore::ResourcePhase) override;
    void channelsReady(const Channels &channels) override;
    void profilesReady(const Profiles &profiles) override;
private:
    void PreDestruction() noexcept final
    {
        static_assert(FinalDevice<TestDevice, decltype(&TestDevice::PreDestruction)>);
        DAQDevice::PreDestruction();
    }
};
struct FakeChannel {};
static_assert(DAQChannelLike<DAQChannel<testChannelParams>> && !DAQChannelLike<FakeChannel>);
static_assert(!std::derived_from<DAQFamily, QObject> && std::derived_from<TestDevice, TestHandle>);

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
    std::weak_ptr<const Spectrum<testChannelParams>> notifiedSpectrum;
    QObject::connect(&device, &TestDevice::channelsReady, &app, [&](const TestDevice::Channels &channels) {
        const auto &channel = std::get<0>(*channels);
        if (channel.spectrum->size() != 16 || QThread::currentThread() != app.thread()) qFatal("spectrum signal differs");
        if (channel.radialWind->size() != 2 || testChannelParams.rangeStartM != 100 || testChannelParams.rangeStepM != 30
            || channel.cnr->size() != 2 || std::abs((*channel.radialWind)[0] - 0.5) > 0.05)
            qFatal("standard radial processing differs");
        notifiedSpectrum = channel.spectrum;
        ++spectra;
        ++radials;
    });
    QObject::connect(&device, &TestDevice::profilesReady, &app, [&](const TestDevice::Profiles &result) {
        const auto &profile = std::get<0>(result);
        // 第 0 束 100 m 起、30 m 门距，60° 仰角：高度起点 100·sin60°、层距 30·sin60°。
        if (!profile || profile->size() != 2 || std::abs(profile->heightStartM() - 100 * std::sin(std::numbers::pi / 3)) > 1e-9
            || std::abs(profile->heightStepM() - 30 * std::sin(std::numbers::pi / 3)) > 1e-9) qFatal("wind profile differs");
        ++profiles;
    });
    if (!device.open(Parameters{std::make_shared<Transport>()}) || !until([&] { return device.isOpen(); })) return 1;
    // 四个方位各一帧，第四帧带同步位：前三帧不反演，第四帧对满环反演一次。
    for (int i = 0; i < 4; ++i) {
        const auto parameters = *device.paramsFor(1, WorldModel(i * 90.0, 60.0), std::nullopt, QStringLiteral("s"), i == 3);
        if (!device.begin(parameters, [&](auto frame) {
            const auto typed = std::dynamic_pointer_cast<const TestCollector>(frame);
            if (!typed || !typed->settled() || typed->processedOn() == app.thread() || QThread::currentThread() != app.thread())
                qFatal("frame was not processed on the receive thread before completion");
            if (std::get<0>(typed->channels()).spectrum != notifiedSpectrum.lock()) qFatal("signal copied the spectrum");
            ++completed;
        }, {}) || !until([&] { return completed == i + 1 && radials == i + 1; })) return 2;
        if (profiles != (i == 3 ? 1 : 0)) return 3;
    }
    // 越出型号范围的参数在 paramsFor 处拦下，造不出参数，也就到不了 begin。
    if (device.paramsFor(11, std::nullopt, std::nullopt, {}, false)) return 5;
    if (spectra != 4) return 4;
    device.startTest(11, false, {}, QStringLiteral("rejected"));
    if (device.holder()) return 6;
    // 有效参数进入单卡协程后跨多轮复用；停止仍沿原取消链释放本卡的锁。
    device.startTest(1, false, {}, QStringLiteral("single-card"));
    if (!device.holder() || !until([&] { return spectra >= 6; })) return 7;
    device.cancelTest();
    if (device.holder() || profiles != 1) return 8;
    return 0;
}
#include "processing.moc"
