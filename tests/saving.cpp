#include "DAQFamily/AcqSaver.h"
#include <array>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>
#include <QByteArray>
#include <QByteArrayView>
#include <QChar>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QIODevice>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QSaveFile>
#include <QString>
#include <QString>
#include <QTemporaryDir>
#include <QThread>
#include <QtLogging>

namespace {
int checks = 0;
std::mutex diagnosticGate;
std::vector<QString> diagnostics;

void captureDiagnostic(QtMsgType, const QMessageLogContext &, const QString &message)
{
    const std::lock_guard lock(diagnosticGate);
    diagnostics.push_back(message);
}
void check(bool ok, const char *message)
{
    ++checks;
    if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}

// 只等待测试自己提交的单个文件成为完整 JSON，不给生产保存器增加排空/回调接口。
QJsonObject readCompleted(const QString &path)
{
    QElapsedTimer elapsed;
    elapsed.start();
    while (elapsed.elapsed() < 5000) {
        QFile file(path);
        if (file.open(QIODevice::ReadOnly)) {
            const auto document = QJsonDocument::fromJson(file.readAll());
            if (document.isObject()) return document.object();
        }
        QThread::msleep(5);
    }
    check(false, "saved JSON did not complete");
    return {};
}

QString waitDiagnostic(const std::size_t index)
{
    QElapsedTimer elapsed;
    elapsed.start();
    while (elapsed.elapsed() < 5000) {
        {
            const std::lock_guard lock(diagnosticGate);
            if (diagnostics.size() > index) return diagnostics[index];
        }
        QThread::msleep(5);
    }
    check(false, "save failure diagnostic did not arrive");
    return {};
}

// 两路：A 正常换算；B 以 NaN 参考造出缺测风速，检查缺测写成 null。
inline constexpr DAQParams savedA{
    .wavelengthM = 1.55e-6, .zeroWindMHz = 1.5,
    .rangeStartM = 0, .rangeStepM = 15,
    .frequencyStartMHz = 0, .frequencyStepMHz = 1,
    .rangeGateCount = 2, .frequencyBinCount = 4,
};
inline constexpr DAQParams savedB{
    .wavelengthM = 1.55e-6, .zeroWindMHz = std::numeric_limits<double>::quiet_NaN(),
    .rangeStartM = 0, .rangeStepM = 15,
    .frequencyStartMHz = 0, .frequencyStepMHz = 1,
    .rangeGateCount = 2, .frequencyBinCount = 4,
};
inline constexpr DAQParams profileSpec{
    .wavelengthM = 1.55e-6, .zeroWindMHz = 1.5,
    .rangeStartM = 10, .rangeStepM = 10,
    .frequencyStartMHz = 0, .frequencyStepMHz = 1,
    .rangeGateCount = 4, .frequencyBinCount = 4,
};
class SavedFrame final : public TypedDAQCollector<DAQChannel<savedA>, DAQChannel<savedB>>
{
public:
    explicit SavedFrame(const DAQParameters &p) : TypedDAQCollector(p) {}
    std::vector<std::vector<unsigned char>> buildStart() const override { return {}; }
    int timeoutMs() const override { return 0; }
private:
    std::optional<Frame> assemble(QByteArrayView, RxTime at) override
    {
        frameTime_ = at;
        const std::array<double, 8> values{1, 2, 9, 2, 1, 3, 8, 3};
        auto a = std::make_unique<Spectrum<savedA>>();
        auto b = std::make_unique<Spectrum<savedB>>();
        std::copy(values.begin(), values.end(), a->begin());
        std::copy(values.begin(), values.end(), b->begin());
        return Frame{DAQChannel<savedA>(std::move(a), "A"), DAQChannel<savedB>(std::move(b), "B")};
    }
};
static_assert(DAQCollectorLike<SavedFrame>);

inline constexpr DAQParams singleBinSpec{
    .wavelengthM = 1.55e-6, .zeroWindMHz = 1.5,
    .rangeStartM = 0, .rangeStepM = 15,
    .frequencyStartMHz = 0, .frequencyStepMHz = 1,
    .rangeGateCount = 2, .frequencyBinCount = 1,
};
class SingleBinFrame final : public TypedDAQCollector<DAQChannel<singleBinSpec>>
{
public:
    explicit SingleBinFrame(const DAQParameters &p) : TypedDAQCollector(p) {}
    std::vector<std::vector<unsigned char>> buildStart() const override { return {}; }
    int timeoutMs() const override { return 0; }
private:
    std::optional<Frame> assemble(QByteArrayView, RxTime at) override
    {
        frameTime_ = at;
        auto values = std::make_unique<Spectrum<singleBinSpec>>();
        (*values)[0] = 3; (*values)[1] = 4;
        return Frame{DAQChannel<singleBinSpec>(std::move(values), "A")};
    }
};

template<class T>
concept SaveFrameAccepted = requires(std::shared_ptr<T> frame) {
    AcqSaver::saveFrame(std::move(frame), QString{});
};
static_assert(SaveFrameAccepted<const SavedFrame>);
static_assert(!SaveFrameAccepted<SavedFrame>);

// 存盘测试不经设备：自行派生通行证造参数（通行证只防误用，不防刻意派生）。
struct TestKey : DAQParamsKey { TestKey() {} };
}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QTemporaryDir temp;
    check(temp.isValid(), "temporary directory unavailable");
    const QDir root(temp.path());
    const auto at = std::chrono::system_clock::now();

    // 无朝向帧：朝向写 null，未标定通道的风速写 null，CNR 照常。
    auto collecting = std::make_shared<SavedFrame>(DAQParameters(TestKey(), 3, std::nullopt, root, QStringLiteral("session"), false));
    check(collecting->feed(QByteArray("x"), at), "frame did not complete");
    const std::shared_ptr<const SavedFrame> frame = std::move(collecting);
    AcqSaver::saveFrame(frame, QStringLiteral("Card"));
    const auto saved = readCompleted(root.filePath("session/" + AcqSaver::fileStem(at) + ".json"));
    const auto info = saved.value("frame").toObject();
    check(info.value("session").toString() == "session" && info.value("frameTime").toString() == AcqSaver::timeStem(at)
          && info.value("azimuthDeg").isNull() && info.value("elevationDeg").isNull()
          && !info.value("sync").toBool() && info.value("accumSec").toInt() == 3 && info.value("card").toString() == "Card",
          "frame information differs");
    const auto channels = saved.value("channels").toArray();
    check(channels.size() == 2, "channel count differs");
    const auto a = channels[0].toObject(), b = channels[1].toObject();
    check(a.value("name").toString() == "A" && b.value("name").toString() == "B", "channel order or names differ");
    check(a.value("rangeStartM").toDouble() == 0 && a.value("rangeStepM").toDouble() == 15
          && a.value("freqStartMHz").toDouble() == 0 && a.value("freqStepMHz").toDouble() == 1
          && a.value("zeroWindMHz").toDouble() == 1.5, "channel axes differ");
    const auto spectrum = a.value("spectrum").toArray();
    check(spectrum.size() == 2 && spectrum[0].toArray().size() == 4 && spectrum[1].toArray()[2].toDouble() == 8,
          "spectrum rows differ");
    check(a.value("radialWind").toArray().size() == 2 && !a.value("radialWind").toArray()[0].isNull(), "wind missing");
    check(b.value("radialWind").toArray()[0].isNull() && !b.value("cnr").toArray()[0].isNull(), "missing wind not null");

    // 频点不足仍保存原谱；旧格式中无法拟合的两个产物保持空数组。
    auto single = std::make_shared<SingleBinFrame>(DAQParameters(TestKey(), 1, std::nullopt, root, "single", false));
    check(single->feed(QByteArray("x"), at), "single-bin frame did not complete");
    AcqSaver::saveFrame(std::shared_ptr<const SingleBinFrame>(std::move(single)), "Card");
    const auto singleSaved = readCompleted(root.filePath("single/" + AcqSaver::fileStem(at) + ".json"))
        .value("channels").toArray()[0].toObject();
    check(singleSaved.value("spectrum").toArray()[1].toArray()[0].toDouble() == 4
        && singleSaved.value("radialWind").isArray() && singleSaved.value("radialWind").toArray().isEmpty()
        && singleSaved.value("cnr").isArray() && singleSaved.value("cnr").toArray().isEmpty(), "single-bin save format differs");

    // 风廓线：东风 5 m/s、上升 0.5 m/s → 来向 90°。
    auto filling = std::make_shared<VerticalWindProfileUnit<profileSpec>>(at, 90.0);
    (*filling)[0] = LinAlg::Vec3(-5, 0, 0.5);
    (*filling)[1] = LinAlg::Vec3(0, -3, 0);
    (*filling)[3] = LinAlg::Vec3::Zeros();
    const std::shared_ptr<const VerticalWindProfileUnit<profileSpec>> frozen = std::move(filling);
    const auto unit = frozen;
    AcqSaver::saveWindProfiles(frame, std::tuple{std::shared_ptr<const VerticalWindProfileUnit<profileSpec>>{}, unit});
    const auto profile = readCompleted(root.filePath("session/profile_" + AcqSaver::fileStem(at) + ".json"));
    const auto first = profile.value("channels").toArray()[0].toObject();
    check(profile.value("frame").toObject().value("session").toString() == "session"
          && first.value("name").toString() == "B" && first.value("heightStartM").toDouble() == 10
          && first.value("heightStepM").toDouble() == 10 && first.value("horizontalSpeed").toArray().size() == 4, "profile layout differs");
    check(std::abs(first.value("horizontalDirectionDeg").toArray()[0].toDouble() - 90.0) < 1e-3
          && std::abs(first.value("horizontalSpeed").toArray()[0].toDouble() - 5.0) < 1e-3
          && std::abs(first.value("verticalSpeed").toArray()[0].toDouble() - 0.5) < 1e-3
          && std::abs(first.value("horizontalDirectionDeg").toArray()[1].toDouble() - 0.0) < 1e-3,
          "profile direction or speeds differ");

    for (const auto *key : {"horizontalDirectionDeg", "horizontalSpeed", "verticalSpeed"})
        check(first.value(key).toArray()[2].isNull(), "missing layer must remain JSON null");
    check(first.value("horizontalSpeed").toArray()[3].toDouble(-1) == 0
          && first.value("verticalSpeed").toArray()[3].toDouble(-1) == 0, "calm layer must remain zero");

    // 原 Collector 的最后份额应在调用线程释放；保存线程只持上下文、同步时间和结果。
    // 用销毁线程身份验证，避免以写盘速度推断生命周期。
    const auto caller = std::this_thread::get_id();
    std::atomic<bool> destroyedHere = false;
    std::shared_ptr<SavedFrame> source(new SavedFrame(DAQParameters(TestKey(), 7,
        WorldModel(12, 30, 0), root, QStringLiteral("released"), true)),
        [caller, &destroyedHere](SavedFrame *value) {
            destroyedHere.store(std::this_thread::get_id() == caller);
            delete value;
        });
    check(source->feed(QByteArray("x"), at), "lifetime frame did not complete");
    std::shared_ptr<const SavedFrame> sourceFrame = std::move(source);
    const std::weak_ptr<const SavedFrame> sourceWeak = sourceFrame;
    auto result = std::make_shared<VerticalWindProfileUnit<profileSpec>>(at + std::chrono::seconds(10), 90.0);
    (*result)[0] = LinAlg::Vec3(-5, 0, 0.5);
    (*result)[1] = LinAlg::Vec3(0, -3, 0);
    (*result)[3] = LinAlg::Vec3::Zeros();
    std::shared_ptr<const VerticalWindProfileUnit<profileSpec>> onlyResult = std::move(result);
    AcqSaver::saveWindProfiles(std::move(sourceFrame), std::tuple{decltype(onlyResult){}, std::move(onlyResult)});
    check(sourceWeak.expired() && destroyedHere.load(), "profile saver retained Collector or destroyed it off-thread");
    const auto independent = readCompleted(root.filePath("released/profile_" + AcqSaver::fileStem(at) + ".json"));
    const auto independentInfo = independent.value("frame").toObject();
    check(independentInfo.size() == 6 && independentInfo.value("session").toString() == "released"
          && independentInfo.value("frameTime").toString() == AcqSaver::timeStem(at)
          && independentInfo.value("azimuthDeg").toDouble() == 12
          && independentInfo.value("elevationDeg").toDouble() == 30
          && independentInfo.value("sync").toBool() && independentInfo.value("accumSec").toInt() == 7,
          "profile saver lost acquisition context or used result time instead of synchronization time");
    check(independent.value("channels") == profile.value("channels"), "direct profile traversal changed saved values");
    QFile exact(root.filePath("released/profile_" + AcqSaver::fileStem(at) + ".json"));
    check(exact.open(QIODevice::ReadOnly), "cannot read exact saved profile");
    check(exact.readAll().endsWith("\"horizontalDirectionDeg\":[90,0,null,180],\"horizontalSpeed\":[5,3,null,0],\"verticalSpeed\":[0.5,0,null,0]}]}"),
          "profile JSON field order, numeric format or missing/calm values changed");

    // 整帧保存也只捕获结果份额；Collector 在调用线程释放后，原数组仍可写出。
    std::atomic<bool> frameDestroyedHere = false;
    std::shared_ptr<SavedFrame> savingSource(new SavedFrame(DAQParameters(TestKey(), 3,
        std::nullopt, root, QStringLiteral("released-frame"), false)),
        [caller, &frameDestroyedHere](SavedFrame *value) {
            frameDestroyedHere.store(std::this_thread::get_id() == caller);
            delete value;
        });
    check(savingSource->feed(QByteArray("x"), at), "independent save frame did not complete");
    std::shared_ptr<const SavedFrame> savingFrame = std::move(savingSource);
    const std::weak_ptr<const SavedFrame> savingWeak = savingFrame;
    AcqSaver::saveFrame(std::move(savingFrame), QStringLiteral("Card"));
    check(savingWeak.expired() && frameDestroyedHere.load(), "frame saver retained Collector or destroyed it off-thread");
    const auto savedIndependent = readCompleted(root.filePath("released-frame/" + AcqSaver::fileStem(at) + ".json"));
    auto expectedInfo = info;
    expectedInfo.insert("session", "released-frame");
    check(savedIndependent.value("frame").toObject() == expectedInfo
          && savedIndependent.value("channels") == saved.value("channels"),
          "frame saver lost context, channel order or original result values");
    QFile originalFile(root.filePath("session/" + AcqSaver::fileStem(at) + ".json"));
    QFile independentFile(root.filePath("released-frame/" + AcqSaver::fileStem(at) + ".json"));
    check(originalFile.open(QIODevice::ReadOnly) && independentFile.open(QIODevice::ReadOnly), "cannot compare saved frame bytes");
    check(originalFile.readAll().replace("\"session\":\"session\"", "\"session\":\"released-frame\"") == independentFile.readAll(),
          "frame JSON field order, numeric format or missing values changed");

    // 不落盘：saveRoot 为空时入口直接返回。
    AcqSaver::saveFrame(std::make_shared<const SavedFrame>(DAQParameters(TestKey(), 1, std::nullopt, std::nullopt, {}, false)),
                        QStringLiteral("Card"));

    // 字符串字段往返保留全部 JSON 控制字符、引号、反斜杠和 UTF-8；不改变普通帧格式。
    QString cardName = QString::fromUtf8("采集卡 \"A\" \\");
    for (ushort code = 0; code < 0x20; ++code) cardName.append(QChar(code));
    auto escapedFrame = std::make_shared<SavedFrame>(DAQParameters(TestKey(), 1, std::nullopt, root, "escaped", false));
    check(escapedFrame->feed(QByteArray("x"), at), "escape frame did not complete");
    AcqSaver::saveFrame(std::shared_ptr<const SavedFrame>(std::move(escapedFrame)), cardName);
    const QString escapedPath = root.filePath("escaped/" + AcqSaver::fileStem(at) + ".json");
    check(readCompleted(escapedPath).value("frame").toObject().value("card").toString() == cardName,
          "JSON string controls, quotes, backslashes or UTF-8 did not round-trip");
    QFile escapedFile(escapedPath);
    check(escapedFile.open(QIODevice::ReadOnly), "cannot inspect escaped JSON");
    const auto escapedBytes = escapedFile.readAll();
    for (char code = 0; code < 0x20; ++code)
        check(!escapedBytes.contains(code), "raw control character leaked into JSON");

    // 用普通文件阻挡目录创建，用目录占据目标文件名；跨平台稳定触发两种失败，不依赖权限配置。
    const auto previousHandler = qInstallMessageHandler(captureDiagnostic);
    QFile obstacle(root.filePath("obstacle"));
    check(obstacle.open(QIODevice::WriteOnly), "cannot create directory obstacle");
    obstacle.close();
    auto blockedFrame = std::make_shared<SavedFrame>(DAQParameters(TestKey(), 1, std::nullopt, root, "obstacle/session", false));
    check(blockedFrame->feed(QByteArray("x"), at), "blocked frame did not complete");
    AcqSaver::saveFrame(std::shared_ptr<const SavedFrame>(std::move(blockedFrame)), "Card");
    const auto directoryFailure = waitDiagnostic(0);
    check(directoryFailure.contains("Cannot create acquisition directory:")
          && directoryFailure.contains(root.filePath("obstacle/session"))
          && directoryFailure.contains(AcqSaver::fileStem(at) + ".json"), "directory failure lacks stage or full path");

    const auto occupiedPath = root.filePath("occupied/profile_" + AcqSaver::fileStem(at) + ".json");
    check(QDir().mkpath(occupiedPath), "cannot create filename obstacle");
    auto occupiedFrame = std::make_shared<SavedFrame>(DAQParameters(TestKey(), 1, std::nullopt, root, "occupied", true));
    check(occupiedFrame->feed(QByteArray("x"), at), "occupied frame did not complete");
    // 从同一 QSaveFile 路径取得平台自己的错误文案，不硬编码英文或本地化系统信息。
    QSaveFile expectedFailure(occupiedPath);
    check(!expectedFailure.open(QIODevice::WriteOnly), "directory unexpectedly opened as output file");
    AcqSaver::saveWindProfiles(std::shared_ptr<const SavedFrame>(std::move(occupiedFrame)), std::tuple{unit});
    const auto fileFailure = waitDiagnostic(1);
    check(fileFailure.contains("Cannot open acquisition file:") && fileFailure.contains(occupiedPath)
          && fileFailure.contains(expectedFailure.errorString()), "file failure lacks path or concrete error");
    qInstallMessageHandler(previousHandler);
    std::cout << "saving checks passed: " << checks << '\n';
    return 0;
}
